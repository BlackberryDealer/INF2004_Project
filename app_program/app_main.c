/**
 * @file    app_main.c
 * @brief   usermain() boot order, static task creation and the console,
 *          error-check, timing-probe and stack-measurement helpers.
 * @owner   Team (integrator; change only by SHARED pull request)
 * @kind    task
 * @reqs    NFR11 (stack free space), NFR13 (template + BARR-C), FR17
 *          (no task drives a motor before every kernel object exists)
 *
 * Boot sequence (Part D.15 of the team spec; nobody else changes it):
 *   1. board_pins_check()             pin clash and GP0/GP1 function check
 *   2. every <module>_init()          create all kernel objects first
 *   3. every <module>_start()         then start the tasks, mission last
 *   4. TEST=1 only: lower usermain to APP_PRI_TEST and run every suite
 *   5. tk_slp_tsk(TMO_FEVR)           usermain must never return
 *
 * This is the only application file allowed to call tm_printf() directly.
 */

#include "app_main.h"          /* own header first (BARR-C 4.3.c) */
#include "app.h"
#include "board_pins.h"
#include <tm/tmonitor.h>       /* tm_printf: console output */

#if (APP_PROBE != 0)
#include <bsp/libbsp.h>        /* kernel BSP GPIO: GP14 timing probe */
#endif

#include "subsystem1/telem.h"
#include "subsystem1/test_telem.h"
#include "subsystem2/safety.h"
#include "subsystem2/fault.h"
#include "subsystem2/motion.h"
#include "subsystem2/test_motion.h"
#include "subsystem3/line.h"
#include "subsystem3/barcode.h"
#include "subsystem3/test_line.h"
#include "subsystem3/test_barcode.h"
#include "subsystem4/imu.h"
#include "subsystem4/test_imu.h"
#include "subsystem5/status_board.h"
#include "subsystem5/mission.h"
#include "subsystem5/sys_tick.h"
#include "subsystem5/sys_health.h"
#include "subsystem5/drv_gpio_irq.h"
#include "subsystem5/obstacle.h"
#include "subsystem5/test_mission.h"
#include "subsystem5/test_obstacle.h"

/* ---- private constants and types ------------------------------------ */

#define APP_TASK_MAX             (8u)    /* seven tasks + one spare       */
#define APP_STACK_WORD_BYTES     (8u)    /* stacks are uint64_t arrays    */
#define APP_STACK_PATTERN        (0xA5A5A5A5A5A5A5A5u)
#define APP_PERCENT              (100u)
#define APP_TEST_START_DELAY_MS  (500u)  /* let every task start first    */

/* One row per task started through app_start_task(). */
typedef struct
{
    uint64_t const * p_stack;
    uint32_t         stack_bytes;
    char const *     p_name;
} app_task_entry_t;

/* ---- private data ---------------------------------------------------- */

static app_task_entry_t g_task_table[APP_TASK_MAX] = {0};
static uint32_t         g_task_count               = 0u;
static bool             gb_is_table_full_logged    = false;

#if (APP_PROBE != 0)
static bool             gb_is_probe_ready          = false;
#endif

/* ---- private function prototypes ------------------------------------- */

static void     app_table_add(uint64_t const * p_stack, uint32_t stack_bytes,
                              char const * p_name);
static uint32_t app_stack_free_bytes(app_task_entry_t const * p_entry);

/* ---- public functions ------------------------------------------------- */

/**
 * app_start_task
 * @brief   Creates and starts a task on a caller-owned static stack
 *          (TA_USERBUF), after filling that stack with a known pattern so
 *          app_stack_report() can measure how much of it was never used.
 * @param   p_entry      task entry function, cast to the kernel FP type
 * @param   priority     initial priority, one of APP_PRI_*
 * @param   p_stack      static uint64_t array (8-byte aligned)
 * @param   stack_bytes  size of p_stack in bytes, incl. 128 B system part
 * @param   p_name       task name for the boot log
 * @return  task ID (> 0) or a negative kernel error code
 * @context usermain (via the <module>_start() functions)
 */
ID app_start_task(FP p_entry, PRI priority, uint64_t * p_stack,
                  uint32_t stack_bytes, char const * p_name)
{
    T_CTSK   ctsk       = {0};
    ID       tid        = E_PAR;
    ER       ercd       = E_OK;
    uint32_t word_count = stack_bytes / APP_STACK_WORD_BYTES;
    uint32_t idx        = 0u;

    if ((NULL != p_stack) && (NULL != p_entry))
    {
        for (idx = 0u; idx < word_count; idx++)
        {
            p_stack[idx] = APP_STACK_PATTERN;
        }

        ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
        ctsk.task    = p_entry;
        ctsk.itskpri = priority;
        ctsk.stksz   = (SZ) stack_bytes;  /* cast: <= 2176 B, fits SZ   */
        ctsk.bufptr  = (void *) p_stack;  /* cast: kernel buffer type   */

        tid = tk_cre_tsk(&ctsk);
        app_check(tid, p_name);
    }
    else
    {
        app_check(E_PAR, p_name);
    }

    if (E_OK < tid)
    {
        ercd = tk_sta_tsk(tid, 0);
        app_check(ercd, p_name);

        if (E_OK == ercd)
        {
            app_table_add(p_stack, stack_bytes, p_name);
        }
        else
        {
            tid = ercd;
        }
    }

    return tid;
}

/**
 * app_check
 * @brief   Prints "ERROR <what>: <code>" if a kernel call failed.
 * @param   ercd    kernel return value (negative = error)
 * @param   p_what  short description of the call that was checked
 * @return  none
 * @context any task (never an ISR or handler)
 */
void app_check(ER ercd, char const * p_what)
{
    if (E_OK > ercd)
    {
        (void) tm_printf((UB const *) "ERROR %s: %d\n", /* cast: T-Monitor */
                         p_what, ercd);         /* takes UB*; printing    */
    }                                           /* cannot fail usefully   */
}

/**
 * app_log
 * @brief   One console line through T-Monitor. Up to three %d arguments.
 * @param   p_format  format string (only %d conversions)
 * @param   arg_a     first %d argument (ignored if unused)
 * @param   arg_b     second %d argument (ignored if unused)
 * @param   arg_c     third %d argument (ignored if unused)
 * @return  none
 * @context any task; never an ISR, handler, or the radio task in RUN
 */
void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    /* Casts: T-Monitor takes UB*; %d reads an INT, which has the same
       32-bit width as int32_t on the RP2040. The return value is only a
       character count, so it is ignored.                                */
    (void) tm_printf((UB const *) p_format, (INT) arg_a, (INT) arg_b,
                     (INT) arg_c);
}

/**
 * app_probe_set
 * @brief   Drives the GP14 timing probe. Does nothing unless the image
 *          was built with PROBE=1.
 * @param   b_is_high  true = probe pin high
 * @return  none
 * @context any task or ISR (PROBE=1 builds only; GPIO set/clear is atomic)
 */
void app_probe_set(bool b_is_high)
{
#if (APP_PROBE != 0)
    /* Kernel BSP GPIO (SIO): the SDK's gpio.h cannot be included next to
       the kernel headers. Both calls only fail for a pin >= 30.        */
    if (!gb_is_probe_ready)
    {
        (void) gpio_set_pin(PIN_PROBE, GPIO_MODE_OUT);
        gb_is_probe_ready = true;
    }
    (void) gpio_set_val(PIN_PROBE, b_is_high ? 1u : 0u);
#else
    (void) b_is_high;   /* unused: probe compiled out (PROBE=0) */
#endif
}

/**
 * app_stack_report
 * @brief   Prints the unused (never written) bytes of every task stack.
 * @return  none
 * @context any task (prints; reads other stacks without locking, which is
 *          safe because the pattern words are only ever overwritten)
 */
void app_stack_report(void)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < g_task_count; idx++)
    {
        uint32_t const free_bytes =
            app_stack_free_bytes(&g_task_table[idx]);

        app_log("stack %d free %d of %d: ",
                (int32_t) idx,               /* cast: < APP_TASK_MAX     */
                (int32_t) free_bytes,        /* cast: <= stack size      */
                (int32_t) g_task_table[idx].stack_bytes); /* cast: same  */
        (void) tm_printf((UB const *) "%s\n", /* cast: T-Monitor UB*     */
                         g_task_table[idx].p_name);
    }
}

/**
 * app_stack_free_pct_min
 * @brief   Smallest free-stack percentage over every task (NFR11c).
 * @return  0..100; 100 if no task has been started yet
 * @context any task
 */
uint32_t app_stack_free_pct_min(void)
{
    uint32_t pct_min = APP_PERCENT;
    uint32_t idx     = 0u;

    for (idx = 0u; idx < g_task_count; idx++)
    {
        uint32_t const total = g_task_table[idx].stack_bytes;
        uint32_t const pct   =
            (app_stack_free_bytes(&g_task_table[idx]) * APP_PERCENT) / total;

        if (pct < pct_min)
        {
            pct_min = pct;
        }
    }

    return pct_min;
}

/**
 * usermain
 * @brief   Application entry point, called by the kernel's initial task at
 *          priority 1. Creates every kernel object, starts every task, runs
 *          the self-tests in TEST=1 builds, then sleeps forever.
 * @return  never returns (returning would shut the kernel down)
 * @context initial task
 */
INT usermain(void)
{
    T_RVER ver = {0};

    (void) tk_ref_ver(&ver);   /* cannot fail with a valid pointer */
    (void) tm_printf((UB const *) /* cast: T-Monitor takes UB* */
                     "\n=== CSC28 PicoCar on micro T-Kernel"
                     " (spec 0x%04x, product 0x%04x) ===\n",
                     ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    app_log("dual-core (SMP) build\n", 0, 0, 0);
#else
    app_log("single-core build\n", 0, 0, 0);
#endif

    /* 1. Pins */
    board_pins_check();

    /* 2. Every kernel object exists before any task runs */
    app_check(status_board_init(), "status_board_init");
    app_check(fault_init(), "fault_init");
    app_check(safety_init(), "safety_init");
    app_check(mission_init(), "mission_init");
    app_check(sys_tick_init(), "sys_tick_init");
    app_check(sys_health_init(), "sys_health_init");
    app_check(drv_gpio_irq_init(), "drv_gpio_irq_init");
    app_check(motion_init(), "motion_init");
    app_check(line_init(), "line_init");
    app_check(barcode_init(), "barcode_init");
    app_check(imu_init(), "imu_init");
    app_check(obstacle_init(), "obstacle_init");
    app_check(telem_init(), "telem_init");

    /* 3. Start the tasks; mission (the watchdog feeder) last */
    app_check(motion_start(), "motion_start");
    app_check(line_start(), "line_start");
    app_check(barcode_start(), "barcode_start");
    app_check(imu_start(), "imu_start");
    app_check(obstacle_start(), "obstacle_start");
    app_check(telem_start(), "telem_start");
    app_check(sys_tick_start(), "sys_tick_start");
    app_check(mission_start(), "mission_start");

    app_log("%d tasks started\n",
            (int32_t) g_task_count, 0, 0);   /* cast: <= APP_TASK_MAX */

#if APP_TEST
    /* 4. Self-tests. Lower usermain below every application task first,
          so the suites cannot starve mission_task (the watchdog feeder). */
    (void) tk_dly_tsk(APP_TEST_START_DELAY_MS); /* E_OK or released early */
    app_check(tk_chg_pri(TSK_SELF, APP_PRI_TEST), "tk_chg_pri");

    app_log("[TEST] === test run start ===\n", 0, 0, 0);
    test_telem_run();
    test_motion_run();
    test_line_run();
    test_barcode_run();
    test_imu_run();
    test_obstacle_run();
    test_mission_run();
    app_log("[TEST] === test run end ===\n", 0, 0, 0);

    app_stack_report();
#endif

    /* 5. Sleep forever: returning from usermain() shuts the kernel down */
    (void) tk_slp_tsk(TMO_FEVR);   /* never woken */

    return 0;
}

/* ---- private functions ------------------------------------------------ */

/**
 * app_table_add
 * @brief   Remembers a started task's stack for the stack report. When the
 *          table is full the task still runs; it is only left out of the
 *          report, and that is logged once.
 * @param   p_stack      task stack (first = lowest address)
 * @param   stack_bytes  stack size in bytes
 * @param   p_name       task name
 * @return  none
 * @context usermain
 */
static void app_table_add(uint64_t const * p_stack, uint32_t stack_bytes,
                          char const * p_name)
{
    if (g_task_count < APP_TASK_MAX)
    {
        g_task_table[g_task_count].p_stack     = p_stack;
        g_task_table[g_task_count].stack_bytes = stack_bytes;
        g_task_table[g_task_count].p_name      = p_name;
        g_task_count++;
    }
    else if (!gb_is_table_full_logged)
    {
        app_log("stack table full: task not measured\n", 0, 0, 0);
        gb_is_table_full_logged = true;
    }
    else
    {
        /* already logged once */
    }
}

/**
 * app_stack_free_bytes
 * @brief   Counts untouched pattern words from the lowest stack address
 *          upward (the stack grows down, so they are the unused part).
 * @param   p_entry  table row of the task
 * @return  bytes never written, 0..stack_bytes
 * @context any task
 */
static uint32_t app_stack_free_bytes(app_task_entry_t const * p_entry)
{
    uint32_t const word_count = p_entry->stack_bytes / APP_STACK_WORD_BYTES;
    uint32_t       free_words = 0u;
    bool           b_is_untouched = true;

    while (b_is_untouched && (free_words < word_count))
    {
        if (APP_STACK_PATTERN == p_entry->p_stack[free_words])
        {
            free_words++;
        }
        else
        {
            b_is_untouched = false;
        }
    }

    return (free_words * APP_STACK_WORD_BYTES);
}

/*** end of file ***/
