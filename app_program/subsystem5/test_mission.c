/**
 * @file    test_mission.c
 * @brief   On-target tests for the tick counters and the 1 us time base.
 *          S5-U04 (K4 flood) and S5-U05 (state x event matrix) changed
 *          live system state, so they run on the host instead
 *          (testing/subsystem5/host/sim_mission_host.c, fix brief Fix 2/3).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    test
 * @reqs    NFR17, NFR10b (time base)
 */

#include "test_mission.h"      /* own header first (BARR-C 4.3.c) */
#include "../test_report.h"
#include "sys_tick.h"

#if APP_TEST

/* ---- private constants and types ------------------------------------ */

#define TEST_BUSY_PRI          (11)     /* above every periodic task   */
#define TEST_BUSY_US           (2000u)  /* S5-U06: 2 ms busy loop      */
#define TEST_BUSY_ROUNDS       (40u)
#define TEST_BUSY_DLY_A_MS     (1)      /* wakes 2 ticks later         */
#define TEST_BUSY_DLY_B_MS     (11)     /* wakes 3 ticks later         */
#define TEST_BUSY_WAIT_MS      (1500)
#define TEST_LAT_MIN_US        (1500u)
#define TEST_BUSY_STACK_BYTES  (512u + APP_SYS_STACK_BYTES)
#define TEST_TIMER_DLY_MS      (1000)   /* S5-U29: 1 s kernel delay    */
#define TEST_TIMER_MIN_US      (990000u)
#define TEST_TIMER_MAX_US      (1020000u)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_test_busy_stack[TEST_BUSY_STACK_BYTES / 8u];

/* ---- private function prototypes ------------------------------------- */

static void test_mission_timer(void);
static void test_mission_latency(void);
static void test_busy_task(INT stacd, void * p_exinf);

#endif /* APP_TEST */

/* ---- public functions ------------------------------------------------- */

/**
 * test_mission_run
 * @brief   Runs S5-U29 (time base) and S5-U06 (latency). Touches no
 *          shared state (no faults, modes, safety bits or motion).
 * @context usermain (TEST=1 builds, priority 21)
 */
void test_mission_run(void)
{
#if APP_TEST
    test_mission_timer();
    test_mission_latency();
#endif
}

#if APP_TEST

/* ---- private functions ------------------------------------------------ */

/**
 * test_mission_timer
 * @brief   S5-U29 (fix brief, Fix 4): sys_tick_now_us() advances about
 *          1 000 000 us across a 1 s kernel delay, so the watchdog tick
 *          that drives the TIMER is running.
 * @context usermain
 */
static void test_mission_timer(void)
{
    uint32_t start_us = 0u;
    uint32_t diff_us  = 0u;

    TEST_INIT();
    start_us = sys_tick_now_us();
    (void) tk_dly_tsk(TEST_TIMER_DLY_MS);   /* cannot fail */
    diff_us  = sys_tick_now_us() - start_us;

    TEST_CHECK("S5-U29", (diff_us >= TEST_TIMER_MIN_US)
               && (diff_us <= TEST_TIMER_MAX_US),
               "timer advances ~1 000 000 us in 1 s");
    app_log("S5-U29 timer diff %d us\n",
            (int32_t) diff_us, 0, 0);   /* cast: < 2^31 */
    TEST_SUMMARY("time base");
}

/**
 * test_mission_latency
 * @brief   S5-U06: a priority-11 task that busy-loops 2 ms at tick
 *          boundaries delays the periodic tasks, and the latency counter
 *          shows it.
 * @context usermain
 */
static void test_mission_latency(void)
{
    uint32_t const before_us = sys_tick_latency_max_us();
    uint32_t       after_us  = 0u;
    ID             tid       = 0;

    TEST_INIT();
    tid = app_start_task((FP) test_busy_task, /* cast: kernel entry type */
                         TEST_BUSY_PRI, g_test_busy_stack,
                         TEST_BUSY_STACK_BYTES, "test_busy");
    (void) tk_dly_tsk(TEST_BUSY_WAIT_MS);  /* busy task runs and exits */
    after_us = sys_tick_latency_max_us();

    TEST_CHECK("S5-U06", tid > 0, "busy task started");
    TEST_CHECK("S5-U06", after_us > before_us, "latency counter moved");
    TEST_CHECK("S5-U06", after_us >= TEST_LAT_MIN_US,
               "latency shows the 2 ms delay");
    app_log("S5-U06 latency before %d after %d us\n",
            (int32_t) before_us, (int32_t) after_us, 0); /* cast: <= 4095 */
    TEST_SUMMARY("mission latency");
}

/**
 * test_busy_task
 * @brief   Wakes on a tick, busy-loops 2 ms (deliberately longer than the
 *          20 us task rule: this is the load S5-U06 needs), and alternates
 *          2- and 3-tick sleeps so it lines up with every R3 phase.
 * @param   stacd   unused (kernel start code)
 * @param   p_exinf unused
 * @context test_busy task (priority 11, TEST=1 only)
 */
static void test_busy_task(INT stacd, void * p_exinf)
{
    uint32_t start_us = 0u;

    (void) stacd;    /* unused: kernel start code */
    (void) p_exinf;  /* unused: no extended info  */

    for (uint32_t idx = 0u; idx < TEST_BUSY_ROUNDS; idx++)
    {
        if (0u == (idx & 1u))
        {
            (void) tk_dly_tsk(TEST_BUSY_DLY_A_MS);  /* cannot fail */
        }
        else
        {
            (void) tk_dly_tsk(TEST_BUSY_DLY_B_MS);  /* cannot fail */
        }
        start_us = sys_tick_now_us();
        while ((sys_tick_now_us() - start_us) < TEST_BUSY_US)
        {
            /* busy: simulated load */
        }
    }
    tk_ext_tsk();
}

#endif /* APP_TEST */

/*** end of file ***/
