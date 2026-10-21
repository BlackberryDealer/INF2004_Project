/**
 * @file    test_mission.c
 * @brief   On-target tests for the mission queue, the statechart and the
 *          tick counters.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    test
 * @reqs    min-bit item 7, NFR17, B5.2.6 (every state x every event)
 */

#include "test_mission.h"      /* own header first (BARR-C 4.3.c) */
#include "../test_report.h"
#include "../app_types.h"
#include "../subsystem2/fault.h"
#include "mission.h"
#include "sys_tick.h"
#include "mission_run.h"

#if APP_TEST

/* ---- private constants and types ------------------------------------ */

#define TEST_K4_ENTRIES        (16u)    /* baseline K4 depth           */
#define TEST_K4_BYTES          (256u)   /* baseline K4 buffer          */
#define TEST_FLOOD_POSTS       (20u)    /* more than K4 can hold       */
#define TEST_POST_MAX_US       (200u)   /* a failed post must not wait */
#define TEST_DRAIN_MS          (300)    /* mission empties K4          */
#define TEST_BUSY_PRI          (11)     /* above every periodic task   */
#define TEST_BUSY_US           (2000u)  /* S5-U06: 2 ms busy loop      */
#define TEST_BUSY_ROUNDS       (40u)
#define TEST_BUSY_DLY_A_MS     (1)      /* wakes 2 ticks later         */
#define TEST_BUSY_DLY_B_MS     (11)     /* wakes 3 ticks later         */
#define TEST_BUSY_WAIT_MS      (1500)
#define TEST_LAT_MIN_US        (1500u)
#define TEST_BUSY_STACK_BYTES  (512u + APP_SYS_STACK_BYTES)
/* (7 states + 4 RUN substates + 7 AVOID steps) x 24 event types */
#define TEST_MATRIX_CASES      (432u)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_test_busy_stack[TEST_BUSY_STACK_BYTES / 8u];

/* ---- private function prototypes ------------------------------------- */

static void test_mission_flood(void);
static void test_mission_matrix(void);
static void test_mission_latency(void);
static void test_busy_task(INT stacd, void * p_exinf);

#endif /* APP_TEST */

/* ---- public functions ------------------------------------------------- */

/**
 * test_mission_run
 * @brief   Runs S5-U04, S5-U05 and S5-U06.
 * @context usermain (TEST=1 builds, priority 1)
 */
void test_mission_run(void)
{
#if APP_TEST
    test_mission_flood();
    test_mission_matrix();
    test_mission_latency();
#endif
}

#if APP_TEST

/* ---- private functions ------------------------------------------------ */

/**
 * test_mission_flood
 * @brief   S5-U04: flooding K4 fails without blocking and sets
 *          APP_FAULT_QUEUE_OVERFLOW. usermain (priority 1) posts, so
 *          mission_task (16) cannot drain K4 during the flood.
 * @context usermain
 */
static void test_mission_flood(void)
{
    uint32_t sent       = 0u;
    uint32_t failed     = 0u;
    uint32_t worst_us   = 0u;
    uint32_t start_us   = 0u;
    uint32_t took_us    = 0u;
    bool     b_was_sent = false;

    TEST_INIT();
    fault_clear(APP_FAULT_QUEUE_OVERFLOW);   /* test build only */

    for (uint32_t idx = 0u; idx < TEST_FLOOD_POSTS; idx++)
    {
        start_us   = sys_tick_now_us();
        b_was_sent = mission_post_event(APP_EVT_SUMMARY, idx);
        took_us    = sys_tick_now_us() - start_us;
        if (b_was_sent)
        {
            sent++;
        }
        else
        {
            failed++;
            if (took_us > worst_us)
            {
                worst_us = took_us;
            }
        }
    }

    TEST_CHECK("S5-U04", TEST_K4_ENTRIES == sent, "K4 holds 16 events");
    TEST_CHECK("S5-U04", (TEST_FLOOD_POSTS - TEST_K4_ENTRIES) == failed,
               "extra posts are refused");
    TEST_CHECK("S5-U04", worst_us < TEST_POST_MAX_US,
               "a refused post returns at once");
    TEST_CHECK("S5-U04",
               0u != (fault_get() & APP_FAULT_QUEUE_OVERFLOW),
               "QUEUE_OVERFLOW fault set");
    TEST_CHECK("S5-U04", TEST_K4_BYTES == mission_queue_high_water(),
               "high-water mark = whole buffer");
    app_log("S5-U04 worst refused post %d us\n",
            (int32_t) worst_us, 0, 0);   /* cast: < 2^31 */

    (void) tk_dly_tsk(TEST_DRAIN_MS);       /* let mission empty K4 */
    fault_clear(APP_FAULT_QUEUE_OVERFLOW);  /* test build only */
    TEST_SUMMARY("mission K4");
}

/**
 * test_mission_matrix
 * @brief   S5-U05: every state x every event type gives a defined state
 *          and no crash. Wheels must be off the ground (it issues motion
 *          commands).
 * @context usermain
 */
static void test_mission_matrix(void)
{
    uint32_t cases = 0u;
    uint32_t bad   = 0u;

    TEST_INIT();
    bad = mission_test_matrix(&cases);
    TEST_CHECK("S5-U05", TEST_MATRIX_CASES == cases,
               "432 state/event combinations tried");
    TEST_CHECK("S5-U05", 0u == bad, "every outcome is a defined state");
    /* casts: both < 1000 */
    app_log("S5-U05 cases %d undefined %d\n", (int32_t) cases,
            (int32_t) bad, 0);
    TEST_SUMMARY("mission matrix");
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
