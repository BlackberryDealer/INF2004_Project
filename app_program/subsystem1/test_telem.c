/**
 * @file    test_telem.c
 * @brief   On-target tests of telemetry (TEST=1 builds only):
 *          S1-U03 (a full K11 drops and counts, nothing blocks) and an
 *          S1-U12 smoke check (remote commands never block). The exact
 *          S1-U12 counts are checked on the laptop against the real
 *          mqtt_bridge.c (testing/subsystem1/host), because the command
 *          drop counter has no getter in the frozen contract.
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    FR10, FR11, NFR8
 */

#include "test_telem.h"        /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#include "../app_types.h"
#include "telem.h"
#include "mqtt_bridge.h"
#include "../subsystem5/sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define TEST_TEL_FILL_MAX      (16u)    /* more than K11 can ever hold   */
#define TEST_TEL_WAIT_MS       (250u)   /* > one 200 ms state slot       */
#define TEST_TEL_NO_BLOCK_US   (1000u)  /* a TMO_POL call is far faster  */
#define TEST_TEL_CMD_GAP_MS    (50u)

/* A valid heartbeat payload (baseline: route byte + JSON text). */
static uint8_t const g_test_payload[] =
{
    (uint8_t) APP_MQ_TYPE_HEARTBEAT,
    '{', '"', 'u', 'p', '"', ':', '0', '}'
};

static uint8_t const g_test_stop[] =
{
    '{', '"', 'c', 'm', 'd', '"', ':', '"', 'S', 'T', 'O', 'P', '"', '}'
};
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_telem_run
 * @brief   Runs every on-target test of telem and prints the [TEST]
 *          summary line. Empty unless built with TEST=1. Precondition for
 *          S1-U03: runs before the MQTT link is up (500 ms after boot),
 *          so the radio task is not draining K11.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_telem_run(void)
{
#if APP_TEST
    uint8_t  buf[MQTT_BRIDGE_TX_MAX] = {0u};
    uint8_t  mq_type     = 0u;
    uint32_t queued      = 0u;
    uint32_t drained     = 0u;
    uint32_t drops_0     = 0u;
    uint32_t start_us    = 0u;
    uint32_t took_us     = 0u;
    ER       ercd        = E_OK;
    bool     b_is_full   = false;

    TEST_INIT();

    /* S1-U03: fill K11 the way telem_task does (TMO_POL) until it is full */
    while ((!b_is_full) && (queued < TEST_TEL_FILL_MAX))
    {
        start_us = sys_tick_now_us();
        ercd     = tk_snd_mbf(telem_k11_id(), g_test_payload,
                              (INT) sizeof(g_test_payload), /* cast: 9 */
                              TMO_POL);
        took_us  = sys_tick_now_us() - start_us;
        if (E_OK == ercd)
        {
            queued++;
        }
        else
        {
            b_is_full = true;
        }
    }
    TEST_CHECK("S1-U03", b_is_full && (queued <= 8u),
               "K11 holds at most 8 messages; the next send fails");
    TEST_CHECK("S1-U03", took_us < TEST_TEL_NO_BLOCK_US,
               "the refused send returned at once (nothing blocks)");

    drops_0 = telem_drops();
    (void) tk_dly_tsk(TEST_TEL_WAIT_MS);   /* telem_task queues a state */
    TEST_CHECK("S1-U03", telem_drops() >= (drops_0 + 1u),
               "telem_task's payload into a full K11 is counted");

    /* empty K11 again so the test payloads are never published */
    while (mqtt_bridge_take(buf, sizeof(buf), &mq_type) > 0u)
    {
        drained++;
    }
    TEST_CHECK("S1-U03", drained >= queued,
               "mqtt_bridge_take drains every queued payload");

    /* S1-U12 smoke check: two STOPs 50 ms apart never block the caller */
    start_us = sys_tick_now_us();
    mqtt_bridge_post_command(g_test_stop, sizeof(g_test_stop));
    took_us = sys_tick_now_us() - start_us;
    (void) tk_dly_tsk(TEST_TEL_CMD_GAP_MS);
    start_us = sys_tick_now_us();
    mqtt_bridge_post_command(g_test_stop, sizeof(g_test_stop));
    took_us = took_us + (sys_tick_now_us() - start_us);
    TEST_CHECK("S1-U12", took_us < TEST_TEL_NO_BLOCK_US,
               "two remote STOPs return at once (counts: host test)");

    TEST_SUMMARY("telem");
#endif
}

/*** end of file ***/
