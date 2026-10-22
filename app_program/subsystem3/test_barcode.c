/**
 * @file    test_barcode.c
 * @brief   Phase 1 on-target barcode edge-timing bench harness (S3-I07).
 * @owner   Buddy 3 (line, barcode)
 * @kind    test
 * @reqs    FR2, NFR5
 *
 * Kernel side only (no SDK headers): PIN_BAR_DO is owned by Buddy 5's
 * drv_gpio_irq (R1) and read through the kernel's in_w(GPIO_IN).
 * When TEST_BARCODE_ENABLE_BENCH is 1, attaches a bench callback to
 * drv_gpio_irq on PIN_BAR_DO and logs edge timestamps while pushing the
 * robot over sample barcodes (B3.1.2 S3-I07).
 */

#include "test_barcode.h"      /* own header first (BARR-C 4.3.c) */
#include "../app.h"

#if (0 != APP_TEST)
#include "../board_pins.h"
#include "../subsystem5/drv_gpio_irq.h"
#include "../test_report.h"
#include "barcode.h"
#include "cfg_barcode.h"
#endif

/* ---- private constants and types ------------------------------------ */

#if (0 != APP_TEST)
#define TEST_BAR_PIN_MASK     (1u << PIN_BAR_DO)
#define TEST_BAR_ENABLE_BENCH (0)
#define TEST_BAR_BENCH_EDGES  (30u)
#define TEST_BAR_BENCH_POLL_MS (20)
#endif

/* ---- private data ---------------------------------------------------- */

#if (0 != APP_TEST) && (0 != TEST_BAR_ENABLE_BENCH)
static volatile uint32_t g_bench_edge_cnt = 0u;
static volatile uint32_t g_bench_last_us  = 0u;
static volatile uint32_t g_bench_dt_us    = 0u;
static volatile uint32_t g_bench_level    = 0u;

static void test_barcode_bench_isr(uint32_t events, uint32_t now_us)
{
    bool const b_pin_high = (0u != (in_w(GPIO_IN) & TEST_BAR_PIN_MASK));

    (void) events;
    g_bench_dt_us   = now_us - g_bench_last_us;
    g_bench_last_us = now_us;
    g_bench_level   = (BAR_DO_BLACK_IS_HIGH == b_pin_high) ? 1u : 0u;
    g_bench_edge_cnt++;
}
#endif

/* ---- private function prototypes ------------------------------------- */

#if (0 != APP_TEST)
static void test_barcode_edge_bench(void);
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_barcode_run
 * @brief   Runs Phase 1 on-target barcode checks and optional S3-I07 bench
 *          edge logger.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
void test_barcode_run(void)
{
#if (0 != APP_TEST)
    bool const     b_pin_high = (0u != (in_w(GPIO_IN) & TEST_BAR_PIN_MASK));
    uint32_t const is_black   = (BAR_DO_BLACK_IS_HIGH == b_pin_high) ? 1u : 0u;
    uint32_t const seq0       = barcode_branch_seq();

    TEST_INIT();

    app_log("S3-I07 PIN_BAR_DO raw=%d black=%d seq=%d\n",
            (int32_t) b_pin_high, /* cast: bool 0/1 fits int32_t */
            (int32_t) is_black,   /* cast: 0/1 fits int32_t */
            (int32_t) seq0        /* cast: 8-bit seq fits int32_t */);

    test_barcode_edge_bench();

    TEST_CHECK("S3-I07",
               (0u == seq0) &&
               (E_PAR == drv_gpio_irq_attach(PIN_BAR_DO, DRV_GPIO_IRQ_BOTH,
                                             NULL)),
               "barcode branch seq starts at 0 and R1 validates PIN_BAR_DO");
    TEST_SUMMARY("barcode");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if (0 != APP_TEST)
/**
 * test_barcode_edge_bench
 * @brief   When TEST_BAR_ENABLE_BENCH != 0, attaches a callback via
 *          drv_gpio_irq_attach(PIN_BAR_DO, DRV_GPIO_IRQ_BOTH, ...) and
 *          logs edge intervals for S3-I07 bench characterization.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
static void test_barcode_edge_bench(void)
{
#if (0 != TEST_BAR_ENABLE_BENCH)
    uint32_t seen = 0u;
    ER const ercd = drv_gpio_irq_attach(PIN_BAR_DO, DRV_GPIO_IRQ_BOTH,
                                        test_barcode_bench_isr);

    if (E_OK == ercd)
    {
        while (seen < TEST_BAR_BENCH_EDGES)
        {
            uint32_t const cur = g_bench_edge_cnt;

            if (cur != seen)
            {
                seen = cur;
                app_log("S3-I07 edge=%d dt_us=%d level=%d\n",
                        (int32_t) seen,           /* cast: count fits int32_t */
                        (int32_t) g_bench_dt_us,  /* cast: dt fits int32_t */
                        (int32_t) g_bench_level   /* cast: 0/1 fits int32_t */);
            }
            else
            {
                /* Wait for next barcode edge. */
            }
            (void) tk_dly_tsk(TEST_BAR_BENCH_POLL_MS);
        }
    }
    else
    {
        /* PIN_BAR_DO already attached by barcode_start. */
    }
#endif
}
#endif /* APP_TEST */

/*** end of file ***/
