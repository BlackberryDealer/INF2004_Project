/**
 * @file    test_barcode.c
 * @brief   On-target unit tests for code39 and barcode_map_set (S3-U03..U08).
 * @owner   Buddy 3 (line, barcode)
 * @kind    test
 * @reqs    FR2, FR3, NFR5
 */

#include "test_barcode.h"      /* own header first (BARR-C 4.3.c) */
#include "../app.h"

#if (0 != APP_TEST)

#include "../test_report.h"
#include "barcode.h"
#include "cfg_barcode.h"
#include "code39.h"

/* ---- private constants and types ------------------------------------ */

#define TEST_NARROW_W (20u)
#define TEST_WIDE_W   (50u)
#define TEST_CHAR_A   (10u)

/* ---- private data ---------------------------------------------------- */

/* '*A*' forward widths: '*' (0x094), gap, 'A' (0x109), gap, '*' (0x094) */
static uint8_t const g_test_star_a_star[BAR_ELEMENTS] =
{
    /* '*' = 010010100 */
    TEST_NARROW_W, TEST_WIDE_W,   TEST_NARROW_W,
    TEST_NARROW_W, TEST_WIDE_W,   TEST_NARROW_W,
    TEST_WIDE_W,   TEST_NARROW_W, TEST_NARROW_W,
    /* inter-char gap */
    TEST_NARROW_W,
    /* 'A' = 100001001 */
    TEST_WIDE_W,   TEST_NARROW_W, TEST_NARROW_W,
    TEST_NARROW_W, TEST_NARROW_W, TEST_WIDE_W,
    TEST_NARROW_W, TEST_NARROW_W, TEST_WIDE_W,
    /* inter-char gap */
    TEST_NARROW_W,
    /* '*' = 010010100 */
    TEST_NARROW_W, TEST_WIDE_W,   TEST_NARROW_W,
    TEST_NARROW_W, TEST_WIDE_W,   TEST_NARROW_W,
    TEST_WIDE_W,   TEST_NARROW_W, TEST_NARROW_W
};

/* ---- private function prototypes ------------------------------------- */

#if (0 != APP_TEST)
static void test_barcode_edge_bench(void);
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_barcode_run
 * @brief   Runs on-target barcode decode and map validation checks.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
void test_barcode_run(void)
{
    uint32_t char_idx = 0u;
    uint32_t rc_ok    = 0u;
    uint32_t rc_short = 0u;

    TEST_INIT();

    rc_ok    = code39_decode(g_test_star_a_star, BAR_ELEMENTS, &char_idx);
    rc_short = code39_decode(g_test_star_a_star, 28u, &char_idx);

    TEST_CHECK("S3-U03", (CODE39_OK == rc_ok) && (TEST_CHAR_A == char_idx),
               "code39 decodes *A* to index 10");
    TEST_CHECK("S3-U05", (CODE39_INVALID == rc_short),
               "28-element capture rejected");

    /* S3-U08: out-of-range arguments rejected, valid set accepted. */
    barcode_map_set(43u, APP_BAR_CMD_LEFT);
    barcode_map_set(TEST_CHAR_A, 4u);
    barcode_map_set(TEST_CHAR_A, APP_BAR_CMD_LEFT);
    TEST_CHECK("S3-U08", (0u == barcode_branch_seq()),
               "barcode_map_set boundary calls safe");

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
