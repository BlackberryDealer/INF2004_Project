/**
 * @file    test_line.c
 * @brief   On-target unit tests for line_track (S3-U01, U02, U06, U14).
 * @owner   Buddy 3 (line, barcode)
 * @kind    test
 * @reqs    FR1, FR19, NFR1, NFR4
 */

#include "test_line.h"         /* own header first (BARR-C 4.3.c) */
#include "../app.h"

#if (0 != APP_TEST)
#include "../app_bits.h"
#include "../app_layout.h"
#include "../test_report.h"
#include "cfg_line.h"
#include "line.h"
#include "line_track.h"

/* ---- private constants and types ------------------------------------ */

#define TEST_CAL_LO        (500u)
#define TEST_CAL_HI        (3500u)
#define TEST_CAL_MID       (2000u)
#define TEST_MIN_COUNTS    (29u)
#define TEST_CONF_COUNTS   (86u)
#define TEST_RAW_SAMPLES   (16u)    /* 160 ms during stationary CAL_IMU */
#define TEST_RAW_DELAY_MS  (10)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static bool     test_line_early_branch(void);
static bool     test_line_late_branch(void);
static uint32_t test_line_isqrt(uint32_t val);
static bool     test_line_raw_stats_smoke(void);

#endif /* APP_TEST */

/* ---- public functions ------------------------------------------------- */

/**
 * test_line_run
 * @brief   Runs on-target unit checks for line_track.
 * @return  none
 * @context usermain (APP_PRI_TEST)
 */
void test_line_run(void)
{
#if (0 != APP_TEST)
    line_track_cal_t cal    = {0u, 0u, 0u, 0u, 0u, 0u};
    bool             b_ok   = false;
    uint32_t         norm_w = 0u;
    uint32_t         norm_b = 0u;
    uint32_t         norm_m = 0u;

    TEST_INIT();

    b_ok = line_track_cal_finish(&cal, TEST_CAL_LO, TEST_CAL_HI,
                                 TEST_CAL_HI, TEST_CAL_LO);
    TEST_CHECK("S3-U06", b_ok, "cal_finish accepts both polarities");

    norm_w = line_track_normalise(TEST_CAL_LO, cal.white_l, cal.black_l,
                                  cal.recip_l_q16);
    norm_b = line_track_normalise(TEST_CAL_HI, cal.white_l, cal.black_l,
                                  cal.recip_l_q16);
    norm_m = line_track_normalise(TEST_CAL_MID, cal.white_l, cal.black_l,
                                  cal.recip_l_q16);

    TEST_CHECK("S3-U01", (0u == norm_w) && (1000u == norm_b),
               "exact endpoints 0 and 1000");
    TEST_CHECK("S3-U01", (500u == norm_m), "midpoint normalises to 500");
    TEST_CHECK("S3-U01", (1000 == line_track_steering(1000u, 0u)) &&
                         (-1000 == line_track_steering(0u, 1000u)),
               "steering sign convention +left -right");
    TEST_CHECK("S3-U14", test_line_early_branch(),
               "early barcode branch + right sat classifies T");
    TEST_CHECK("S3-U02", test_line_late_branch(),
               "late barcode branch + right sat + line present -> CROSS");
    TEST_CHECK("S3-U56", test_line_raw_stats_smoke(),
               "SH_IR_RAW stationary ADC sampling & std-dev under US ping");

    TEST_SUMMARY("line");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if (0 != APP_TEST)
/**
 * test_line_early_branch
 * @brief   Verifies S3-U14 early barcode branch + right sat + white -> CROSS.
 * @return  true if classified as APP_JCT_CROSS, else false.
 * @context usermain (APP_PRI_TEST)
 */
static bool test_line_early_branch(void)
{
    line_track_jct_t    jct     = {0u, 0u, false, false, 0u};
    line_track_jct_in_t step_in = {0u, 0u, 0u, 0u, TEST_MIN_COUNTS,
                                   TEST_CONF_COUNTS, false};
    uint32_t            cls     = APP_JCT_NONE;

    line_track_jct_reset(&jct);
    step_in.norm_l          = 500u;
    step_in.norm_r          = 950u;
    step_in.path_counts     = 100u;
    step_in.branch_seq      = 1u;
    step_in.b_branch_recent = true;
    (void) line_track_jct_step(&jct, &step_in);

    step_in.norm_l          = 100u;
    step_in.norm_r          = 100u;
    step_in.path_counts     = 140u;
    step_in.b_branch_recent = false;
    (void) line_track_jct_step(&jct, &step_in);

    step_in.path_counts = 230u;
    cls                 = line_track_jct_step(&jct, &step_in);

    return (APP_JCT_CROSS == cls);
}

/**
 * test_line_late_branch
 * @brief   Verifies S3-U02 late barcode branch + right sat + line -> CROSS.
 * @return  true if classified as APP_JCT_CROSS, else false.
 * @context usermain (APP_PRI_TEST)
 */
static bool test_line_late_branch(void)
{
    line_track_jct_t    jct     = {0u, 0u, false, false, 0u};
    line_track_jct_in_t step_in = {0u, 0u, 0u, 0u, TEST_MIN_COUNTS,
                                   TEST_CONF_COUNTS, false};
    uint32_t            cls     = APP_JCT_NONE;

    line_track_jct_reset(&jct);
    step_in.norm_l      = 500u;
    step_in.norm_r      = 950u;
    step_in.path_counts = 300u;
    step_in.branch_seq  = 1u;
    (void) line_track_jct_step(&jct, &step_in);

    step_in.norm_l      = 500u;
    step_in.norm_r      = 500u;
    step_in.path_counts = 340u;
    (void) line_track_jct_step(&jct, &step_in);

    step_in.branch_seq  = 2u;
    step_in.path_counts = 430u;
    cls                 = line_track_jct_step(&jct, &step_in);

    return (APP_JCT_CROSS == cls);
}

/**
 * test_line_isqrt
 * @brief   Integer floor(sqrt(val)) for S3-U56 / S3-I06 std-dev check.
 * @param   val  Input variance in ADC codes squared.
 * @return  Floor square root.
 * @context usermain (APP_PRI_TEST)
 */
static uint32_t test_line_isqrt(uint32_t val)
{
    uint32_t rem  = val;
    uint32_t root = 0u;
    uint32_t bit  = 1u << 30u;

    while (bit > rem)
    {
        bit >>= 2u;
    }

    while (0u != bit)
    {
        if (rem >= (root + bit))
        {
            rem  -= (root + bit);
            root  = (root >> 1u) + bit;
        }
        else
        {
            root >>= 1u;
        }
        bit >>= 2u;
    }

    return root;
}

/**
 * test_line_raw_stats_smoke
 * @brief   Samples SH_IR_RAW across 16 line_task cycles while obstacle_task
 *          pings and logs boot min/max/std-dev for S3-I05 / S3-I06.
 * @return  true if isqrt math and all 12-bit ADC samples are valid.
 * @context usermain (APP_PRI_TEST)
 */
static bool test_line_raw_stats_smoke(void)
{
    bool     b_ok  = (0u == test_line_isqrt(0u))
                     && (10u == test_line_isqrt(100u));
    uint32_t idx   = 0u;
    uint32_t min_l = 4095u;
    uint32_t max_l = 0u;
    uint32_t min_r = 4095u;
    uint32_t max_r = 0u;
    uint32_t sum_l = 0u;
    uint32_t sum_r = 0u;
    uint32_t raw_l[TEST_RAW_SAMPLES] = {0u};
    uint32_t raw_r[TEST_RAW_SAMPLES] = {0u};

    for (idx = 0u; idx < TEST_RAW_SAMPLES; idx++)
    {
        uint32_t const word   = line_raw_word();
        uint32_t const code_l = app_bits_get(word, SH_IR_RAW_L_POS,
                                             SH_IR_RAW_L_BITS);
        uint32_t const code_r = app_bits_get(word, SH_IR_RAW_R_POS,
                                             SH_IR_RAW_R_BITS);

        raw_l[idx] = code_l;
        raw_r[idx] = code_r;
        sum_l     += code_l;
        sum_r     += code_r;
        min_l      = (code_l < min_l) ? code_l : min_l;
        max_l      = (code_l > max_l) ? code_l : max_l;
        min_r      = (code_r < min_r) ? code_r : min_r;
        max_r      = (code_r > max_r) ? code_r : max_r;
        /* casts: SH_IR_RAW_L/R_MAX are positive 12-bit layout bounds (4095) */
        if ((code_l > (uint32_t) SH_IR_RAW_L_MAX)
            || (code_r > (uint32_t) SH_IR_RAW_R_MAX))
        {
            b_ok = false;
        }
        (void) tk_dly_tsk(TEST_RAW_DELAY_MS);
    }

    {
        uint32_t const mean_l = sum_l / TEST_RAW_SAMPLES;
        uint32_t const mean_r = sum_r / TEST_RAW_SAMPLES;
        uint32_t       var_l  = 0u;
        uint32_t       var_r  = 0u;

        for (idx = 0u; idx < TEST_RAW_SAMPLES; idx++)
        {
            uint32_t const diff_l = (raw_l[idx] > mean_l)
                                    ? (raw_l[idx] - mean_l)
                                    : (mean_l - raw_l[idx]);
            uint32_t const diff_r = (raw_r[idx] > mean_r)
                                    ? (raw_r[idx] - mean_r)
                                    : (mean_r - raw_r[idx]);

            var_l += (diff_l * diff_l);
            var_r += (diff_r * diff_r);
        }
        var_l /= TEST_RAW_SAMPLES;
        var_r /= TEST_RAW_SAMPLES;

        /* casts: 12-bit ADC stats fit int32_t */
        app_log("[S3-I05/I06 boot] 16-smp L %d..%d sd %d\n",
                (int32_t) min_l, (int32_t) max_l,
                (int32_t) test_line_isqrt(var_l));
        app_log("[S3-I05/I06 boot] 16-smp R %d..%d sd %d\n",
                (int32_t) min_r, (int32_t) max_r,
                (int32_t) test_line_isqrt(var_r));
    }

    return b_ok;
}
#endif /* APP_TEST */

/*** end of file ***/
