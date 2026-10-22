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
#include "../test_report.h"
#include "cfg_line.h"
#include "line_track.h"

/* ---- private constants and types ------------------------------------ */

#define TEST_CAL_LO        (500u)
#define TEST_CAL_HI        (3500u)
#define TEST_CAL_MID       (2000u)
#define TEST_MIN_COUNTS    (29u)
#define TEST_CONF_COUNTS   (86u)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static bool test_line_early_branch(void);
static bool test_line_late_branch(void);

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
               "early barcode branch + left sat classifies T");
    TEST_CHECK("S3-U02", test_line_late_branch(),
               "late barcode branch + left sat + line present -> CROSS");

    TEST_SUMMARY("line");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if (0 != APP_TEST)
/**
 * test_line_early_branch
 * @brief   Verifies S3-U14 early barcode branch + left sat + white -> T.
 * @return  true if classified as APP_JCT_T, else false.
 * @context usermain (APP_PRI_TEST)
 */
static bool test_line_early_branch(void)
{
    line_track_jct_t    jct = {0u, 0u, false, false, 0u};
    line_track_jct_in_t in  = {0u, 0u, 0u, 0u, TEST_MIN_COUNTS,
                               TEST_CONF_COUNTS, false};
    uint32_t            cls = APP_JCT_NONE;

    line_track_jct_reset(&jct);
    in.norm_l          = 950u;
    in.norm_r          = 500u;
    in.path_counts     = 100u;
    in.branch_seq      = 1u;
    in.b_branch_recent = true;
    (void) line_track_jct_step(&jct, &in);

    in.norm_l          = 100u;
    in.norm_r          = 100u;
    in.path_counts     = 140u;
    in.b_branch_recent = false;
    (void) line_track_jct_step(&jct, &in);

    in.path_counts = 230u;
    cls            = line_track_jct_step(&jct, &in);

    return (APP_JCT_T == cls);
}

/**
 * test_line_late_branch
 * @brief   Verifies S3-U02 late barcode branch + left sat + line -> CROSS.
 * @return  true if classified as APP_JCT_CROSS, else false.
 * @context usermain (APP_PRI_TEST)
 */
static bool test_line_late_branch(void)
{
    line_track_jct_t    jct = {0u, 0u, false, false, 0u};
    line_track_jct_in_t in  = {0u, 0u, 0u, 0u, TEST_MIN_COUNTS,
                               TEST_CONF_COUNTS, false};
    uint32_t            cls = APP_JCT_NONE;

    line_track_jct_reset(&jct);
    in.norm_l      = 950u;
    in.norm_r      = 500u;
    in.path_counts = 300u;
    in.branch_seq  = 1u;
    (void) line_track_jct_step(&jct, &in);

    in.norm_l      = 500u;
    in.norm_r      = 500u;
    in.path_counts = 340u;
    (void) line_track_jct_step(&jct, &in);

    in.branch_seq  = 2u;
    in.path_counts = 430u;
    cls            = line_track_jct_step(&jct, &in);

    return (APP_JCT_CROSS == cls);
}
#endif /* APP_TEST */

/*** end of file ***/
