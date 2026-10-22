/**
 * @file    test_imu.c
 * @brief   On-target tests of imu (TEST=1 builds only).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    test
 * @reqs    S4-U13 (run-start resets), S4-U11 (Phase 4, packed board)
 *
 * Runs in usermain at APP_PRI_TEST (21) while the whole system runs, so
 * a test here must never change live system state. Setting the live
 * mission mode bits would violate the owner rule (mission_task only),
 * so S4-U13 checks the pure reset/ds behaviour the imu_task code path
 * uses; the mode-driven half (APP_MODE_RUN_START for 30 ms) is verified
 * in Phase 3 once mission drives the bit for real. The host test in
 * testing/subsystem4/host covers the same functions.
 */

#include "test_imu.h" /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#include "../app_types.h"
#include "terrain.h"
#endif

/* ---- private function prototypes ------------------------------------- */

#if APP_TEST
static bool test_imu_terrain_reset(void);
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_imu_run
 * @brief   Runs every on-target test of imu and prints the
 *          [TEST] summary line. Empty unless built with TEST=1.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_imu_run(void)
{
#if APP_TEST
    TEST_INIT();

    /* S4-U13: run start must reset the run peak and terrain state, and
       a position rebase must give ds = 0 (never a false climb). */
    TEST_CHECK("S4-U13", test_imu_terrain_reset(),
               "terrain_reset clears state and accumulators");
    TEST_CHECK("S4-U13", 0u == terrain_ds_counts(-100, -100),
               "position rebase gives ds = 0 (no false climb)");
    TEST_CHECK("S4-U13", 4u == terrain_ds_counts(4, 20),
               "ds is the slower wheel's delta");

    TEST_SUMMARY("imu");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if APP_TEST
/**
 * test_imu_terrain_reset
 * @brief   Seeds a dirty monitor, resets it, steps it once while flat
 *          and checks that every field is back to the run-start state.
 * @return  true when every field is cleared
 * @context usermain (test only)
 */
static bool test_imu_terrain_reset(void)
{
    terrain_t        ter;
    terrain_input_t  in;
    terrain_output_t out;
    bool             b_ok = true;

    ter.state          = TERRAIN_DESCENDING;
    ter.hyst           = 3u;
    ter.acc_um_mg      = 123456;
    ter.climb_counts   = 77u;
    ter.peak_now_mm    = 12u;
    ter.pitch_max_ddeg = 320;

    terrain_reset(&ter);

    in.ax_mg     = 0;
    in.az_mg     = 1000;
    in.a_fwd_mg  = 0;
    in.ds_counts = 0u;
    terrain_step(&ter, &in, &out);

    if (TERRAIN_FLAT != ter.state)
    {
        b_ok = false;
    }
    if (0u != ter.hyst)
    {
        b_ok = false;
    }
    if (0 != ter.acc_um_mg)
    {
        b_ok = false;
    }
    if (0u != ter.climb_counts)
    {
        b_ok = false;
    }
    if (0u != ter.peak_now_mm)
    {
        b_ok = false;
    }
    if (0 != ter.pitch_max_ddeg)
    {
        b_ok = false;
    }
    if (0 != out.pitch_ddeg)
    {
        b_ok = false;
    }
    if (0u != out.hump_now_mm)
    {
        b_ok = false;
    }
    if (0u != out.hump_done_mm)
    {
        b_ok = false;
    }

    return b_ok;
}
#endif

/*** end of file ***/
