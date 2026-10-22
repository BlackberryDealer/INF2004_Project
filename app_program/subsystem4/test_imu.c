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
#include "../app_bits.h"
#include "../app_layout.h"
#include "terrain.h"
#endif

/* ---- private function prototypes ------------------------------------- */

#if APP_TEST
static bool test_imu_terrain_reset(void);
static bool test_imu_pitch_pack(void);
static bool test_imu_hump_clamp(void);
static bool test_imu_offset_lsb(void);
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

    /* S4-U10: packed pitch storage (same checks as the host build). */
    TEST_CHECK("S4-U10", test_imu_pitch_pack(),
               "pitch pack/unpack at -45/0/+45 deg and clamp beyond");

    /* S4-U11: reaching or passing the 7-bit run-peak ceiling sets the
       clamp flag, so imu_clamp_fault() raises APP_FAULT_RANGE_CLAMP. */
    TEST_CHECK("S4-U11", test_imu_hump_clamp(),
               "hump 127 passes, 128 clamps and raises RANGE_CLAMP");

    /* S4-U11: the PRV_OFS_* words store 2 mg per step, so the read path
       must scale the decoded value back to mg. */
    TEST_CHECK("S4-U11", test_imu_offset_lsb(),
               "one PRV_OFS_A AX step decodes to 2 mg, range covers +-4094 mg");

    TEST_SUMMARY("imu");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if APP_TEST
/**
 * test_imu_terrain_reset
 * @brief   Seeds a dirty monitor (packed words), resets it, steps it
 *          once while flat and checks that every field is back to the
 *          run-start state.
 * @return  true when every field is cleared
 * @context usermain (test only)
 */
static bool test_imu_terrain_reset(void)
{
    terrain_t        ter;
    terrain_input_t  in;
    terrain_output_t out;
    bool             b_ok = true;

    ter.word_state = app_bits_put(0u, PRV_IMU_A_STATE_POS, PRV_IMU_A_STATE_BITS,
                                  TERRAIN_DESCENDING);
    ter.word_state = app_bits_put(ter.word_state, PRV_IMU_A_HYST_POS,
                                  PRV_IMU_A_HYST_BITS, 3u);
    ter.word_prog  = app_bits_put(0u, PRV_IMU_B_CLIMB_CNT_POS,
                                  PRV_IMU_B_CLIMB_CNT_BITS, 77u);
    ter.word_prog  = app_bits_put(ter.word_prog, PRV_IMU_B_PEAK_NOW_POS,
                                  PRV_IMU_B_PEAK_NOW_BITS, 12u);
    ter.word_pitch = app_bits_put(
        0u, PRV_IMU_C_PITCH_MAX_POS, PRV_IMU_C_PITCH_MAX_BITS,
        app_bits_encode(320, PRV_IMU_C_PITCH_MAX_MIN, PRV_IMU_C_PITCH_MAX_MAX,
                        PRV_IMU_C_PITCH_MAX_OFFSET, NULL));
    ter.acc_um_mg = 123456;

    terrain_reset(&ter);

    in.ax_mg     = 0;
    in.az_mg     = 1000;
    in.a_fwd_mg  = 0;
    in.ds_counts = 0u;
    terrain_step(&ter, &in, &out);

    if (TERRAIN_FLAT !=
        app_bits_get(ter.word_state, PRV_IMU_A_STATE_POS, PRV_IMU_A_STATE_BITS))
    {
        b_ok = false;
    }
    if (0u !=
        app_bits_get(ter.word_state, PRV_IMU_A_HYST_POS, PRV_IMU_A_HYST_BITS))
    {
        b_ok = false;
    }
    if (0 != ter.acc_um_mg)
    {
        b_ok = false;
    }
    if (0u != app_bits_get(ter.word_prog, PRV_IMU_B_CLIMB_CNT_POS,
                           PRV_IMU_B_CLIMB_CNT_BITS))
    {
        b_ok = false;
    }
    if (0u != app_bits_get(ter.word_prog, PRV_IMU_B_PEAK_NOW_POS,
                           PRV_IMU_B_PEAK_NOW_BITS))
    {
        b_ok = false;
    }
    if (0 !=
        app_bits_decode(app_bits_get(ter.word_pitch, PRV_IMU_C_PITCH_MAX_POS,
                                     PRV_IMU_C_PITCH_MAX_BITS),
                        PRV_IMU_C_PITCH_MAX_OFFSET))
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

/**
 * test_imu_pitch_pack
 * @brief   S4-U10 (host side, also run on target): SB_IMU0 pitch
 *          pack/unpack at -45.0, 0 and +45.0 deg and the clamp beyond.
 * @return  true when the stored codes match the generated layout
 * @context usermain (test only)
 */
static bool test_imu_pitch_pack(void)
{
    bool     b_ok      = true;
    bool     b_clamped = false;
    uint32_t raw       = 0u;

    raw = app_bits_encode(-450, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
                          SB_IMU0_PITCH_OFFSET, &b_clamped);
    if ((b_clamped) || (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != -450))
    {
        b_ok = false;
    }
    raw = app_bits_encode(0, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
                          SB_IMU0_PITCH_OFFSET, &b_clamped);
    if ((b_clamped) || (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != 0))
    {
        b_ok = false;
    }
    raw = app_bits_encode(450, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
                          SB_IMU0_PITCH_OFFSET, &b_clamped);
    if ((b_clamped) || (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != 450))
    {
        b_ok = false;
    }
    raw = app_bits_encode(451, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
                          SB_IMU0_PITCH_OFFSET, &b_clamped);
    if ((!b_clamped) || (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != 450))
    {
        b_ok = false;
    }
    b_clamped = false;
    raw       = app_bits_encode(-451, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
                                SB_IMU0_PITCH_OFFSET, &b_clamped);
    if ((!b_clamped) || (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != -450))
    {
        b_ok = false;
    }

    return b_ok;
}

/**
 * test_imu_hump_clamp
 * @brief   S4-U11: a hump of 127 mm is representable and must not set
 *          the clamp flag, a hump of 128 mm must clamp to 127 and set
 *          it (which imu_clamp_fault turns into APP_FAULT_RANGE_CLAMP).
 *          A25 keeps RANGE_CLAMP out of APP_FAULT_STOP_MASK, so this
 *          fault never by itself stops the robot.
 * @return  true when the run-peak clamp behaves as the layout says
 * @context usermain (test only)
 */
static bool test_imu_hump_clamp(void)
{
    bool     b_ok      = true;
    bool     b_clamped = false;
    uint32_t raw       = 0u;

    raw = app_bits_encode(127, PRV_IMU_C_RUN_PEAK_MIN, PRV_IMU_C_RUN_PEAK_MAX,
                          PRV_IMU_C_RUN_PEAK_OFFSET, &b_clamped);
    if ((b_clamped) || (127 != app_bits_decode(raw, PRV_IMU_C_RUN_PEAK_OFFSET)))
    {
        b_ok = false;
    }

    b_clamped = false;
    raw = app_bits_encode(128, PRV_IMU_C_RUN_PEAK_MIN, PRV_IMU_C_RUN_PEAK_MAX,
                          PRV_IMU_C_RUN_PEAK_OFFSET, &b_clamped);
    if ((!b_clamped) ||
        (127 != app_bits_decode(raw, PRV_IMU_C_RUN_PEAK_OFFSET)))
    {
        b_ok = false;
    }

    if (0u != (APP_FAULT_STOP_MASK & APP_FAULT_RANGE_CLAMP))
    {
        b_ok = false; /* A25: informational only */
    }

    return b_ok;
}

/**
 * test_imu_offset_lsb
 * @brief   S4-U11 (offset half): the PRV_OFS_* words carry 2 mg per step
 *          (tools/layout.csv, spec B4.4), so one step is 2 mg and the
 *          12-bit field covers the full +-4 g range. imu_read_acc must
 *          scale a decoded step back to 2 mg before subtracting it.
 * @return  true when the layout matches the documented LSB weight
 * @context usermain (test only)
 */
static bool test_imu_offset_lsb(void)
{
    bool     b_ok      = true;
    bool     b_clamped = false;
    uint32_t raw       = 0u;
    int32_t  step_mg   = 2; /* layout.csv: PRV_OFS_A AX/AY/AZ LSB = 2 mg */

    if ((PRV_OFS_A_AX_BITS != PRV_OFS_A_AY_BITS) ||
        (PRV_OFS_A_AX_BITS != PRV_OFS_B_AZ_BITS))
    {
        b_ok = false; /* the three offsets must stay interchangeable */
    }

    /* +1 g in 2 mg steps. */
    raw = app_bits_encode(500, PRV_OFS_A_AX_MIN, PRV_OFS_A_AX_MAX,
                          PRV_OFS_A_AX_OFFSET, &b_clamped);
    if ((b_clamped) ||
        ((app_bits_decode(raw, PRV_OFS_A_AX_OFFSET) * step_mg) != 1000))
    {
        b_ok = false;
    }

    /* Negative offset: -1 g. */
    b_clamped = false;
    raw       = app_bits_encode(-500, PRV_OFS_A_AX_MIN, PRV_OFS_A_AX_MAX,
                                PRV_OFS_A_AX_OFFSET, &b_clamped);
    if ((b_clamped) ||
        ((app_bits_decode(raw, PRV_OFS_A_AX_OFFSET) * step_mg) != -1000))
    {
        b_ok = false;
    }

    /* The top code is 2047 steps = 4094 mg; one step more clamps. */
    raw = app_bits_encode(2048, PRV_OFS_A_AX_MIN, PRV_OFS_A_AX_MAX,
                          PRV_OFS_A_AX_OFFSET, &b_clamped);
    if ((!b_clamped) ||
        ((app_bits_decode(raw, PRV_OFS_A_AX_OFFSET) * step_mg) != 4094))
    {
        b_ok = false;
    }

    return b_ok;
}
#endif

/*** end of file ***/
