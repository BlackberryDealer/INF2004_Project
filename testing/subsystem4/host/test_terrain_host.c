/*
 *  test_terrain_host.c
 *  Laptop tests of the pure Terrain Monitor (terrain.c): synthetic hump
 *  replays (S4-U01, U02, U06, U07), integer trig against math.h
 *  (S4-U03), the forward-acceleration history (S4-U12) and the lift-stop
 *  check (S4-U16, Change Requirement 2). All HOST_CHECK
 *  calls live in main() because HOST_INIT declares its counters there.
 *
 *  Host-only, outside BARR-C scope (docs/BARR_C_DEVIATIONS.md, D7):
 *  math.h and printf are used here but never on the target.
 */

#include "host_test.h"
#include "terrain.h"
#include "cfg_imu.h"
#include "odometry.h"
#include "app_bits.h"
#include "app_layout.h"
#include <math.h>

#define HOST_PI (3.14159265358979323846)

/* Synthetic-hump replay parameters */
#define HOST_FLAT_CYCLES (20u)
#define HOST_PHASE_COUNT (5u) /* flat, climb, top, descend, flat */
#define HOST_AZ_FLAT_MG  (1000)

typedef struct
{
    uint32_t hump_done_mm; /* height of the last reported hump */
    uint32_t hump_events;  /* number of humps reported */
    uint32_t max_hump_now_mm;
    int32_t  max_pitch_ddeg;
} host_replay_result_t;

static host_replay_result_t host_replay_hump(uint32_t height_mm,
                                             uint32_t half_len_mm,
                                             uint32_t ds_counts,
                                             uint32_t slip_mul_x2)
{
    terrain_t            ter;
    host_replay_result_t res          = {0u, 0u, 0u, 0};
    uint32_t             ds_um        = ds_counts * APP_UM_PER_COUNT;
    uint32_t             climb_cycles = (half_len_mm * 1000u) / ds_um;
    int32_t  slope_mg = (int32_t)((height_mm * 1000u) / half_len_mm);
    uint32_t phase_len[HOST_PHASE_COUNT];
    int32_t  phase_ax[HOST_PHASE_COUNT];
    uint32_t phase = 0u;
    uint32_t cycle = 0u;

    phase_len[0] = HOST_FLAT_CYCLES;
    phase_len[1] = climb_cycles;
    phase_len[2] = HOST_FLAT_CYCLES;
    phase_len[3] = climb_cycles;
    phase_len[4] = HOST_FLAT_CYCLES;

    phase_ax[0] = 0;
    phase_ax[1] = slope_mg;
    phase_ax[2] = 0;
    phase_ax[3] = 0 - slope_mg;
    phase_ax[4] = 0;

    terrain_reset(&ter);

    for (phase = 0u; phase < HOST_PHASE_COUNT; phase++)
    {
        for (cycle = 0u; cycle < phase_len[phase]; cycle++)
        {
            terrain_input_t  in;
            terrain_output_t out;

            in.ax_mg     = phase_ax[phase];
            in.az_mg     = HOST_AZ_FLAT_MG;
            in.a_fwd_mg  = 0;
            in.ds_counts = terrain_ds_counts((int32_t)(ds_counts * slip_mul_x2),
                                             (int32_t)ds_counts);

            terrain_step(&ter, &in, &out);

            if (out.hump_done_mm > 0u)
            {
                res.hump_done_mm = out.hump_done_mm;
                res.hump_events++;
            }
            if (out.hump_now_mm > res.max_hump_now_mm)
            {
                res.max_hump_now_mm = out.hump_now_mm;
            }
            if (out.pitch_ddeg > res.max_pitch_ddeg)
            {
                res.max_pitch_ddeg = out.pitch_ddeg;
            }
        }
    }

    return res;
}

static bool host_atan2_grid_ok(void)
{
    uint32_t ddeg = 0u;

    for (ddeg = 0u; ddeg < 3600u; ddeg += 10u)
    {
        double  rad  = ((double)ddeg / 1800.0) * HOST_PI;
        int32_t y    = (int32_t)(sin(rad) * 1000.0);
        int32_t x    = (int32_t)(cos(rad) * 1000.0);
        int32_t a    = terrain_atan2_ddeg(y, x);
        int32_t diff = a - (int32_t)ddeg;

        if (diff > 1800)
        {
            diff -= 3600;
        }
        if (diff < -1800)
        {
            diff += 3600;
        }
        if (diff < 0)
        {
            diff = 0 - diff;
        }
        if (diff > 2)
        {
            return false;
        }
    }
    return true;
}

static bool host_atan2_points_ok(void)
{
    return ((900 == terrain_atan2_ddeg(1000, 0)) &&
            (-900 == terrain_atan2_ddeg(-1000, 0)) &&
            (1800 == terrain_atan2_ddeg(0, -1000)) &&
            (0 == terrain_atan2_ddeg(0, 1000)) &&
            (0 == terrain_atan2_ddeg(0, 0)) &&
            (450 == terrain_atan2_ddeg(1000, 1000)));
}

static bool host_sin_cos_ok(void)
{
    return ((0 == terrain_sin_q14(0)) && (16384 == terrain_sin_q14(900)) &&
            (8192 == terrain_sin_q14(300)) &&
            (-8192 == terrain_sin_q14(-300)) && (16384 == terrain_cos_q14(0)) &&
            (0 == terrain_cos_q14(900)) && (8192 == terrain_cos_q14(600)));
}

static bool host_heading_math_ok(void)
{
    return ((3599 == terrain_heading_wrap_ddeg(3599)) &&
            (0 == terrain_heading_wrap_ddeg(3600)) &&
            (3599 == terrain_heading_wrap_ddeg(-1)) &&
            (5 == terrain_heading_wrap_ddeg(7205)) &&
            (200 == terrain_heading_delta_ddeg(3500u, 100u)) &&
            (-200 == terrain_heading_delta_ddeg(100u, 3500u)) &&
            (1800 == terrain_heading_delta_ddeg(0u, 1800u)));
}

static bool host_ds_guard_ok(void)
{
    return ((0u == terrain_ds_counts(-100, -100)) &&
            (5u == terrain_ds_counts(5, 10)) &&
            (2u == terrain_ds_counts(-4, 2)) &&
            (0u == terrain_ds_counts(0, 0)));
}

/* S4-U16: feeds n cycles of one (ax, ay, az) to terrain_lift_step and
   returns the count; *pb_lifted_any is set if any cycle reported lifted. */
static uint32_t host_lift_feed(uint32_t cnt, int32_t ax_mg, int32_t az_mg,
                               uint32_t n, bool * pb_lifted_any)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < n; idx++)
    {
        bool b_lifted = false;

        cnt = terrain_lift_step(cnt, ax_mg, 0, az_mg, &b_lifted);
        if (b_lifted)
        {
            *pb_lifted_any = true;
        }
    }

    return cnt;
}

int main(void)
{
    HOST_INIT();

    {
        bool b_grid_ok = host_atan2_grid_ok();

        HOST_CHECK("S4-U03", b_grid_ok, "atan2 1-deg grid error <= 0.2 deg");
    }
    {
        bool b_points_ok = host_atan2_points_ok();

        HOST_CHECK("S4-U03", b_points_ok, "atan2 cardinal and diagonal points");
    }
    {
        bool b_ok = host_sin_cos_ok();

        HOST_CHECK("S4-U03", b_ok, "sin/cos table points at 0, 30, 60, 90 deg");
    }
    {
        bool b_ok = host_heading_math_ok();

        HOST_CHECK("S4-U03", b_ok, "heading wrap and shortest-delta maths");
    }
    {
        bool b_ok = host_ds_guard_ok();

        HOST_CHECK("S4-U13", b_ok, "ds from the slower wheel; rebase gives 0");
    }
    {
        /* S4-U10: SB_IMU0 pitch pack/unpack and clamp (Phase 4). */
        bool     b_clamped = false;
        bool     b_ok      = true;
        uint32_t raw =
            app_bits_encode(-450, SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
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
        if ((!b_clamped) ||
            (app_bits_decode(raw, SB_IMU0_PITCH_OFFSET) != -450))
        {
            b_ok = false;
        }
        HOST_CHECK("S4-U10", b_ok,
                   "pitch pack/unpack at -45/0/+45 deg and clamp beyond");
    }
    {
        terrain_speed_hist_t hist;
        int32_t              a0 = 0;
        int32_t              a1 = 0;
        int32_t              a2 = 0;
        int32_t              a3 = 0;
        int32_t              a4 = 0;
        int32_t              a5 = 0;
        int32_t              a6 = 0;
        int32_t              a7 = 0;
        int32_t              a8 = 0;

        terrain_speed_hist_reset(&hist);
        a0 = terrain_speed_hist_push(&hist, 0);
        a1 = terrain_speed_hist_push(&hist, 10);
        a2 = terrain_speed_hist_push(&hist, 20);
        a3 = terrain_speed_hist_push(&hist, 30);
        a4 = terrain_speed_hist_push(&hist, 40);
        a5 = terrain_speed_hist_push(&hist, 40);
        a6 = terrain_speed_hist_push(&hist, 40);
        a7 = terrain_speed_hist_push(&hist, 0);
        a8 = terrain_speed_hist_push(&hist, 0);

        /* Gain from the measured 324 um per count (A4, 2026-10-29):
           324 x 1024000 / 24517 = 13532 (Q10), so 1 cpw = 13.2 mg.     */
        HOST_CHECK("S4-U12",
                   (0 == a0) && (132 == a1) && (264 == a2) && (396 == a3) &&
                       (528 == a4),
                   "fresh (zero-filled) history: a_fwd = speed x gain");
        HOST_CHECK("S4-U12", 528 == a5,
                   "40 - 0 over 50 ms = 528 mg (hand calculation)");
        HOST_CHECK("S4-U12", 396 == a6, "40 - 10 over 50 ms = 396 mg");
        HOST_CHECK("S4-U12", -264 == a7, "0 - 20 over 50 ms = -264 mg");
        HOST_CHECK("S4-U12", -396 == a8, "0 - 30 over 50 ms = -396 mg");
    }
    {
        host_replay_result_t u01 = host_replay_hump(20u, 200u, 4u, 1u);
        host_replay_result_t u02 = host_replay_hump(40u, 500u, 4u, 1u);
        host_replay_result_t u06 = host_replay_hump(20u, 300u, 4u, 1u);
        host_replay_result_t u07 = host_replay_hump(20u, 200u, 4u, 2u);

        HOST_CHECK("S4-U01", 1u == u01.hump_events,
                   "one hump reported for the recorded profile");
        HOST_CHECK("S4-U01",
                   (u01.hump_done_mm >= 15u) && (u01.hump_done_mm <= 25u),
                   "20 mm ruler hump within +-20 % / +-5 mm");
        HOST_CHECK("S4-U01", (u01.max_pitch_ddeg >= 45),
                   "climb pitch was seen (>= 4.5 deg)");

        HOST_CHECK("S4-U02", 0u == u02.hump_events,
                   "ramp longer than TERRAIN_MAX_CLIMB_MM: no hump");

        HOST_CHECK("S4-U06", 1u == u06.hump_events,
                   "long hump at ~150 mm/s is not rejected");
        HOST_CHECK("S4-U06",
                   (u06.hump_done_mm >= 15u) && (u06.hump_done_mm <= 25u),
                   "long-hump peak within tolerance");

        HOST_CHECK("S4-U07", 1u == u07.hump_events,
                   "slip replay still reports the hump");
        HOST_CHECK("S4-U07",
                   (u07.max_hump_now_mm * 10u) >= (u01.max_hump_now_mm * 9u),
                   "slip changes the height by <= 10 %");
    }

    {
        /* S4-U16: lift-stop thresholds (cfg_imu.h: az < 500 mg, or
           | |a| - 1 g | > 300 mg, 5 cycles in a row). */
        bool     b_any  = false;
        bool     b_last = false;
        uint32_t cnt    = 0u;

        cnt = host_lift_feed(0u, 0, 1000, 200u, &b_any);
        HOST_CHECK("S4-U16", (0u == cnt) && !b_any,
                   "flat driving (az +1000 mg, 2 s): never lifted");
        b_any = false;
        cnt   = host_lift_feed(0u, 707, 707, 200u, &b_any);
        HOST_CHECK("S4-U16", (0u == cnt) && !b_any,
                   "45 deg slope held 2 s (A24 limit): |a| = 1 g, not lifted");
        b_any = false;
        cnt   = host_lift_feed(0u, 200, 1000, 200u, &b_any);
        HOST_CHECK("S4-U16", (0u == cnt) && !b_any,
                   "braking / accelerating at 0.2 g on the flat: not lifted");
        b_any = false;
        cnt   = host_lift_feed(0u, 0, 1400, 4u, &b_any);
        cnt   = host_lift_feed(cnt, 0, 1000, 1u, &b_any);
        cnt   = host_lift_feed(cnt, 0, 600, 4u, &b_any);
        HOST_CHECK("S4-U16", !b_any && (4u == cnt),
                   "40 ms bumps (+400 / -400 mg) with a level cycle between:"
                   " no stop, the count restarted");
        b_any = false;
        cnt   = host_lift_feed(0u, 0, 1350, 4u, &b_any);
        HOST_CHECK("S4-U16", !b_any && (4u == cnt),
                   "hand lift (+350 mg): not yet after 4 cycles (40 ms)");
        cnt = terrain_lift_step(cnt, 0, 0, 1350, &b_last);
        HOST_CHECK("S4-U16", b_last && (IMU_LIFT_CONFIRM_CYCLES == cnt),
                   "hand lift: stop on the 5th cycle (50 ms)");
        cnt = host_lift_feed(cnt, 0, 1350, 20u, &b_any);
        HOST_CHECK("S4-U16", (IMU_LIFT_CONFIRM_CYCLES == cnt)
                   && (cnt <= (uint32_t)PRV_IMU_A_LIFT_CNT_MAX),
                   "count saturates at 5: fits the 3-bit LIFT_CNT field");
        b_any = false;
        (void)host_lift_feed(0u, 0, -1000, 5u, &b_any);
        HOST_CHECK("S4-U16", b_any, "upside down (az -1000 mg): stop");
        b_any = false;
        (void)host_lift_feed(0u, 940, 342, 5u, &b_any); /* 70 deg */
        HOST_CHECK("S4-U16", b_any, "tipped over 70 deg (az 342 mg): stop");
        b_any = false;
        (void)host_lift_feed(0u, 0, 650, 5u, &b_any);
        HOST_CHECK("S4-U16", b_any,
                   "lowered fast / dropping (|a| 650 mg): stop");
        b_any = false;
        (void)host_lift_feed(0u, 30000, 30000, 5u, &b_any);
        HOST_CHECK("S4-U16", b_any,
                   "out-of-range input is clamped, no overflow: stop");
        HOST_CHECK("S4-U16",
                   (APP_MCLASS_LIFTED == terrain_classify(TERRAIN_CLIMBING,
                                                          true, 90, 200, 10))
                   && (APP_MCLASS_CLIMBING
                       == terrain_classify(TERRAIN_CLIMBING, false, 0, 0, 10)),
                   "class LIFTED wins while SAFETY_LIFT is set, else as before");
    }

    /* S4-U17 (car run 2026-11-18 13:19): a smooth level lift kept az at
       ~1000 mg, so only the IR pair saw it (both 3890..4067 raw); the
       darkest tape that day read 3570. */
    {
        uint32_t cnt    = 0u;
        bool     b_lift = false;
        bool     b_any  = false;
        uint32_t idx    = 0u;

        for (idx = 0u; idx < 200u; idx++)   /* 2 s on a cross: 3570/3570 */
        {
            cnt = terrain_ir_lift_step(cnt, 3570u, 3570u, &b_lift);
            b_any = b_any || b_lift;
        }
        HOST_CHECK("S4-U17", !b_any && (0u == cnt),
                   "both on the darkest tape (3570): never lifted");
        b_any = false;
        for (idx = 0u; idx < 200u; idx++)   /* one sensor off the floor */
        {
            cnt = terrain_ir_lift_step(cnt, 4050u, 3200u, &b_lift);
            b_any = b_any || b_lift;
        }
        HOST_CHECK("S4-U17", !b_any, "only one sensor high: not lifted");
        b_any = false;
        cnt   = 0u;
        for (idx = 0u; idx < 100u; idx++)   /* dark ramp 13:28: 3708/3905 */
        {
            cnt = terrain_ir_lift_step(cnt, 3708u, 3905u, &b_lift);
            b_any = b_any || b_lift;
        }
        HOST_CHECK("S4-U17", !b_any && (0u == cnt),
                   "dark ramp (L 3708, R 3905) for 1 s: never lifted");
        for (idx = 0u; idx < 29u; idx++)    /* a 290 ms crest, then floor */
        {
            cnt = terrain_ir_lift_step(cnt, 3996u, 3999u, &b_lift);
            b_any = b_any || b_lift;
        }
        cnt = terrain_ir_lift_step(cnt, 3200u, 3999u, &b_lift);
        HOST_CHECK("S4-U17", !b_any && !b_lift && (0u == cnt),
                   "290 ms with no floor, then floor: no stop, count reset");
        for (idx = 0u; idx < 29u; idx++)
        {
            cnt = terrain_ir_lift_step(cnt, 4021u, 3980u, &b_lift);
        }
        HOST_CHECK("S4-U17", !b_lift && (29u == cnt),
                   "logged level lift: not yet after 29 cycles");
        cnt = terrain_ir_lift_step(cnt, 4023u, 4028u, &b_lift);
        HOST_CHECK("S4-U17", b_lift && (IMU_LIFT_IR_CYCLES == cnt),
                   "logged level lift: stop on the 30th cycle (300 ms)");
        for (idx = 0u; idx < 50u; idx++)
        {
            cnt = terrain_ir_lift_step(cnt, 4050u, 4050u, &b_lift);
        }
        HOST_CHECK("S4-U17", (IMU_LIFT_IR_CYCLES == cnt)
                   && (cnt <= (uint32_t)PRV_IMU_A_IR_LIFT_CNT_MAX),
                   "count saturates at 30: fits the 5-bit IR_LIFT_CNT field");
    }

    /* S4-U18 (track 2026-11-18): a thin strip only the right wheel
       climbs tilts the car sideways; h = track x |ay| / 1000 mg. */
    {
        uint32_t const zero = ((uint32_t)PRV_IMU_R_FILT_OFFSET
                               << PRV_IMU_R_FILT_POS);
        uint32_t word  = zero;
        uint32_t done  = 0u;
        uint32_t total = 0u;
        uint32_t n_rep = 0u;
        int32_t  ay    = 0;
        uint32_t idx   = 0u;

        for (idx = 0u; idx < 300u; idx++)   /* straight, +-30 mg noise */
        {
            ay   = ((idx % 2u) == 0u) ? 30 : -30;
            word = terrain_roll_hump_step(word, ay, 6, 6, &done);
            n_rep += (done > 0u) ? 1u : 0u;
        }
        HOST_CHECK("S4-U18", 0u == n_rep, "flat, +-30 mg vibration: no hump");

        /* 8 mm hump: ay up to -64 mg (8 / 125 x 1000), ~250 ms on it */
        word = zero;
        for (idx = 0u; idx < 10u; idx++)
        {
            word = terrain_roll_hump_step(word, -(int32_t)(idx * 64u / 10u),
                                          6, 7, &done);
            total += done;
        }
        for (idx = 0u; idx < 10u; idx++)
        {
            word = terrain_roll_hump_step(word, -64, 6, 7, &done);
            total += done;
        }
        for (idx = 0u; idx < 30u; idx++)
        {
            ay   = (idx < 10u) ? -(int32_t)(64u - (idx * 64u / 10u)) : 0;
            word = terrain_roll_hump_step(word, ay, 6, 6, &done);
            total += done;
            n_rep += (done > 0u) ? 1u : 0u;
        }
        HOST_CHECK("S4-U18", (1u == n_rep) && (total >= 7u) && (total <= 9u),
                   "8 mm one-wheel hump (ay 64 mg): one report, 7..9 mm");

        /* same tilt either side: the sign does not matter */
        word  = zero;
        total = 0u;
        for (idx = 0u; idx < 60u; idx++)
        {
            ay   = (idx < 20u) ? 120 : 0;
            word = terrain_roll_hump_step(word, ay, 6, 6, &done);
            total += done;
        }
        HOST_CHECK("S4-U18", (total >= 14u) && (total <= 15u),
                   "+120 mg (other mounting sign): 15 mm");

        /* a curve: 80 mg sideways while turning never starts a hump */
        word  = zero;
        n_rep = 0u;
        for (idx = 0u; idx < 100u; idx++)
        {
            ay   = (idx < 60u) ? 80 : 0;
            word = terrain_roll_hump_step(word, ay, 4, 9, &done);
            n_rep += (done > 0u) ? 1u : 0u;
        }
        HOST_CHECK("S4-U18", 0u == n_rep, "turning (dl 4, dr 9): no hump");

        /* parked tilted / on a long side slope for 3 s: dropped */
        word  = zero;
        n_rep = 0u;
        for (idx = 0u; idx < 320u; idx++)
        {
            ay   = (idx < 300u) ? 90 : 0;
            word = terrain_roll_hump_step(word, ay, 6, 6, &done);
            n_rep += (done > 0u) ? 1u : 0u;
        }
        HOST_CHECK("S4-U18", 0u == n_rep, "3 s held tilt: no report (ramp rule)");

        /* not moving: no start */
        word  = zero;
        n_rep = 0u;
        for (idx = 0u; idx < 60u; idx++)
        {
            ay   = (idx < 20u) ? 120 : 0;
            word = terrain_roll_hump_step(word, ay, 0, 0, &done);
            n_rep += (done > 0u) ? 1u : 0u;
        }
        HOST_CHECK("S4-U18", 0u == n_rep, "stationary tilt (handling): no hump");
    }

    return (0 == HOST_SUMMARY("terrain")) ? 0 : 1;
}
