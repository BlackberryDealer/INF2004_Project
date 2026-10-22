/*
 *  test_terrain_host.c
 *  Laptop tests of the pure Terrain Monitor (terrain.c): synthetic hump
 *  replays (S4-U01, U02, U06, U07), integer trig against math.h
 *  (S4-U03) and the forward-acceleration history (S4-U12). All HOST_CHECK
 *  calls live in main() because HOST_INIT declares its counters there.
 *
 *  Host-only, outside BARR-C scope (docs/BARR_C_DEVIATIONS.md, D7):
 *  math.h and printf are used here but never on the target.
 */

#include "host_test.h"
#include "terrain.h"
#include "cfg_imu.h"
#include "odometry.h"
#include <math.h>

#define HOST_PI         (3.14159265358979323846)

/* Synthetic-hump replay parameters */
#define HOST_FLAT_CYCLES     (20u)
#define HOST_PHASE_COUNT     (5u)   /* flat, climb, top, descend, flat */
#define HOST_AZ_FLAT_MG      (1000)

typedef struct
{
    uint32_t hump_done_mm;    /* height of the last reported hump */
    uint32_t hump_events;     /* number of humps reported */
    uint32_t max_hump_now_mm;
    int32_t  max_pitch_ddeg;
} host_replay_result_t;

static host_replay_result_t host_replay_hump(uint32_t height_mm,
                                             uint32_t half_len_mm,
                                             uint32_t ds_counts,
                                             uint32_t slip_mul_x2)
{
    terrain_t ter;
    host_replay_result_t res = {0u, 0u, 0u, 0};
    uint32_t ds_um = ds_counts * APP_UM_PER_COUNT;
    uint32_t climb_cycles = (half_len_mm * 1000u) / ds_um;
    int32_t  slope_mg = (int32_t) ((height_mm * 1000u) / half_len_mm);
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
            terrain_input_t in;
            terrain_output_t out;

            in.ax_mg = phase_ax[phase];
            in.az_mg = HOST_AZ_FLAT_MG;
            in.a_fwd_mg = 0;
            in.ds_counts = terrain_ds_counts(
                (int32_t) (ds_counts * slip_mul_x2),
                (int32_t) ds_counts);

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
        double rad = ((double) ddeg / 1800.0) * HOST_PI;
        int32_t y = (int32_t) (sin(rad) * 1000.0);
        int32_t x = (int32_t) (cos(rad) * 1000.0);
        int32_t a = terrain_atan2_ddeg(y, x);
        int32_t diff = a - (int32_t) ddeg;

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
    return ((900 == terrain_atan2_ddeg(1000, 0))
            && (-900 == terrain_atan2_ddeg(-1000, 0))
            && (1800 == terrain_atan2_ddeg(0, -1000))
            && (0 == terrain_atan2_ddeg(0, 1000))
            && (0 == terrain_atan2_ddeg(0, 0))
            && (450 == terrain_atan2_ddeg(1000, 1000)));
}

static bool host_sin_cos_ok(void)
{
    return ((0 == terrain_sin_q14(0))
            && (16384 == terrain_sin_q14(900))
            && (8192 == terrain_sin_q14(300))
            && (-8192 == terrain_sin_q14(-300))
            && (16384 == terrain_cos_q14(0))
            && (0 == terrain_cos_q14(900))
            && (8192 == terrain_cos_q14(600)));
}

static bool host_heading_math_ok(void)
{
    return ((3599 == terrain_heading_wrap_ddeg(3599))
            && (0 == terrain_heading_wrap_ddeg(3600))
            && (3599 == terrain_heading_wrap_ddeg(-1))
            && (5 == terrain_heading_wrap_ddeg(7205))
            && (200 == terrain_heading_delta_ddeg(3500u, 100u))
            && (-200 == terrain_heading_delta_ddeg(100u, 3500u))
            && (1800 == terrain_heading_delta_ddeg(0u, 1800u)));
}

static bool host_ds_guard_ok(void)
{
    return ((0u == terrain_ds_counts(-100, -100))
            && (5u == terrain_ds_counts(5, 10))
            && (2u == terrain_ds_counts(-4, 2))
            && (0u == terrain_ds_counts(0, 0)));
}

int main(void)
{
    HOST_INIT();

    {
        bool b_grid_ok = host_atan2_grid_ok();

        HOST_CHECK("S4-U03", b_grid_ok,
                   "atan2 1-deg grid error <= 0.2 deg");
    }
    {
        bool b_points_ok = host_atan2_points_ok();

        HOST_CHECK("S4-U03", b_points_ok,
                   "atan2 cardinal and diagonal points");
    }
    {
        bool b_ok = host_sin_cos_ok();

        HOST_CHECK("S4-U03", b_ok,
                   "sin/cos table points at 0, 30, 60, 90 deg");
    }
    {
        bool b_ok = host_heading_math_ok();

        HOST_CHECK("S4-U03", b_ok,
                   "heading wrap and shortest-delta maths");
    }
    {
        bool b_ok = host_ds_guard_ok();

        HOST_CHECK("S4-U13", b_ok,
                   "ds from the slower wheel; rebase gives 0");
    }
    {
        terrain_speed_hist_t hist;
        int32_t a0 = 0;
        int32_t a1 = 0;
        int32_t a2 = 0;
        int32_t a3 = 0;
        int32_t a4 = 0;
        int32_t a5 = 0;
        int32_t a6 = 0;
        int32_t a7 = 0;
        int32_t a8 = 0;

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

        HOST_CHECK("S4-U12", (0 == a0) && (0 == a1) && (0 == a2)
                   && (0 == a3) && (0 == a4),
                   "window not full yet: a_fwd = 0");
        HOST_CHECK("S4-U12", 569 == a5,
                   "40 - 0 over 50 ms = 569 mg (hand calculation)");
        HOST_CHECK("S4-U12", 427 == a6,
                   "40 - 10 over 50 ms = 427 mg");
        HOST_CHECK("S4-U12", -284 == a7, "0 - 20 over 50 ms = -284 mg");
        HOST_CHECK("S4-U12", -427 == a8, "0 - 30 over 50 ms = -427 mg");
    }
    {
        host_replay_result_t u01 = host_replay_hump(20u, 200u, 4u, 1u);
        host_replay_result_t u02 = host_replay_hump(40u, 500u, 4u, 1u);
        host_replay_result_t u06 = host_replay_hump(20u, 300u, 4u, 1u);
        host_replay_result_t u07 = host_replay_hump(20u, 200u, 4u, 2u);

        HOST_CHECK("S4-U01", 1u == u01.hump_events,
                   "one hump reported for the recorded profile");
        HOST_CHECK("S4-U01", (u01.hump_done_mm >= 15u)
                   && (u01.hump_done_mm <= 25u),
                   "20 mm ruler hump within +-20 % / +-5 mm");
        HOST_CHECK("S4-U01", (u01.max_pitch_ddeg >= 45),
                   "climb pitch was seen (>= 4.5 deg)");

        HOST_CHECK("S4-U02", 0u == u02.hump_events,
                   "ramp longer than TERRAIN_MAX_CLIMB_MM: no hump");

        HOST_CHECK("S4-U06", 1u == u06.hump_events,
                   "long hump at ~150 mm/s is not rejected");
        HOST_CHECK("S4-U06", (u06.hump_done_mm >= 15u)
                   && (u06.hump_done_mm <= 25u),
                   "long-hump peak within tolerance");

        HOST_CHECK("S4-U07", 1u == u07.hump_events,
                   "slip replay still reports the hump");
        HOST_CHECK("S4-U07", (u07.max_hump_now_mm * 10u)
                   >= (u01.max_hump_now_mm * 9u),
                   "slip changes the height by <= 10 %");
    }

    return (0 == HOST_SUMMARY("terrain")) ? 0 : 1;
}
