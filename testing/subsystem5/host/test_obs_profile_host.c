/*
 * test_obs_profile_host.c
 * Laptop tests for obs_profile.c (S5-U01..U03, U02b, U23). Host-only test code,
 * outside BARR-C scope (like host_test.h).
 * Build and run: make -C testing/subsystem5/host
 */

#include <stdint.h>
#include <stdbool.h>
#include "../../host/host_test.h"
#include "subsystem5/obs_profile.h"
#include "subsystem5/cfg_obstacle.h"

/* sin(5k deg), Q14, k = 0..36 (same values as obs_profile.c) */
static int32_t const sin_q14[37] =
{
    0, 1428, 2845, 4240, 5604, 6924, 8192, 9397, 10531, 11585,
    12551, 13421, 14189, 14849, 15396, 15826, 16135, 16322, 16384,
    16322, 16135, 15826, 15396, 14849, 14189, 13421, 12551, 11585,
    10531, 9397, 8192, 6924, 5604, 4240, 2845, 1428, 0
};

static void scan_clear(uint16_t * p_scan)
{
    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        p_scan[idx] = (uint16_t) APP_DIST_NONE_MM;
    }
}

/* A flat face perpendicular to the robot at forward distance depth_mm,
   seen from step first to step last: d = depth / sin(theta). */
static void scan_face(uint16_t * p_scan, uint32_t first, uint32_t last,
                      int32_t depth_mm)
{
    for (uint32_t step = first; step <= last; step++)
    {
        p_scan[step] = (uint16_t) ((depth_mm * 16384) / sin_q14[step + 6u]);
    }
}

static bool in_range(int32_t value, int32_t lo, int32_t hi)
{
    return (value >= lo) && (value <= hi);
}

int main(void)
{
    uint16_t      scan[APP_SCAN_STEPS];
    obs_profile_t prof;
    obs_plan_t    plan;

    HOST_INIT();

    /* S5-U01: face 300 mm ahead from 75 to 100 deg (slightly right of
       centre): x_right ~ +80, x_left ~ -52, width ~ 132. */
    scan_clear(scan);
    scan_face(scan, 9u, 14u, 300);
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U01", prof.b_is_valid, "profile valid");
    HOST_CHECK("S5-U01", in_range((int32_t) prof.width_mm, 126, 138),
               "width ~ 132 mm");
    HOST_CHECK("S5-U01", in_range((int32_t) prof.nearest_mm, 296, 301),
               "nearest ~ 300 mm");
    HOST_CHECK("S5-U01", in_range(prof.x_right_mm, 76, 84)
               && in_range(prof.x_left_mm, -56, -48), "lateral extent");
    HOST_CHECK("S5-U01", (APP_PROFILE_CLEAR_MM == prof.clear_left_mm)
               && (APP_PROFILE_CLEAR_MM == prof.clear_right_mm),
               "no other objects: both sides clear");
    HOST_CHECK("S5-U01", OBS_OUT_PLAN == plan.outcome, "outcome PLAN");
    HOST_CHECK("S5-U01", APP_SIDE_LEFT == plan.side,
               "picks the smaller shift (left)");
    HOST_CHECK("S5-U01", in_range((int32_t) plan.leg_lateral_mm, 173, 181),
               "lateral leg = corridor + 52");
    HOST_CHECK("S5-U01", plan.leg_forward_mm
               == (prof.nearest_mm + OBS_DEPTH_ASSUMED_MM + OBS_ROBOT_LEN_MM),
               "forward leg = nearest + depth + length");
    HOST_CHECK("S5-U01", !plan.b_was_clamped, "legs within NFR16");

    /* S5-U01: same face, an object 190 mm to the left blocks that side,
       so the plan goes right even though the shift is larger. */
    scan_face(scan, 18u, 18u, 329);   /* 120 deg, d ~ 380 */
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U01", prof.clear_left_mm < 250u, "left gap too small");
    HOST_CHECK("S5-U01", (OBS_OUT_PLAN == plan.outcome)
               && (APP_SIDE_RIGHT == plan.side), "falls back to the right");

    /* S5-U02: nothing near -> CONTINUE, profile not valid */
    scan_clear(scan);
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U02", !prof.b_is_valid, "empty scan: no profile");
    HOST_CHECK("S5-U02", OBS_OUT_CONTINUE == plan.outcome,
               "empty scan: CONTINUE");

    /* S5-U02: object far to the right (30-40 deg, x ~ +230..260) is
       outside the 125 mm corridor -> CONTINUE */
    scan_clear(scan);
    scan[0] = 300u;
    scan[1] = 300u;
    scan[2] = 300u;
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U02", !prof.b_is_valid,
               "object right of the corridor does not block");
    HOST_CHECK("S5-U02", OBS_OUT_CONTINUE == plan.outcome,
               "clear of the path: CONTINUE");

    /* S5-U02b (review R02): side object at 30 deg / 300 mm (x ~ 260,
       nearer forward y ~ 150) and a central object at 90 deg / 200 mm.
       The central one blocks, so it is the obstacle; not CONTINUE.     */
    scan_clear(scan);
    scan[0] = 300u;
    scan_face(scan, 11u, 13u, 200);
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U02b", prof.b_is_valid && (prof.angle_step >= 11u)
               && (prof.angle_step <= 13u)
               && in_range((int32_t) prof.nearest_mm, 197, 201),
               "the blocking central object is chosen");
    HOST_CHECK("S5-U02b", OBS_OUT_CONTINUE != plan.outcome,
               "plan is not CONTINUE");

    /* S5-U03: centred obstacle with objects 190 mm out on both sides */
    scan_clear(scan);
    scan_face(scan, 10u, 14u, 300);
    scan_face(scan, 6u, 6u, 329);     /* 60 deg, x ~ +190 */
    scan_face(scan, 18u, 18u, 329);   /* 120 deg, x ~ -190 */
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U03", (prof.clear_left_mm < 250u)
               && (prof.clear_right_mm < 250u), "both gaps too small");
    HOST_CHECK("S5-U03", OBS_OUT_NO_PATH == plan.outcome,
               "both sides blocked: NO_PATH");
    HOST_CHECK("S5-U03", !plan.b_was_clamped, "not a clamp case");

    /* S5-U03: wall from 50 to 130 deg at 250 mm needs a 334 mm shift,
       over the 300 mm NFR16 limit -> clamped, NO_PATH */
    scan_clear(scan);
    scan_face(scan, 4u, 20u, 250);
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U03", plan.b_was_clamped, "lateral leg over its limit");
    HOST_CHECK("S5-U03", OBS_OUT_NO_PATH == plan.outcome,
               "clipped plan is never driven: NO_PATH");
    HOST_CHECK("S5-U03", OBS_LEG_LAT_MAX_MM == plan.leg_lateral_mm,
               "reported leg clipped to 300 mm");

    /* S5-U23 (review V13-04): a run whose nearest point sits exactly on
       the corridor edge (x_left = +125) needs shift 0; the leg must be
       OBS_LEG_MIN_MM, never 0. Face at 250 mm from 30 to 60 deg puts
       x_left at 250 x cot(60) = 144; use a single point at 63.4 deg
       instead: build a scan by hand at step 6 (60 deg) with d chosen so
       x = d x cos(60) = 125 -> d = 250.                                */
    scan_clear(scan);
    scan[6] = 250u;
    scan[5] = 270u;
    obs_profile_build(scan, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U23", prof.b_is_valid && (125 == prof.x_left_mm),
               "run touches the corridor edge (x_left = 125)");
    HOST_CHECK("S5-U23", (OBS_OUT_PLAN == plan.outcome)
               && (APP_SIDE_LEFT == plan.side)
               && (OBS_LEG_MIN_MM == plan.leg_lateral_mm),
               "lateral leg = OBS_LEG_MIN_MM, never 0");

    return (0 == HOST_SUMMARY("subsystem5 host")) ? 0 : 1;
}
