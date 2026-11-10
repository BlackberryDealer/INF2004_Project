/*
 * test_obs_profile_host.c
 * Laptop tests for obs_profile.c (S5-U01..U03, U02b, U23, U44..U53). Host-only test code,
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

/* S5-U53 (review 2026-11-09): after a PLAN, the car's body at the end of
   the lateral leg plus OBS_SIDE_MARGIN_MM must clear every near return
   the pass drives by (forward <= nearest + depth + robot length): the
   "full path plus margin" check, independent of obs_profile.c.       */
static bool path_clear(uint16_t const * p_scan, obs_profile_t const * p_prof,
                       obs_plan_t const * p_plan)
{
    int32_t const xc    = (APP_SIDE_LEFT == p_plan->side)
                          ? -(int32_t) p_plan->leg_lateral_mm
                          : (int32_t) p_plan->leg_lateral_mm;
    int32_t const reach = (int32_t) (p_prof->nearest_mm
                                     + OBS_DEPTH_ASSUMED_MM
                                     + OBS_ROBOT_LEN_MM);
    bool          b_ok  = true;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        if (p_scan[idx] < OBS_NEAR_MM)
        {
            uint32_t const k   = idx + 6u;
            int32_t const  c   = (k <= 18u) ? sin_q14[18u - k]
                                            : -sin_q14[k - 18u];
            int32_t const  x   = ((int32_t) p_scan[idx] * c) / 16384;
            int32_t const  y   = ((int32_t) p_scan[idx] * sin_q14[k])
                                 / 16384;
            int32_t const  off = (x > xc) ? (x - xc) : (xc - x);

            if ((y <= reach) && (off < (int32_t) (OBS_ROBOT_HALF_W_MM
                                                  + OBS_SIDE_MARGIN_MM)))
            {
                b_ok = false;
            }
        }
    }

    return b_ok;
}

int main(void)
{
    uint16_t      scan[APP_SCAN_STEPS];
    obs_profile_t prof;
    obs_plan_t    plan;

    HOST_INIT();

    /* S5-U01: face 300 mm ahead from 75 to 100 deg (slightly right of
       centre). 2026-11-09 (S5-U45): a 25 deg span is narrower than the
       ~30 deg beam, so after beam correction only the shortest echo
       (90 deg) is left and the estimate is the 100 mm minimum centred on
       it: x -50..+50 (was x -52..+80, width 132, pencil-beam model). */
    scan_clear(scan);
    scan_face(scan, 9u, 14u, 300);
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U01", prof.b_is_valid, "profile valid");
    HOST_CHECK("S5-U01", OBS_WIDTH_MIN_MM == prof.width_mm,
               "width = 100 mm minimum after beam correction");
    HOST_CHECK("S5-U01", in_range((int32_t) prof.raw_width_mm, 126, 138),
               "measured span ~ 132 mm still reported");
    HOST_CHECK("S5-U01", in_range((int32_t) prof.nearest_mm, 296, 301),
               "nearest ~ 300 mm");
    HOST_CHECK("S5-U01", in_range(prof.x_left_mm, -55, -15)
               && (100 == (prof.x_right_mm - prof.x_left_mm)),
               "lateral extent 100 mm around the plateau middle (85..90 deg,"
               " S5-U51)");
    HOST_CHECK("S5-U01", (APP_PROFILE_CLEAR_MM == prof.clear_left_mm)
               && (APP_PROFILE_CLEAR_MM == prof.clear_right_mm),
               "no other objects: both sides clear");
    HOST_CHECK("S5-U01", OBS_OUT_PLAN == plan.outcome, "outcome PLAN");
    HOST_CHECK("S5-U01", APP_SIDE_LEFT == plan.side,
               "picks the smaller shift (left)");
    HOST_CHECK("S5-U01", (plan.leg_lateral_mm
                          == (uint32_t) (175 - prof.x_left_mm)),
               "lateral leg = corridor + uncertainty margin - x_left (S5-U48)");
    HOST_CHECK("S5-U01", plan.leg_forward_mm
               == (prof.nearest_mm + OBS_DEPTH_ASSUMED_MM + OBS_ROBOT_LEN_MM),
               "forward leg = nearest + depth + length");
    HOST_CHECK("S5-U01", !plan.b_was_clamped, "legs within NFR16");

    /* S5-U01: same face, an object 190 mm to the left blocks that side,
       so the plan goes right even though the shift is larger. */
    scan_face(scan, 18u, 18u, 329);   /* 120 deg, d ~ 380 */
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U01", prof.clear_left_mm < 250u, "left gap too small");
    HOST_CHECK("S5-U01", (OBS_OUT_PLAN == plan.outcome)
               && (APP_SIDE_RIGHT == plan.side), "falls back to the right");

    /* S5-U02: nothing near -> CONTINUE, profile not valid */
    scan_clear(scan);
    obs_profile_build(scan, 0u, &prof);
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
    obs_profile_build(scan, 0u, &prof);
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
    obs_profile_build(scan, 0u, &prof);
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
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U03", (prof.clear_left_mm < 250u)
               && (prof.clear_right_mm < 250u), "both gaps too small");
    HOST_CHECK("S5-U03", OBS_OUT_NO_PATH == plan.outcome,
               "both sides blocked: NO_PATH");
    HOST_CHECK("S5-U03", !plan.b_was_clamped, "not a clamp case");

    /* S5-U03: obstacle so far ahead that the forward leg (nearest +
       depth + robot length = 500 + 150 + 180 = 830 mm) is over the
       800 mm NFR16 limit -> clamped, NO_PATH. (2026-11-09: the old case,
       a wall 50..130 deg at 250 mm needing a 334 mm shift, is now planned
       as the 20 cm maximum obstacle, see below; S5-U45.)               */
    scan_clear(scan);
    scan_face(scan, 11u, 13u, 500);
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U03", plan.b_was_clamped, "forward leg over its limit");
    HOST_CHECK("S5-U03", OBS_OUT_NO_PATH == plan.outcome,
               "clipped plan is never driven: NO_PATH");
    HOST_CHECK("S5-U03", OBS_LEG_FWD_MAX_MM == plan.leg_forward_mm,
               "reported leg clipped to 800 mm");

    /* S5-U45: a flat face 50..130 deg at 250 mm is estimated at the
       200 mm maximum, centred on the middle of the near-equal plateau
       (70..110 deg -> 90 deg). 2026-11-09 18:10 (S5-U51): every return
       of this wall (<= 326 mm, within 30 deg of the estimate) is one a
       20 x 15 cm box could echo through the beam, so from one position
       it is planned as the professor's maximum box: PLAN 275 mm. (S5-U49
       had made it NO_PATH with a 24 mm smear range that also blocked
       every real car scan.) A wider wall still blocks: S5-U49 below.  */
    scan_clear(scan);
    scan_face(scan, 4u, 20u, 250);
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U45", (OBS_WIDTH_MAX_MM == prof.width_mm)
               && (-100 == prof.x_left_mm) && (100 == prof.x_right_mm),
               "wide face: 200 mm maximum centred on the shortest echo");
    /* 2026-11-09 18:40 (S5-U53): the wall's returns may be the box, so
       they widen the extent the bypass must clear (+-326 mm at 50/130
       deg) -> the shift is over 300 mm -> NO_PATH: the car stops.      */
    HOST_CHECK("S5-U45", OBS_OUT_NO_PATH == plan.outcome,
               "80 cm wall at 250 mm: path + margin cannot clear it, NO_PATH");

    /* S5-U23 (review V13-04): an estimate whose left edge sits exactly
       where the shift becomes 0 needs the leg OBS_LEG_MIN_MM, never 0.
       Since S5-U45/U48 the shift is (125 + OBS_EST_MARGIN_MM) - x_left
       and the estimate is the 100 mm minimum centred on the shortest
       echo's plateau, so that echo is put alone at x = +225 (40 deg,
       294 mm); the run blocks the corridor through its 65 deg point
       (296 mm, x = 125), which is not part of the plateau.            */
    scan_clear(scan);
    scan[2] = 294u;
    scan[3] = 320u;     /* outside the 16 mm plateau (S5-U51) */
    scan[4] = 320u;
    scan[5] = 320u;
    scan[6] = 320u;
    scan[7] = 296u;
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U23", prof.b_is_valid
               && ((int32_t) (OBS_ROBOT_HALF_W_MM + OBS_SIDE_MARGIN_MM
                              + OBS_EST_MARGIN_MM) == prof.x_left_mm),
               "estimate edge exactly at the shift-0 line (x_left = 175)");
    HOST_CHECK("S5-U23", (OBS_OUT_PLAN == plan.outcome)
               && (APP_SIDE_LEFT == plan.side)
               && (OBS_LEG_MIN_MM == plan.leg_lateral_mm),
               "lateral leg = OBS_LEG_MIN_MM, never 0");
    /* the edge beyond the shift-0 line (x_left > 175) must not wrap the
       unsigned shift (S5-U45 guard; since S5-U49 the outer returns may
       block the lanes, so the outcome need not be PLAN)               */
    scan[2] = 280u;                              /* x = 214 -> x_left 164 */
    scan[1] = 300u;                              /* 35 deg: x = 246       */
    scan[0] = 290u;                              /* 30 deg: x = 251       */
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U23", plan.leg_lateral_mm <= OBS_LEG_LAT_MAX_MM,
               "no unsigned wrap: leg within NFR16");

    /* S5-U44 (on-car 2026-11-06 14:19): box ~240 mm ahead at 65..90 deg,
       other objects behind it at 640 mm on both sides (30..60 and
       95..125 deg). Every step was < 650 mm, so the whole 30..125 deg
       arc was one "obstacle" 840 mm wide: lateral leg > 300 mm, NO_PATH
       twice, FAULT. A range jump > OBS_RUN_JUMP_MM now ends a run, and
       only points the bypass drives past (forward <= nearest + depth +
       robot length) limit the free gaps.                               */
    scan_clear(scan);
    scan_face(scan, 7u, 12u, 240);              /* box: 65..90 deg */
    for (uint32_t step = 0u; step <= 6u; step++)
    {
        scan[step] = 640u;                       /* right, behind it */
    }
    for (uint32_t step = 13u; step <= 19u; step++)
    {
        scan[step] = 640u;                       /* left, behind it */
    }
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    /* 2026-11-09 (S5-U45): the 25 deg box span is pulled in by the beam
       half-angle on each side -> 100 mm minimum centred on the middle
       of the 240 mm plateau (80..90 deg -> 80 deg, x ~ +42, S5-U48; was
       x 0..+112 with the pencil-beam model).                         */
    HOST_CHECK("S5-U44", prof.b_is_valid
               && (OBS_WIDTH_MIN_MM == prof.width_mm)
               && in_range((int32_t) prof.raw_width_mm, 100, 130),
               "box estimate 100 mm (span ~112), not the whole 30..125 arc");
    HOST_CHECK("S5-U44", in_range((int32_t) prof.nearest_mm, 235, 245)
               && in_range(prof.x_left_mm, -15, 0)
               && in_range(prof.x_right_mm, 85, 100),
               "box edges x ~ -8..+92 mm, nearest ~240 mm");
    HOST_CHECK("S5-U44", (OBS_OUT_PLAN == plan.outcome) && !plan.b_was_clamped
               && (plan.leg_lateral_mm <= OBS_LEG_LAT_MAX_MM),
               "objects behind the box do not block: PLAN, leg not clamped");
    /* S5-U53: gaps now measured from the extent (estimate + 50 mm and
       any box-like return): left 262, right 178 (too small)          */
    HOST_CHECK("S5-U44", in_range((int32_t) prof.clear_left_mm, 250, 275)
               && in_range((int32_t) prof.clear_right_mm, 165, 190)
               && (APP_SIDE_LEFT == plan.side),
               "gaps only from points beside the pass leg (the 60 and 120"
               " deg points at y 554, x +-320): pass left");
    /* separate objects (range jump) close beside the box, within the
       pass leg, must still block both sides                          */
    for (uint32_t step = 0u; step <= 6u; step++)
    {
        scan[step] = 450u;
    }
    for (uint32_t step = 13u; step <= 19u; step++)
    {
        scan[step] = 450u;
    }
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U44", OBS_OUT_NO_PATH == plan.outcome,
               "real objects beside the box: still NO_PATH");

    /* S5-U45 (on-car 2026-11-09 16:28): box straight ahead, nearest echo
       ~400 mm. The HC-SR04's wide cone kept hitting the box while the
       servo pointed well to either side: one run 636 mm "wide" ->
       lateral leg > 300 mm -> NO_PATH twice -> FAULT, although both gaps
       were 1020 (clear). Professor (forum 2026-10-25): 10..20 cm; use the
       maximum expected width with the scan, subtract the background,
       correct for the beam. Smear modelled on the 18:00 car scans: it
       grows with the angle off the box (+4 near it, up to +56 mm at
       50/130 deg; <= 30 deg beyond the estimate's edges, S5-U49/U51).
       Now: estimate within OBS_WIDTH_MIN_MM..OBS_WIDTH_MAX_MM, PLAN,
       legs in NFR16.                                                   */
    {
        static uint16_t const smear[9] =
        {
            396u, 400u, 404u, 404u, 420u, 428u, 436u, 444u, 452u
        };                                       /* 90, 85/95 ... 50/130 */

        scan_clear(scan);
        for (uint32_t off = 0u; off < 9u; off++)
        {
            scan[12u - off] = smear[off];
            scan[12u + off] = smear[off];
        }
    }
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U45", prof.b_is_valid
               && in_range((int32_t) prof.width_mm,
                           (int32_t) OBS_WIDTH_MIN_MM,
                           (int32_t) OBS_WIDTH_MAX_MM),
               "smeared box: width estimate 100..200 mm, not the span");
    HOST_CHECK("S5-U45", in_range((int32_t) prof.raw_width_mm, 540, 600),
               "the measured span (~580 mm) is still reported");
    HOST_CHECK("S5-U45", in_range((int32_t) prof.nearest_mm, 390, 400)
               && (12u == prof.angle_step),
               "nearest = the shortest echo (396 mm at 90 deg), not a"
               " beam-edge point's forward distance");
    /* S5-U53/U54: scanned from ~40 cm, the box's smear projects up to
       +-290 mm and may be the box: the shift needed is over 300 mm ->
       NO_PATH. The car now creeps to ~25 cm before it scans (S5-U54),
       where the same box gives the 18:00 scans of S5-U51.             */
    HOST_CHECK("S5-U45", OBS_OUT_NO_PATH == plan.outcome,
               "smear scanned from 40 cm: path + margin not provable, NO_PATH");

    /* S5-U49: the same smear reaching 45 and 135 deg (~31 deg beyond the
       200 mm estimate's edges) is no longer excused: those returns lie in
       both bypass lanes -> NO_PATH (the mission reverses 150 mm and
       rescans once, then stays stopped). Widen OBS_SMEAR_DEG only from
       car range logs.                                                  */
    scan[3]  = 460u;
    scan[21] = 460u;
    obs_profile_build(scan, 0u, &prof);
    obs_profile_plan(&prof, &plan);
    HOST_CHECK("S5-U49", (OBS_OUT_NO_PATH == plan.outcome)
               && (3u == prof.block_step_right)
               && (21u == prof.block_step_left),
               "smear 31 deg beyond the estimate: lanes unconfirmed");
    scan[3]  = (uint16_t) APP_DIST_NONE_MM;
    scan[21] = (uint16_t) APP_DIST_NONE_MM;

    /* S5-U46 (track 2026-11-09): box ahead, a chair beside the track on
       the left. 2026-11-09 18:40: the real 18:00 run 1 scan (from 248
       mm) has it: 620..628 mm at 130..140 deg (x ~ -400..-480) is a
       separate object 242 mm from the extent, less than the car plus
       margins (250) -> the car passes on the right.                  */
    {
        static uint16_t const chair_scan[APP_SCAN_STEPS] =
        {
            732u, 724u, 724u, 724u, 724u, 292u, 268u, 260u, 260u, 252u,
            244u, 244u, 244u, 240u, 240u, 248u, 252u, 252u, 268u, 272u,
            620u, 628u, 628u, 608u, 604u
        };

        obs_profile_build(chair_scan, 0x6u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U46", (prof.clear_left_mm < 250u)
                   && (20u == prof.block_step_left)
                   && (prof.clear_right_mm >= 250u),
                   "chair (130 deg, 620 mm) blocks the left gap, right clear");
        HOST_CHECK("S5-U46", (OBS_OUT_PLAN == plan.outcome)
                   && (APP_SIDE_RIGHT == plan.side) && !plan.b_was_clamped
                   && path_clear(chair_scan, &prof, &plan),
                   "passes on the side away from the chair, path + margin"
                   " clear");
    }

    /* S5-U48 (on-car 2026-11-09 17:00): the car passed left and its side
       hit the box (SIDE TOO CLOSE 52 mm, FAULT). Real scan below: the
       shortest echo, 252 mm, reads the same from 65 to 100 deg, so the
       box can be anywhere in that span. The estimate was centred on the
       first of those steps (75 deg, x +65) -> left edge -35, lateral leg
       160 mm. Now: centred on the middle of the equal-range plateau
       (~82 deg, x ~ +35..+45), and the lateral leg adds
       OBS_EST_MARGIN_MM for the estimate's uncertainty.               */
    {
        static uint16_t const car_scan[APP_SCAN_STEPS] =
        {
            1020u, 1020u, 296u, 272u, 256u, 256u, 256u, 252u, 252u, 252u,
            252u, 252u, 252u, 252u, 252u, 256u, 256u, 256u, 268u, 268u,
            268u, 568u, 544u, 544u, 544u
        };

        obs_profile_build(car_scan, 0u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U48", prof.b_is_valid
                   && in_range((prof.x_left_mm + prof.x_right_mm) / 2, -10, 10),
                   "estimate centred mid-plateau (16 mm: 50..130 deg ->"
                   " 90 deg, S5-U51), not on 75 deg");
        /* S5-U53: the 17:00 scene does not pass: the left gap to the
           135 deg object (568 mm) is 229 mm < 250, and the right extent
           reaches the 40 deg echo (296 mm, x +227) -> shift 351 > 300 */
        HOST_CHECK("S5-U48", OBS_OUT_NO_PATH == plan.outcome,
                   "17:00 scene: no lane with path + margin clear, NO_PATH");
    }

    /* S5-U49 (review 2026-11-09): every core return beside
       the estimate was skipped as "beam smear", so a return inside the
       bypass lane never blocked it. Now a core return is excused only if
       the box itself could have produced it: measured (not inferred),
       within OBS_SMEAR_DEG of that side's estimated edge, and at the box's
       own range (within OBS_SMEAR_RANGE_MM of the shortest echo). Any
       other return counts toward the free gap like a separate object. */
    {
        static uint16_t const car_scan[APP_SCAN_STEPS] =
        {
            1020u, 1020u, 296u, 272u, 256u, 256u, 256u, 252u, 252u, 252u,
            252u, 252u, 252u, 252u, 252u, 256u, 256u, 256u, 268u, 268u,
            268u, 568u, 544u, 544u, 544u
        };
        uint16_t scan_b[APP_SCAN_STEPS];
        /* the 11-09 17:00 scan's inferred steps (0xdb6db0) */
        uint32_t const inferred_1700 = 0xDB6DB0u;

        for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
        {
            scan_b[idx] = car_scan[idx];
        }
        /* Review case (restored 2026-11-09 18:40, S5-U53): one return
           at 130 deg, 300 mm (x ~ -193, in the planned left footprint):
           it may be the box's side face or another object - either way
           the car's path plus margin must clear it                     */
        scan_b[20] = 300u;
        obs_profile_build(scan_b, 0u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U49", (20u == prof.ext_step_left)
                   || (20u == prof.block_step_left),
                   "300 mm return at 130 deg is in the left extent or gap"
                   " (logged)");
        HOST_CHECK("S5-U49", !((OBS_OUT_PLAN == plan.outcome)
                               && !path_clear(scan_b, &prof, &plan))
                   && !((OBS_OUT_PLAN == plan.outcome)
                        && (APP_SIDE_LEFT == plan.side)),
                   "never a pass whose path + margin touches that return");

        /* uniform 300 mm at every angle: returns beyond 30 deg of the
           estimate (or farther than the box) are real -> NO_PATH       */
        for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
        {
            scan_b[idx] = 300u;
        }
        obs_profile_build(scan_b, 0u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U49", OBS_OUT_NO_PATH == plan.outcome,
                   "300 mm at every angle: both lanes unconfirmed, NO_PATH");

        /* the real scan, every step measured: the returns beside the box
           may be the box (own range, within 30 deg); since S5-U53 they
           widen the extent the bypass must clear                      */
        obs_profile_build(car_scan, 0u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U49", (OBS_OUT_PLAN != plan.outcome)
                   || path_clear(car_scan, &prof, &plan),
                   "measured scan: any PLAN keeps path + margin clear");

        /* the same values as logged, 14 steps inferred (125/130 deg were
           copies of 120 deg): an inferred step cannot be excused as
           smear, so the left lane (130 deg, x ~ -172) is unconfirmed  */
        obs_profile_build(car_scan, inferred_1700, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U49", (prof.clear_left_mm < 300u)
                   && !((OBS_OUT_PLAN == plan.outcome)
                        && (APP_SIDE_LEFT == plan.side)),
                   "inferred return in the left lane: no left pass");
    }

    /* S5-U51 (on-car 2026-11-09 18:00, two runs, both FAULT): every lane
       was blocked by the box's own side-face echoes (125 deg 272 mm,
       55 deg 292 mm vs. the box at 240 mm; after the reverse 130 deg
       432 mm, 65 deg 424 mm vs. 396 mm): OBS_SMEAR_RANGE_MM 24 was
       tighter than the box's depth. And the rescan's estimate was
       centred on one 396 mm step at 100 deg (shift > 300 mm, clamped)
       although the smear is symmetric about 90 deg. Now a core return
       within 30 deg of the edge is smear (core band 80 mm), and the
       centre is the middle of the steps within OBS_PLATEAU_MM 16 of the
       shortest echo. The real objects (600..800 mm) still count.      */
    {
        static uint16_t const run1_a[APP_SCAN_STEPS] =
        {
            732u, 724u, 724u, 724u, 724u, 292u, 268u, 260u, 260u, 252u,
            244u, 244u, 244u, 240u, 240u, 248u, 252u, 252u, 268u, 272u,
            620u, 628u, 628u, 608u, 604u
        };
        static uint16_t const run1_b[APP_SCAN_STEPS] =
        {
            844u, 844u, 844u, 1004u, 996u, 428u, 416u, 424u, 412u, 408u,
            404u, 404u, 408u, 396u, 396u, 412u, 400u, 412u, 420u, 416u,
            432u, 748u, 740u, 736u, 592u
        };
        static uint16_t const run2_a[APP_SCAN_STEPS] =
        {
            732u, 724u, 724u, 724u, 568u, 276u, 260u, 256u, 256u, 244u,
            244u, 244u, 244u, 244u, 244u, 248u, 256u, 252u, 264u, 296u,
            800u, 824u, 684u, 684u, 684u
        };
        static uint16_t const run2_b[APP_SCAN_STEPS] =
        {
            840u, 840u, 840u, 1020u, 1000u, 428u, 440u, 412u, 408u, 404u,
            400u, 400u, 404u, 408u, 396u, 408u, 400u, 412u, 416u, 412u,
            776u, 748u, 740u, 736u, 592u
        };
        static uint16_t const * const p_runs[4] =
        {
            run1_a, run1_b, run2_a, run2_b
        };
        static uint32_t const inferred[4] =
        {
            0x6u, 0x6u, 0xC00006u, 0x6u
        };
        uint32_t plans = 0u;
        uint32_t safe  = 0u;

        for (uint32_t run = 0u; run < 4u; run++)
        {
            obs_profile_build(p_runs[run], inferred[run], &prof);
            obs_profile_plan(&prof, &plan);
            if ((OBS_OUT_PLAN == plan.outcome) && !plan.b_was_clamped
                && (plan.leg_lateral_mm <= OBS_LEG_LAT_MAX_MM)
                && (in_range((prof.x_left_mm + prof.x_right_mm) / 2,
                             -45, 45)))
            {
                plans++;
            }
            if ((OBS_OUT_PLAN != plan.outcome)
                || path_clear(p_runs[run], &prof, &plan))
            {
                safe++;
            }
        }
        /* S5-U53/U54: the two scans from ~25 cm (run1_a, run2_a) plan with
           path + margin clear; the two from ~40 cm cannot (the creep now
           scans from ~25 cm)                                            */
        HOST_CHECK("S5-U51", 2u == plans,
                   "18:00 car scans from 25 cm: PLAN centred within 45 mm");
        HOST_CHECK("S5-U53", 4u == safe,
                   "every PLAN on the real car scans keeps the car's body"
                   " + 50 mm clear of every return");
        /* run 2 first scan: the 50 deg object (568 mm, x +365, y 435) is
           inside the pass leg and 244 mm from the estimate: right
           blocked by a REAL object, so the pass is to the left        */
        obs_profile_build(run2_a, 0xC00006u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U51", (4u == prof.block_step_right)
                   && (APP_SIDE_LEFT == plan.side),
                   "real 568 mm object at 50 deg still blocks the right");
    }

    /* 12:24:12 UTC: the box needed 334 mm on the right after the
       measured 45-degree echo widened the extent. */
    {
        static uint16_t const logged[APP_SCAN_STEPS] =
        {
            1020u, 1020u, 1020u, 296u, 292u, 296u, 268u, 256u, 252u,
            252u, 244u, 244u, 244u, 244u, 244u, 244u, 244u, 248u,
            256u, 260u, 268u, 276u, 292u, 708u, 704u
        };
        obs_profile_build(logged, 0u, &prof);
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U59", (OBS_OUT_PLAN == plan.outcome)
                   && (APP_SIDE_RIGHT == plan.side)
                   && (334u == plan.leg_lateral_mm)
                   && path_clear(logged, &prof, &plan),
                   "logged outer echo clears with the 334 mm extended pass");

        prof.clear_right_mm = 250u;
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U59", (OBS_OUT_NO_PATH == plan.outcome)
                   && (OBS_LEG_LAT_BASE_MM == plan.leg_lateral_mm),
                   "a return limiting that lane keeps the 300 mm cap");

        prof.clear_right_mm = APP_PROFILE_CLEAR_MM;
        prof.plan_left_mm = -226;
        prof.plan_right_mm = 226;
        obs_profile_plan(&prof, &plan);
        HOST_CHECK("S5-U59", (OBS_OUT_NO_PATH == plan.outcome)
                   && (OBS_LEG_LAT_MAX_MM == plan.leg_lateral_mm),
                   "a required shift of 351 mm still stops");
    }

    return (0 == HOST_SUMMARY("subsystem5 host")) ? 0 : 1;
}
