/**
 * @file    cfg_obstacle.h
 * @brief   Every obstacle tunable. The stop threshold is computed from
 *          measured values (A8, A9, A20, A21) and recomputes itself.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    cfg
 */

#ifndef CFG_OBSTACLE_H
#define CFG_OBSTACLE_H

#include <stdint.h>
#include "../app_types.h"
#include "../subsystem2/cfg_motion.h"

/* ---- ping timing and thresholds (Figure 6.4) ------------------------ */
#define OBS_PING_PERIOD_MS     (60u)
#define OBS_ECHO_TIMEOUT_MS    (40u)     /* MEASURE: A8 */
/* 400 -> 600 (on-car 2026-11-04, S5-U37): with the stop moved out to
   ~40 cm the slow-down starts 200 mm before it.                       */
#define OBS_SLOW_MM            (600u)    /* -> APPROACH */

/* ---- stop threshold (NFR6): computed, never typed in ----------------- */
#define OBS_V_MAX_MM_S         (300u)    /* NFR6 worst case */
#define OBS_CLEARANCE_MIN_MM   (100u)    /* NFR6 */
#define OBS_FRONT_OFFSET_MM    (60u)     /* MEASURE: A21 */
/* travel while two pings confirm the reading */
#define OBS_CONFIRM_TRAVEL_MM  ((OBS_V_MAX_MM_S * 2u * OBS_PING_PERIOD_MS) \
                                / 1000u)
/* travel during the 15 ms flag -> motors-low latency (NFR10a) */
#define OBS_LATENCY_TRAVEL_MM  (((OBS_V_MAX_MM_S * 15u) + 999u) / 1000u)
/* v^2 / 2a; cast: MOTION_BRAKE_DECEL_MM_S2 is a positive constant */
#define OBS_BRAKE_DIST_MM      ((OBS_V_MAX_MM_S * OBS_V_MAX_MM_S)           \
                                / (2u * (uint32_t) MOTION_BRAKE_DECEL_MM_S2))
#define OBS_MARGIN_MM          (20u)
/* Earlier stop requested on the car (2026-11-04, S5-U37): the computed
   minimum (251 mm) left the car ~180 mm from the obstacle, too late to
   plan a clean bypass; stop at about 40 cm instead.                    */
#define OBS_EARLY_STOP_MM      (150u)    /* TUNE */
/* 401 mm with the placeholder values (251 computed + 150 requested) */
#define OBS_STOP_MM            (OBS_CLEARANCE_MIN_MM + OBS_FRONT_OFFSET_MM  \
                                + OBS_CONFIRM_TRAVEL_MM                     \
                                + OBS_LATENCY_TRAVEL_MM                     \
                                + OBS_BRAKE_DIST_MM + OBS_MARGIN_MM         \
                                + OBS_EARLY_STOP_MM)

/* ---- confirmation and fault rules ------------------------------------ */
#define OBS_AGREE_MM           (30u)     /* two pings agree within        */
#define OBS_LOST_PINGS         (2u)      /* lost echoes in APPROACH: stop */
#define OBS_NO_ECHO_PINGS      (3u)      /* silent: FAULT_NO_ECHO (T10)   */
/* Power-up grace (S5-U30): the HC-SR04P on the car answers only some
   time after power-up, and the first silent pings latched NO_ECHO at
   every boot (measured 2026-10-22). Silence is not counted until the
   first echo edge, or until this many pings (~2 s at 60 ms) have
   passed, so an unplugged sensor still raises NO_ECHO.               */
#define OBS_WARMUP_PINGS       (34u)     /* MEASURE: first echo after boot */
#define OBS_SETTLE_PINGS       (3u)      /* MEASURE: A9 (settle ms / 60)  */
/* S5-U52 (2026-11-09 car scans took 8.3 s): a move of at most one
   coarse stride (15 deg, SG90 ~25 ms) settles in one 60 ms ping.      */
#define OBS_SETTLE_SMALL_PINGS (1u)
#define OBS_WATCH_SETTLE_PINGS (4u)      /* MEASURE: A9, worst 120 deg move */

/* C.4b: the settle countdown (PRV_OBS_A SETTLE) is 3 bits, the lost and
   silent counters (LOST, SILENT) 2 bits each.                        */
#if ((OBS_SETTLE_PINGS > 7u) || (OBS_WATCH_SETTLE_PINGS > 7u))
#error "Settle counts no longer fit the 3-bit SETTLE field: widen it"
#endif
#if ((OBS_LOST_PINGS > 3u) || (OBS_NO_ECHO_PINGS > 3u))
#error "Lost/silent ping limits no longer fit their 2-bit counters"
#endif

/* ---- scan and bypass geometry ---------------------------------------- */
#define OBS_LEG_MIN_MM         (20u)     /* shorter legs are rounded up   */
/* 400 -> 650 (2026-11-04): the car now stops ~400 mm away, so the
   obstacle's own scan points (further off-axis) must still count.     */
#define OBS_NEAR_MM            (650u)    /* scan point counts as obstacle */
/* Neighbouring scan steps whose ranges differ by more than this are two
   objects (on-car 2026-11-06: a box at 240 mm merged with objects at
   ~600 mm beside it into one 840 mm "obstacle" -> NO_PATH, S5-U44). A
   flat face changes < 80 mm per 15 deg coarse step at <= 300 mm.      */
#define OBS_RUN_JUMP_MM        (150u)
#define OBS_SIDE_TOO_CLOSE_MM  (60u)
#define OBS_ROBOT_HALF_W_MM    (75u)     /* MEASURE */
#define OBS_ROBOT_LEN_MM       (180u)    /* MEASURE */
#define OBS_DEPTH_ASSUMED_MM   (150u)    /* obstacles <= 150 mm deep: ask lab */
#define OBS_SIDE_MARGIN_MM     (50u)
#define OBS_LEG_LAT_BASE_MM    (300u)    /* normal plan limit (NFR16) */
#define OBS_LEG_LAT_MAX_MM     (350u)    /* extended track-test limit */
#define OBS_EXT_ECHO_LEAD_MM   (12u)     /* three 4 mm range steps */
#define OBS_LEG_FWD_MAX_MM     (800u)    /* NFR16 plan limit */

/* ---- beam-aware width estimate (S5-U45, S5-U46) ----------------------- */
/* On-car 2026-11-09: the HC-SR04's ~30 deg cone kept hitting a box 400 mm
   ahead while the servo pointed up to 45 deg off-axis, so the scan saw a
   636 mm "wall" and planned NO_PATH with both gaps clear. Professor
   (forum 2026-10-25): obstacles are 10..20 cm wide; use the maximum
   expected width with the scan, background subtraction, consecutive-
   angle detection with hysteresis and conservative margins.
   - Background subtraction: only steps whose range is within
     OBS_CORE_BAND_MM of the shortest echo belong to the obstacle (core);
     farther steps are other objects / background.
   - Hysteresis: one out-of-band step between core steps does not split
     the core (OBS_CORE_GAP_STEPS).
   - Beam correction: each end of the core's angular span is pulled in by
     the beam half-angle (OBS_BEAM_HALF_STEPS x 5 deg), never past the
     shortest echo.
   - The estimate is kept within OBS_WIDTH_MIN_MM..OBS_WIDTH_MAX_MM; the
     raw span is still reported (uncertainty, console + profile).      */
#define OBS_CORE_BAND_MM       (80u)     /* TUNE: > box face curvature   */
#define OBS_CORE_GAP_STEPS     (1u)      /* hysteresis: bridged dropouts */
#define OBS_BEAM_HALF_STEPS    (3u)      /* MEASURE: ~15 deg half-beam  */
#define OBS_WIDTH_MIN_MM       (100u)    /* professor: 10..20 cm        */
#define OBS_WIDTH_MAX_MM       (200u)
/* S5-U48 (on-car 2026-11-09 17:00: the car's side hit the box). The
   shortest echo often reads the same over several steps (252 mm from 65
   to 100 deg): the estimate is centred on the middle of those steps
   (ranges within OBS_PLATEAU_MM of the shortest), and the lateral leg
   adds OBS_EST_MARGIN_MM for the estimate's uncertainty (professor:
   conservative safety margins); the free gap must allow for it too.  */
/* 8 -> 16 (on-car 2026-11-09 18:00, S5-U51): with 8 mm one 396 mm step at
   100 deg was the whole plateau, the estimate sat 70 mm left of the
   symmetric smear (55..125 deg) and the shift went over 300 mm.       */
#define OBS_PLATEAU_MM         (16u)     /* 4 x 4 mm range resolution   */
#define OBS_EST_MARGIN_MM      (50u)     /* TUNE */
/* S5-U49 (review 2026-11-09): a core return beside the estimate is
   excused as beam smear only if the obstacle itself could have produced
   it: measured (not inferred), within OBS_SMEAR_DEG of that side's
   estimated edge and at the obstacle's own range (within
   OBS_SMEAR_RANGE_MM of its shortest echo). Every other return counts
   toward the free gap like a separate object (lane unconfirmed).
   OBS_SMEAR_SIN_NUM / _DEN = sin(OBS_SMEAR_DEG).                       */
#define OBS_SMEAR_DEG          (30u)     /* TUNE: car scans reach ~30 deg */
#define OBS_SMEAR_SIN_NUM      (1u)
#define OBS_SMEAR_SIN_DEN      (2u)
/* 24 -> 80 = OBS_CORE_BAND_MM (on-car 2026-11-09 18:00, S5-U51): pointed
   off-axis, the beam echoes off the box's side face and back corner,
   +28..+52 mm beyond its front (and up to its depth): 24 mm blocked both
   lanes on every scan. A core return can be the box; beyond the core
   band (or > OBS_SMEAR_DEG off its edge, or inferred) it is real.     */
#define OBS_SMEAR_RANGE_MM     (OBS_CORE_BAND_MM)

/* ---- creep to the scan distance (S5-U54, S5-U55) ---------------------- */
/* Track review 2026-11-09: the box smear projects into the bypass lanes
   from ~40 cm (every 18:00 rescan there was ambiguous) but not from
   ~25 cm. The car still stops at OBS_STOP_MM (~40 cm, braking buffer),
   then the mission creeps it forward to OBS_SCAN_MM under sonar control
   (obstacle_task brakes on a confirmed reading <= OBS_SCAN_MM, a lost
   echo or disagreeing readings) and scans at standstill. Within
   OBS_CREEP_TOL_MM of OBS_SCAN_MM it scans at once. On car: check >=
   100 mm physical clearance after braking.                            */
#define OBS_SCAN_MM            (250u)    /* TUNE: sonar range at the scan */
#define OBS_CREEP_TOL_MM       (30u)

/* ---- repeated measurements per scan step (S5-U47) ---------------------- */
/* After the servo settles, OBS_SCAN_SAMPLES pings are taken and their
   median is stored (professor: "repeated median-filtered measurements
   after allowing the servo to settle"); one bad echo per step no longer
   becomes a scan point. Must be odd.                                   */
#define OBS_SCAN_SAMPLES       (3u)

#if ((OBS_SCAN_SAMPLES != 3u))
#error "obstacle_median3() takes exactly three samples"
#endif
#if ((OBS_SCAN_MM + OBS_CREEP_TOL_MM) >= OBS_SLOW_MM)
#error "Scan distance must lie inside the slow-down range"
#endif
#if (OBS_SMEAR_DEG != 30u)
#error "OBS_SMEAR_SIN_NUM / _DEN hold sin(30 deg): update them too"
#endif
#if (OBS_WIDTH_MIN_MM > OBS_WIDTH_MAX_MM)
#error "Width estimate limits are reversed"
#endif

/* ---- ramp vs. obstacle discrimination (S5-U42) ----------------------- */
/* While climbing or descending a ramp/hump (or within OBS_RAMP_CLEAR_MM /
   OBS_RAMP_HOLD_PINGS after levelling off), the ultrasonic beam hits the
   inclined ramp face or the floor at the foot of the down-slope. A sloped
   ramp scanned from flat ground before the wheels touch it also returns
   only a narrow normal-incidence glint (< OBS_MIN_WIDTH_MM, whereas real
   obstacles are >= 80 mm wide per A29).                                  */
#define OBS_RAMP_PITCH_DDEG    (20)      /* 2.0 deg (above 1.5 deg level) */
#define OBS_RAMP_HOLD_PINGS    (8u)      /* ~480 ms hold across crest/exit */
#define OBS_RAMP_CLEAR_MM      (150u)    /* flat travel to re-arm after   */
#define OBS_MIN_WIDTH_MM       (28u)     /* < 28 mm scan glint = ramp     */
/* While pitched (gb_on_ramp) the early-stop margin is dropped, but the
   computed NFR6 minimum still stops the car, so a real obstacle on or
   right after the ramp is never driven into (S5-U42). The pitch alone
   never overrides a scan: only a narrow glint does, so a real box still
   gets the normal reverse / bypass (on-car log 2026-11-05).            */
#define OBS_RAMP_STOP_MM       (OBS_STOP_MM - OBS_EARLY_STOP_MM) /* 251 */

/* ---- servo (SG90, A9) ------------------------------------------------- */
#define OBS_SERVO_MIN_US       (500u)
#define OBS_SERVO_MAX_US       (2500u)
#define OBS_SERVO_TRIM_US      (0)       /* MEASURE */
#define OBS_WATCH_DEG_LEFT_PASS   (30u)  /* obstacle on robot's right */
#define OBS_WATCH_DEG_RIGHT_PASS  (150u)

/* C.4b: sentinels stay outside the valid range and fit their fields
   (DIST 9 bits x 4 mm, profile fields 8 bits x 4 mm, A27).          */
/* stop < slow-down <= scan near threshold: checked by S5-U37 (OBS_STOP_MM
   holds a cast, which #if cannot evaluate) */
#if (OBS_NEAR_MM >= APP_PROFILE_CLEAR_MM)
#error "Near threshold must stay below the profile 'clear' sentinel"
#endif
#if ((APP_DIST_NONE_MM / 4u) != 511u) || ((APP_PROFILE_CLEAR_MM / 4u) != 255u)
#error "Distance sentinels no longer match their packed fields"
#endif
#if (OBS_LEG_LAT_BASE_MM > OBS_LEG_LAT_MAX_MM)
#error "The extended lateral limit must cover the normal limit"
#endif
#if (OBS_LEG_LAT_MAX_MM > 1020u) || (OBS_LEG_FWD_MAX_MM > 1020u)
#error "Bypass leg limits no longer fit LEG_LAT/LEG_FWD (8 bits x 4 mm)"
#endif

#endif /* CFG_OBSTACLE_H */

/*** end of file ***/
