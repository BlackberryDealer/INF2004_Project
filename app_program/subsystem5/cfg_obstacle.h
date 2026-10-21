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
#define OBS_SLOW_MM            (400u)    /* -> APPROACH */

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
/* 251 mm with the placeholder values */
#define OBS_STOP_MM            (OBS_CLEARANCE_MIN_MM + OBS_FRONT_OFFSET_MM  \
                                + OBS_CONFIRM_TRAVEL_MM                     \
                                + OBS_LATENCY_TRAVEL_MM                     \
                                + OBS_BRAKE_DIST_MM + OBS_MARGIN_MM)

/* ---- confirmation and fault rules ------------------------------------ */
#define OBS_AGREE_MM           (30u)     /* two pings agree within        */
#define OBS_LOST_PINGS         (2u)      /* lost echoes in APPROACH: stop */
#define OBS_NO_ECHO_PINGS      (3u)      /* silent: FAULT_NO_ECHO (T10)   */
#define OBS_SETTLE_PINGS       (3u)      /* MEASURE: A9 (settle ms / 60)  */

/* ---- scan and bypass geometry ---------------------------------------- */
#define OBS_NEAR_MM            (400u)    /* scan point counts as obstacle */
#define OBS_SIDE_TOO_CLOSE_MM  (60u)
#define OBS_ROBOT_HALF_W_MM    (75u)     /* MEASURE */
#define OBS_ROBOT_LEN_MM       (180u)    /* MEASURE */
#define OBS_DEPTH_ASSUMED_MM   (150u)    /* obstacles <= 150 mm deep: ask lab */
#define OBS_SIDE_MARGIN_MM     (50u)
#define OBS_LEG_LAT_MAX_MM     (300u)    /* NFR16 plan limit */
#define OBS_LEG_FWD_MAX_MM     (800u)    /* NFR16 plan limit */

/* ---- servo (SG90, A9) ------------------------------------------------- */
#define OBS_SERVO_MIN_US       (500u)
#define OBS_SERVO_MAX_US       (2500u)
#define OBS_SERVO_TRIM_US      (0)       /* MEASURE */
#define OBS_WATCH_DEG_LEFT_PASS   (30u)  /* obstacle on robot's right */
#define OBS_WATCH_DEG_RIGHT_PASS  (150u)

#endif /* CFG_OBSTACLE_H */

/*** end of file ***/
