/**
 * @file    app_types.h
 * @brief   Codes, robot constants and conventions shared by all subsystems.
 * @owner   Team (change only by SHARED pull request)
 * @kind    cfg
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Encoding kinds used by tools/layout.csv ------------------------- */
#define APP_KIND_SAT    (0u)  /* top code is a sentinel; clamp is silent    */
#define APP_KIND_RNG    (1u)  /* range assumption; clamp sets RANGE_CLAMP   */
#define APP_KIND_ENUM   (2u)  /* code from a list below                     */
#define APP_KIND_FLAG   (3u)  /* single bit                                 */
#define APP_KIND_PAD    (4u)  /* unused bits, always zero                   */
#define APP_KIND_RAW    (5u)  /* hardware image or wrapping counter         */

/* ---- Robot physical constants (rows owned by Buddy 2) ---------------- */
#define APP_TICK_MS              (10u)
#define APP_SPEED_WINDOW_MS      (50u)
/* A4, car 2026-10-29: about one hand turn gave ~2560 PIO quadrature edges
   per wheel turn (L -2543, R +2582). drv_encoder.c divides the edge count
   by 4, so the firmware sees ~640 counts per turn (refine with the 10-turn
   test). Wheel diameter measured 66 mm: 66 x pi = 207.3 mm. The physical
   ceiling is 480 mm/s (was 500) so the per-tick delta keeps its 7 bits:
   15 counts x (1 + 3 late ticks) = 60 <= 63. Still above every commanded
   speed (<= 300 mm/s).                                                   */
#define APP_COUNTS_PER_REV       (640u)    /* measured ~, A4 (see above)  */
#define APP_WHEEL_CIRC_UM        (207345u) /* measured: 66 mm x pi        */
#define APP_TRACK_UM             (125000u) /* MEASURE: wheel centre-centre */
#define APP_SPEED_PHYS_MAX_MM_S  (480u)    /* above any commanded speed   */
#define APP_LATE_TICKS_MAX       (3u)      /* a read may be 3 ticks late  */
#define APP_RUN_MAX_MM           (32000u)  /* A23: one run <= 32 m        */
#define APP_RUN_MAX_S            (1023u)   /* A28: one run <= 17 min      */

#define APP_UM_PER_COUNT  ((APP_WHEEL_CIRC_UM + (APP_COUNTS_PER_REV / 2u)) \
                           / APP_COUNTS_PER_REV)
#define APP_TICK_COUNTS_MAX  (((APP_SPEED_PHYS_MAX_MM_S * APP_TICK_MS        \
                                * APP_COUNTS_PER_REV) + APP_WHEEL_CIRC_UM   \
                               - 1u) / APP_WHEEL_CIRC_UM)
#define APP_WINDOW_COUNTS_MAX (((APP_SPEED_PHYS_MAX_MM_S                     \
                                 * APP_SPEED_WINDOW_MS * APP_COUNTS_PER_REV) \
                                + APP_WHEEL_CIRC_UM - 1u) / APP_WHEEL_CIRC_UM)
#define APP_RUN_MAX_COUNTS   ((APP_RUN_MAX_MM * 1000u) / APP_UM_PER_COUNT)

#if ((APP_TICK_COUNTS_MAX * (1u + APP_LATE_TICKS_MAX)) > 63u)
#error "Encoder delta no longer fits 7 bits: widen it in tools/layout.csv"
#endif
#if (APP_WINDOW_COUNTS_MAX > 127u)
#error "Wheel speed no longer fits 8 bits: widen it in tools/layout.csv"
#endif
#if (APP_RUN_MAX_COUNTS > 131071u)
#error "Path length no longer fits 17 bits: widen it in tools/layout.csv"
#endif

/* ---- Conventions (every buddy relies on these) ------------------------
 * Steering error: +1000 = line fully under the LEFT sensor, i.e. the line
 *   is LEFT of the robot centre; the robot must turn LEFT (slow left wheel).
 * Wheel speed and position: + = wheel driving the robot forward.
 * Heading (odometry): + = robot has turned RIGHT (clockwise seen from above).
 * Pitch: + = nose up.  Turn rate: + = turning right.
 * Servo angle: 0 deg = robot's right, 90 deg = straight ahead, 180 = left.
 * Distances to obstacles: mm, stored in 4 mm steps.
 * ---------------------------------------------------------------------- */

/* ---- App event types: 5 bits (K4 to mission, K7 to telem) ------------ */
#define APP_EVT_START            (0u)
#define APP_EVT_STOP             (1u)
#define APP_EVT_RESET            (2u)
#define APP_EVT_JUNCTION         (3u)
#define APP_EVT_LINE_LOST        (4u)
#define APP_EVT_LINE_FOUND       (5u)
#define APP_EVT_BARCODE          (6u)
#define APP_EVT_UNMAPPED         (7u)
#define APP_EVT_INVALID          (8u)
#define APP_EVT_OBSTACLE_AHEAD   (9u)
#define APP_EVT_OBSTACLE         (10u)
#define APP_EVT_PLAN             (11u)
#define APP_EVT_CONTINUE         (12u)
#define APP_EVT_NO_PATH          (13u)
#define APP_EVT_SIDE_TOO_CLOSE   (14u)
#define APP_EVT_MOVE_DONE        (15u)
#define APP_EVT_STALL            (16u)
#define APP_EVT_HUMP             (17u)
#define APP_EVT_MOTION_CLASS     (18u)
#define APP_EVT_CAL_DONE         (19u)
#define APP_EVT_SENSOR_FAULT     (20u)
#define APP_EVT_STATE_CHANGE     (21u)
#define APP_EVT_SUMMARY          (22u)
#define APP_EVT_MAP              (23u)
#define APP_EVT_COUNT            (24u)
#define APP_EVT_TYPE_BITS        (5u)

#if (APP_EVT_COUNT > 32u)
#error "Event types no longer fit the 5-bit type field"
#endif

/* ---- Mode request bits: FLG_MODE (K10), written only by mission ------ */
#define APP_MODE_CAL_IMU          (0x01u)
#define APP_MODE_CAL_IR           (0x02u)
#define APP_MODE_STOPPED          (0x04u)  /* obstacle: robot has stopped */
#define APP_MODE_RESCAN           (0x08u)  /* obstacle: scan again        */
#define APP_MODE_RESUME           (0x10u)  /* obstacle: back to MONITOR   */
#define APP_MODE_BARCODE_INHIBIT  (0x20u)
#define APP_MODE_JUNCTION_INHIBIT (0x40u)
#define APP_MODE_RUN_START        (0x80u)  /* new run: reset per-run state */
#define APP_MODE_MOTOR_EN         (0x100u) /* motion may drive (FR17)     */
#define APP_MODE_ALL              (0x1FFu)

/* ---- Fault bits: FLG_FAULT (K12) -------------------------------------- */
#define APP_FAULT_IDX_MOTOR_STALL    (0u)
#define APP_FAULT_IDX_ENCODER        (1u)
#define APP_FAULT_IDX_IMU            (2u)
#define APP_FAULT_IDX_NO_ECHO        (3u)
#define APP_FAULT_IDX_WIFI_LOSS      (4u)
#define APP_FAULT_IDX_QUEUE_OVERFLOW (5u)
#define APP_FAULT_IDX_RANGE_CLAMP    (6u)
#define APP_FAULT_IDX_CAL_TIMEOUT    (7u)
#define APP_FAULT_IDX_REACQUIRE_FAIL (8u)
#define APP_FAULT_IDX_PIO_LOAD       (9u)
#define APP_FAULT_COUNT              (10u)

#define APP_FAULT_MOTOR_STALL     (0x001u)
#define APP_FAULT_ENCODER         (0x002u)
#define APP_FAULT_IMU             (0x004u)
#define APP_FAULT_NO_ECHO         (0x008u)
#define APP_FAULT_WIFI_LOSS       (0x010u)  /* informational */
#define APP_FAULT_QUEUE_OVERFLOW  (0x020u)
#define APP_FAULT_RANGE_CLAMP     (0x040u)  /* informational */
#define APP_FAULT_CAL_TIMEOUT     (0x080u)
#define APP_FAULT_REACQUIRE_FAIL  (0x100u)
#define APP_FAULT_PIO_LOAD        (0x200u)
#define APP_FAULT_ALL             (0x3FFu)
#define APP_FAULT_STOP_MASK       (0x3AFu)  /* all except WIFI, RANGE */

/* ---- Codes (all stored as uint32_t; no C enums in this project) ------ */
#define APP_LINE_ON           (0u)
#define APP_LINE_LOST         (1u)
#define APP_LINE_JUNCTION     (2u)
#define APP_LINE_CALIBRATING  (3u)

#define APP_JCT_NONE    (0u)
#define APP_JCT_T       (1u)
#define APP_JCT_CROSS   (2u)
#define APP_JCT_SIDE_L  (3u)
#define APP_JCT_SIDE_R  (4u)

#define APP_BAR_CMD_LEFT      (0u)
#define APP_BAR_CMD_RIGHT     (1u)
#define APP_BAR_CMD_STRAIGHT  (2u)
#define APP_BAR_CMD_UTURN     (3u)

/* Code 39 character index: '0'-'9' = 0-9, 'A'-'Z' = 10-35, '-' = 36,
   '.' = 37, ' ' = 38, '$' = 39, '/' = 40, '+' = 41, '%' = 42.          */
#define APP_C39_CHAR_COUNT    (43u)

#define APP_MOT_BRAKE      (0u)
#define APP_MOT_FOLLOW     (1u)
#define APP_MOT_MOVE_FWD   (2u)
#define APP_MOT_MOVE_BACK  (3u)
#define APP_MOT_TURN_L     (4u)
#define APP_MOT_TURN_R     (5u)
#define APP_MOT_ESTOP      (6u)
#define APP_MOT_STALLED    (7u)

#define APP_CMD_STOP    (0u)
#define APP_CMD_FOLLOW  (1u)
#define APP_CMD_FWD     (2u)
#define APP_CMD_BACK    (3u)
#define APP_CMD_LEFT    (4u)
#define APP_CMD_RIGHT   (5u)
#define APP_CMD_ZERO    (6u)   /* rebase odometry; BRAKE only; acks MOVE_DONE */

#define APP_MCLASS_STATIONARY    (0u)
#define APP_MCLASS_ACCELERATING  (1u)
#define APP_MCLASS_TURNING       (2u)
#define APP_MCLASS_CLIMBING      (3u)
#define APP_MCLASS_DESCENDING    (4u)
#define APP_MCLASS_IMPACT        (5u)
#define APP_MCLASS_CRUISING      (6u)  /* moving steadily: none of the above */

#define APP_SYS_BOOT            (0u)
#define APP_SYS_CAL_IMU         (1u)
#define APP_SYS_CAL_IR          (2u)
#define APP_SYS_READY           (3u)
#define APP_SYS_RUN             (4u)
#define APP_SYS_FINISHED        (5u)
#define APP_SYS_LINE_LOST_STOP  (6u)
#define APP_SYS_FAULT           (7u)

#define APP_RUN_FOLLOW        (0u)
#define APP_RUN_ARMED         (1u)
#define APP_RUN_EXECUTE_TURN  (2u)
#define APP_RUN_SEARCH_LINE   (3u)
#define APP_RUN_AVOID         (4u)

#define APP_AVOID_WAIT_PLAN      (0u)
#define APP_AVOID_TURN_OUT       (1u)
#define APP_AVOID_LEG_OUT        (2u)
#define APP_AVOID_TURN_PARALLEL  (3u)
#define APP_AVOID_LEG_PASS       (4u)
#define APP_AVOID_REJOIN         (5u)
#define APP_AVOID_REVERSE        (6u)

#define APP_END_STOP        (0u)
#define APP_END_MARKER      (1u)
#define APP_END_LINE_LOST   (2u)
#define APP_END_FAULT       (3u)

#define APP_SCAN_MONITOR       (0u)
#define APP_SCAN_APPROACH      (1u)
#define APP_SCAN_STOPPED       (2u)
#define APP_SCAN_COARSE        (3u)
#define APP_SCAN_FINE          (4u)
#define APP_SCAN_PASSING       (5u)
#define APP_SCAN_BYPASS_WATCH  (6u)
#define APP_SCAN_WATCH_SETTLE  (7u)   /* servo moving to the watch angle */

#define APP_SIDE_LEFT    (0u)
#define APP_SIDE_RIGHT   (1u)
#define APP_CAL_SRC_IMU  (0u)
#define APP_CAL_SRC_IR   (1u)
#define APP_STALL_LEFT   (0x1u)
#define APP_STALL_RIGHT  (0x2u)

/* ---- Obstacle geometry ------------------------------------------------ */
#define APP_DIST_NONE_MM         (2044u) /* distance sentinel: nothing  */
#define APP_PROFILE_CLEAR_MM     (1020u) /* profile sentinel: clear     */
#define APP_SCAN_ANGLE_MIN_DEG   (30u)
#define APP_SCAN_ANGLE_STEP_DEG  (5u)
#define APP_SCAN_STEPS           (25u)   /* 30..150 deg in 5 deg steps  */

/* ---- MQTT format (shared with the laptop decoder) --------------------- */
#define APP_MQ_VERSION        (1u)
#define APP_MQ_TYPE_STATE     (0u)
#define APP_MQ_TYPE_EVENT     (1u)
#define APP_MQ_TYPE_PROFILE   (2u)
#define APP_MQ_TYPE_SUMMARY   (3u)
#define APP_MQ_TYPE_HEARTBEAT (4u)
#define APP_MQ_TYPE_ONLINE    (5u)
#define APP_MQ_TYPE_OFFLINE   (6u)
#define APP_MQ_TYPE_CMD_START (16u)
#define APP_MQ_TYPE_CMD_STOP  (17u)
#define APP_MQ_TYPE_CMD_RESET (18u)
#define APP_MQ_TYPE_CMD_MAP   (19u)
#define APP_MQ_PAYLOAD_MAX    (20u)   /* bytes; largest is state = 18 */

#endif /* APP_TYPES_H */

/*** end of file ***/
