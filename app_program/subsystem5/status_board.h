/**
 * @file    status_board.h
 * @brief   Status board K6 + MTX_STATUS: one writer per part, getters.
 * @owner   Buddy 5 (shared services)
 * @kind    task
 */

#ifndef STATUS_BOARD_H
#define STATUS_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

typedef struct                  /* writer: line_task (B3) */
{
    int32_t  steering;          /* -1000..+1000, see app_types.h sign */
    uint32_t line_status;       /* APP_LINE_*  */
    uint32_t junction;          /* APP_JCT_* (last class seen) */
    bool     b_is_calibrated;
} status_board_line_t;

typedef struct                  /* writer: motion_task (B2) */
{
    int32_t  speed_l_cpw;       /* -127..+127 counts per 50 ms window */
    int32_t  speed_r_cpw;
    uint32_t motion_state;      /* APP_MOT_* */
    int32_t  pos_l_counts;      /* -131071..+131071 */
    int32_t  pos_r_counts;
    uint32_t path_counts;       /* 0..131071, sum of |travel| */
} status_board_motion_t;

typedef struct                  /* writer: imu_task (B4) */
{
    int32_t  pitch_ddeg;        /* -450..+450 (0.1 deg) */
    uint32_t hump_now_mm;       /* 0..127 */
    uint32_t hump_peak_mm;      /* 0..127, highest this run */
    uint32_t motion_class;      /* APP_MCLASS_* */
    bool     b_is_turn_from_enc;/* true: turn rate came from encoders */
    int32_t  turn_rate_dps;     /* -360..+360 */
} status_board_imu_t;

typedef struct                  /* writer: obstacle_task (B5) */
{
    uint32_t distance_mm;       /* 0..2044; APP_DIST_NONE_MM = nothing */
    uint32_t scanner_state;     /* APP_SCAN_* */
    uint32_t angle_step;        /* 0..24 => 30 + 5*step degrees */
    uint32_t plan_side;         /* APP_SIDE_* */
    bool     b_is_profile_valid;
    uint32_t width_mm;          /* 0..1020; APP_PROFILE_CLEAR_MM = clear */
    uint32_t nearest_mm;
    uint32_t clear_left_mm;
    uint32_t clear_right_mm;
    uint32_t leg_lateral_mm;    /* 0..1020, planned bypass legs (RNG) */
    uint32_t leg_forward_mm;
} status_board_obs_t;

typedef struct                  /* writer: mission_task (B5) */
{
    uint32_t sys_state;         /* APP_SYS_* */
    uint32_t run_state;         /* APP_RUN_* */
    uint32_t avoid_step;        /* APP_AVOID_* */
    uint32_t end_reason;        /* APP_END_* */
    bool     b_is_armed;
    uint32_t armed_cmd;         /* APP_BAR_CMD_* */
} status_board_mission_t;

/* Snapshot. Phase 2: five part structs. Phase 4: 11 packed words.
   NOBODY outside status_board.c touches its members. */
typedef struct
{
    status_board_line_t    line;
    status_board_motion_t  motion;
    status_board_imu_t     imu;
    status_board_obs_t     obs;
    status_board_mission_t mission;
} status_board_t;

extern ER       status_board_init(void);   /* creates MTX_STATUS */
extern void     status_board_write_line(status_board_line_t const * p_part);
extern void     status_board_write_motion(status_board_motion_t const * p_part);
extern void     status_board_write_imu(status_board_imu_t const * p_part);
extern void     status_board_write_obstacle(status_board_obs_t const * p_part);
extern void     status_board_write_mission(
                    status_board_mission_t const * p_part);
extern void     status_board_read(status_board_t * p_snapshot);

/* Getters (pure; work on a snapshot) */
extern int32_t  status_board_steering(status_board_t const * p_snap);
extern uint32_t status_board_line_status(status_board_t const * p_snap);
extern uint32_t status_board_junction(status_board_t const * p_snap);
extern bool     status_board_is_calibrated(status_board_t const * p_snap);
extern int32_t  status_board_speed_l(status_board_t const * p_snap);
extern int32_t  status_board_speed_r(status_board_t const * p_snap);
extern uint32_t status_board_motion_state(status_board_t const * p_snap);
extern int32_t  status_board_pos_l(status_board_t const * p_snap);
extern int32_t  status_board_pos_r(status_board_t const * p_snap);
extern uint32_t status_board_path(status_board_t const * p_snap);
extern int32_t  status_board_pitch(status_board_t const * p_snap);
extern uint32_t status_board_hump_now(status_board_t const * p_snap);
extern uint32_t status_board_hump_peak(status_board_t const * p_snap);
extern uint32_t status_board_motion_class(status_board_t const * p_snap);
extern bool     status_board_is_turn_from_enc(status_board_t const * p_snap);
extern int32_t  status_board_turn_rate(status_board_t const * p_snap);
extern uint32_t status_board_distance(status_board_t const * p_snap);
extern uint32_t status_board_scanner_state(status_board_t const * p_snap);
extern uint32_t status_board_angle_step(status_board_t const * p_snap);
extern uint32_t status_board_plan_side(status_board_t const * p_snap);
extern bool     status_board_is_profile_valid(status_board_t const * p_snap);
extern uint32_t status_board_width(status_board_t const * p_snap);
extern uint32_t status_board_nearest(status_board_t const * p_snap);
extern uint32_t status_board_clear_left(status_board_t const * p_snap);
extern uint32_t status_board_clear_right(status_board_t const * p_snap);
extern uint32_t status_board_leg_lateral(status_board_t const * p_snap);
extern uint32_t status_board_leg_forward(status_board_t const * p_snap);
extern uint32_t status_board_sys_state(status_board_t const * p_snap);
extern uint32_t status_board_run_state(status_board_t const * p_snap);
extern uint32_t status_board_avoid_step(status_board_t const * p_snap);
extern uint32_t status_board_end_reason(status_board_t const * p_snap);
extern bool     status_board_is_armed(status_board_t const * p_snap);
extern uint32_t status_board_armed_cmd(status_board_t const * p_snap);

#endif /* STATUS_BOARD_H */

/*** end of file ***/
