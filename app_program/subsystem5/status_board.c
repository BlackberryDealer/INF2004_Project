/**
 * @file    status_board.c
 * @brief   Status board K6 and MTX_STATUS: the 11 packed SB_* words of
 *          tools/layout.csv (min-bit item 6), written whole under the
 *          mutex, so readers always get one consistent snapshot.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    min-bit item 6 (shared data), NFR11
 */

#include "status_board.h"      /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../app_bits.h"
#include "../app_layout.h"

/* ---- private constants and types ------------------------------------ */

#define SB_MM_STEP_SHIFT  (2u)    /* lsb 4 mm (SB_OBS* distance fields) */
#define SB_MM_HALF_STEP   (2u)    /* round to the nearest 4 mm          */

/* One packed field of an SB_* word (tools/layout.csv). min/max/offset
   are in stored units; shift turns mm into 4 mm steps.               */
typedef struct
{
    uint32_t pos;
    uint32_t bits;
    int32_t  min;
    int32_t  max;
    int32_t  offset;
    uint32_t shift;
} sb_field_t;

/* ---- private data ---------------------------------------------------- */

/* Written only under gh_sb_mtx; every word replaced in one store.
   Initial board = natural-unit zero with "nothing seen ahead" and every
   profile field "clear": signed fields hold their offset (= 0), the
   distance and profile fields their maximum (sentinels).               */
static status_board_t g_sb =
{
    .line0 = ((uint32_t) SB_LINE0_STEERING_OFFSET << SB_LINE0_STEERING_POS),
    .mot0  = (((uint32_t) SB_MOT0_SPEED_L_OFFSET << SB_MOT0_SPEED_L_POS)
              | ((uint32_t) SB_MOT0_SPEED_R_OFFSET << SB_MOT0_SPEED_R_POS)),
    .mot1  = ((uint32_t) SB_MOT1_POS_L_OFFSET << SB_MOT1_POS_L_POS),
    .mot2  = ((uint32_t) SB_MOT2_POS_R_OFFSET << SB_MOT2_POS_R_POS),
    .mot3  = 0u,
    .imu0  = ((uint32_t) SB_IMU0_PITCH_OFFSET << SB_IMU0_PITCH_POS),
    .imu1  = ((uint32_t) SB_IMU1_TURN_RATE_OFFSET << SB_IMU1_TURN_RATE_POS),
    .obs0  = ((uint32_t) SB_OBS0_DIST_MAX << SB_OBS0_DIST_POS),
    .obs1  = (((uint32_t) SB_OBS1_WIDTH_MAX << SB_OBS1_WIDTH_POS)
              | ((uint32_t) SB_OBS1_NEAREST_MAX << SB_OBS1_NEAREST_POS)
              | ((uint32_t) SB_OBS1_CLEAR_L_MAX << SB_OBS1_CLEAR_L_POS)
              | ((uint32_t) SB_OBS1_CLEAR_R_MAX << SB_OBS1_CLEAR_R_POS)),
    .obs2  = 0u,
    .mis0  = 0u,
};

/* Field descriptors (one per SB_* field). */
static sb_field_t const g_f_steering =
{
    SB_LINE0_STEERING_POS, SB_LINE0_STEERING_BITS,
    SB_LINE0_STEERING_MIN, SB_LINE0_STEERING_MAX,
    SB_LINE0_STEERING_OFFSET, 0u
};
static sb_field_t const g_f_line_status =
{
    SB_LINE0_STATUS_POS, SB_LINE0_STATUS_BITS,
    SB_LINE0_STATUS_MIN, SB_LINE0_STATUS_MAX,
    SB_LINE0_STATUS_OFFSET, 0u
};
static sb_field_t const g_f_junction =
{
    SB_LINE0_JUNCTION_POS, SB_LINE0_JUNCTION_BITS,
    SB_LINE0_JUNCTION_MIN, SB_LINE0_JUNCTION_MAX,
    SB_LINE0_JUNCTION_OFFSET, 0u
};
static sb_field_t const g_f_calibrated =
{
    SB_LINE0_CALIBRATED_POS, SB_LINE0_CALIBRATED_BITS,
    SB_LINE0_CALIBRATED_MIN, SB_LINE0_CALIBRATED_MAX,
    SB_LINE0_CALIBRATED_OFFSET, 0u
};
static sb_field_t const g_f_speed_l =
{
    SB_MOT0_SPEED_L_POS, SB_MOT0_SPEED_L_BITS,
    SB_MOT0_SPEED_L_MIN, SB_MOT0_SPEED_L_MAX,
    SB_MOT0_SPEED_L_OFFSET, 0u
};
static sb_field_t const g_f_speed_r =
{
    SB_MOT0_SPEED_R_POS, SB_MOT0_SPEED_R_BITS,
    SB_MOT0_SPEED_R_MIN, SB_MOT0_SPEED_R_MAX,
    SB_MOT0_SPEED_R_OFFSET, 0u
};
static sb_field_t const g_f_mot_state =
{
    SB_MOT0_STATE_POS, SB_MOT0_STATE_BITS,
    SB_MOT0_STATE_MIN, SB_MOT0_STATE_MAX,
    SB_MOT0_STATE_OFFSET, 0u
};
static sb_field_t const g_f_pos_l =
{
    SB_MOT1_POS_L_POS, SB_MOT1_POS_L_BITS,
    SB_MOT1_POS_L_MIN, SB_MOT1_POS_L_MAX,
    SB_MOT1_POS_L_OFFSET, 0u
};
static sb_field_t const g_f_pos_r =
{
    SB_MOT2_POS_R_POS, SB_MOT2_POS_R_BITS,
    SB_MOT2_POS_R_MIN, SB_MOT2_POS_R_MAX,
    SB_MOT2_POS_R_OFFSET, 0u
};
static sb_field_t const g_f_path =
{
    SB_MOT3_PATH_POS, SB_MOT3_PATH_BITS,
    SB_MOT3_PATH_MIN, SB_MOT3_PATH_MAX,
    SB_MOT3_PATH_OFFSET, 0u
};
static sb_field_t const g_f_pitch =
{
    SB_IMU0_PITCH_POS, SB_IMU0_PITCH_BITS,
    SB_IMU0_PITCH_MIN, SB_IMU0_PITCH_MAX,
    SB_IMU0_PITCH_OFFSET, 0u
};
static sb_field_t const g_f_hump_now =
{
    SB_IMU0_HUMP_NOW_POS, SB_IMU0_HUMP_NOW_BITS,
    SB_IMU0_HUMP_NOW_MIN, SB_IMU0_HUMP_NOW_MAX,
    SB_IMU0_HUMP_NOW_OFFSET, 0u
};
static sb_field_t const g_f_hump_peak =
{
    SB_IMU0_HUMP_PEAK_POS, SB_IMU0_HUMP_PEAK_BITS,
    SB_IMU0_HUMP_PEAK_MIN, SB_IMU0_HUMP_PEAK_MAX,
    SB_IMU0_HUMP_PEAK_OFFSET, 0u
};
static sb_field_t const g_f_mclass =
{
    SB_IMU0_MCLASS_POS, SB_IMU0_MCLASS_BITS,
    SB_IMU0_MCLASS_MIN, SB_IMU0_MCLASS_MAX,
    SB_IMU0_MCLASS_OFFSET, 0u
};
static sb_field_t const g_f_turn_src =
{
    SB_IMU0_TURN_SRC_POS, SB_IMU0_TURN_SRC_BITS,
    SB_IMU0_TURN_SRC_MIN, SB_IMU0_TURN_SRC_MAX,
    SB_IMU0_TURN_SRC_OFFSET, 0u
};
static sb_field_t const g_f_turn_rate =
{
    SB_IMU1_TURN_RATE_POS, SB_IMU1_TURN_RATE_BITS,
    SB_IMU1_TURN_RATE_MIN, SB_IMU1_TURN_RATE_MAX,
    SB_IMU1_TURN_RATE_OFFSET, 0u
};
static sb_field_t const g_f_distance =
{
    SB_OBS0_DIST_POS, SB_OBS0_DIST_BITS,
    SB_OBS0_DIST_MIN, SB_OBS0_DIST_MAX,
    SB_OBS0_DIST_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_scan_state =
{
    SB_OBS0_SCAN_STATE_POS, SB_OBS0_SCAN_STATE_BITS,
    SB_OBS0_SCAN_STATE_MIN, SB_OBS0_SCAN_STATE_MAX,
    SB_OBS0_SCAN_STATE_OFFSET, 0u
};
static sb_field_t const g_f_angle_step =
{
    SB_OBS0_ANGLE_STEP_POS, SB_OBS0_ANGLE_STEP_BITS,
    SB_OBS0_ANGLE_STEP_MIN, SB_OBS0_ANGLE_STEP_MAX,
    SB_OBS0_ANGLE_STEP_OFFSET, 0u
};
static sb_field_t const g_f_plan_side =
{
    SB_OBS0_PLAN_SIDE_POS, SB_OBS0_PLAN_SIDE_BITS,
    SB_OBS0_PLAN_SIDE_MIN, SB_OBS0_PLAN_SIDE_MAX,
    SB_OBS0_PLAN_SIDE_OFFSET, 0u
};
static sb_field_t const g_f_prof_valid =
{
    SB_OBS0_PROFILE_VALID_POS, SB_OBS0_PROFILE_VALID_BITS,
    SB_OBS0_PROFILE_VALID_MIN, SB_OBS0_PROFILE_VALID_MAX,
    SB_OBS0_PROFILE_VALID_OFFSET, 0u
};
static sb_field_t const g_f_width =
{
    SB_OBS1_WIDTH_POS, SB_OBS1_WIDTH_BITS,
    SB_OBS1_WIDTH_MIN, SB_OBS1_WIDTH_MAX,
    SB_OBS1_WIDTH_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_nearest =
{
    SB_OBS1_NEAREST_POS, SB_OBS1_NEAREST_BITS,
    SB_OBS1_NEAREST_MIN, SB_OBS1_NEAREST_MAX,
    SB_OBS1_NEAREST_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_clear_l =
{
    SB_OBS1_CLEAR_L_POS, SB_OBS1_CLEAR_L_BITS,
    SB_OBS1_CLEAR_L_MIN, SB_OBS1_CLEAR_L_MAX,
    SB_OBS1_CLEAR_L_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_clear_r =
{
    SB_OBS1_CLEAR_R_POS, SB_OBS1_CLEAR_R_BITS,
    SB_OBS1_CLEAR_R_MIN, SB_OBS1_CLEAR_R_MAX,
    SB_OBS1_CLEAR_R_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_leg_lat =
{
    SB_OBS2_LEG_LAT_POS, SB_OBS2_LEG_LAT_BITS,
    SB_OBS2_LEG_LAT_MIN, SB_OBS2_LEG_LAT_MAX,
    SB_OBS2_LEG_LAT_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_leg_fwd =
{
    SB_OBS2_LEG_FWD_POS, SB_OBS2_LEG_FWD_BITS,
    SB_OBS2_LEG_FWD_MIN, SB_OBS2_LEG_FWD_MAX,
    SB_OBS2_LEG_FWD_OFFSET, SB_MM_STEP_SHIFT
};
static sb_field_t const g_f_sys =
{
    SB_MIS0_SYS_POS, SB_MIS0_SYS_BITS,
    SB_MIS0_SYS_MIN, SB_MIS0_SYS_MAX,
    SB_MIS0_SYS_OFFSET, 0u
};
static sb_field_t const g_f_run =
{
    SB_MIS0_RUN_POS, SB_MIS0_RUN_BITS,
    SB_MIS0_RUN_MIN, SB_MIS0_RUN_MAX,
    SB_MIS0_RUN_OFFSET, 0u
};
static sb_field_t const g_f_avoid =
{
    SB_MIS0_AVOID_POS, SB_MIS0_AVOID_BITS,
    SB_MIS0_AVOID_MIN, SB_MIS0_AVOID_MAX,
    SB_MIS0_AVOID_OFFSET, 0u
};
static sb_field_t const g_f_end =
{
    SB_MIS0_END_REASON_POS, SB_MIS0_END_REASON_BITS,
    SB_MIS0_END_REASON_MIN, SB_MIS0_END_REASON_MAX,
    SB_MIS0_END_REASON_OFFSET, 0u
};
static sb_field_t const g_f_armed =
{
    SB_MIS0_ARMED_POS, SB_MIS0_ARMED_BITS,
    SB_MIS0_ARMED_MIN, SB_MIS0_ARMED_MAX,
    SB_MIS0_ARMED_OFFSET, 0u
};
static sb_field_t const g_f_armed_cmd =
{
    SB_MIS0_ARMED_CMD_POS, SB_MIS0_ARMED_CMD_BITS,
    SB_MIS0_ARMED_CMD_MIN, SB_MIS0_ARMED_CMD_MAX,
    SB_MIS0_ARMED_CMD_OFFSET, 0u
};

static ID gh_sb_mtx = 0;

/* ---- private function prototypes ------------------------------------- */

static void     status_board_lock(void);
static void     status_board_unlock(void);
static uint32_t sb_put(uint32_t word, sb_field_t const * p_field,
                       int32_t value);
static uint32_t sb_put_mm(uint32_t word, sb_field_t const * p_field,
                          uint32_t value_mm);
static int32_t  sb_get(uint32_t word, sb_field_t const * p_field);
static uint32_t sb_get_u(uint32_t word, sb_field_t const * p_field);

/* ---- public functions ------------------------------------------------- */

/**
 * status_board_init
 * @brief   Creates MTX_STATUS with priority inheritance.
 * @return  E_OK, or the kernel error code
 * @context usermain (init phase)
 */
ER status_board_init(void)
{
    T_CMTX cmtx = {0};
    ER     ercd = E_OK;

    cmtx.mtxatr = TA_INHERIT;
    gh_sb_mtx   = tk_cre_mtx(&cmtx);

    if (gh_sb_mtx < E_OK)
    {
        ercd = (ER) gh_sb_mtx;  /* cast: negative ID is an error code */
    }

    return ercd;
}

/**
 * status_board_write_line
 * @brief   Stores the line part of the board.
 * @param   p_part new values in natural units
 * @context line_task
 */
void status_board_write_line(status_board_line_t const * p_part)
{
    uint32_t line0 = 0u;

    line0 = sb_put(line0, &g_f_steering, p_part->steering);
    /* cast: line_status is a small code (< 2^31) */
    line0 = sb_put(line0, &g_f_line_status, (int32_t) p_part->line_status);
    /* cast: junction is a small code (< 2^31) */
    line0 = sb_put(line0, &g_f_junction, (int32_t) p_part->junction);
    line0 = sb_put(line0, &g_f_calibrated, p_part->b_is_calibrated ? 1 : 0);

    status_board_lock();
    g_sb.line0 = line0;
    status_board_unlock();
}

/**
 * status_board_write_motion
 * @brief   Stores the motion part of the board.
 * @param   p_part new values in natural units
 * @context motion_task
 */
void status_board_write_motion(status_board_motion_t const * p_part)
{
    uint32_t mot0 = 0u;
    uint32_t mot1 = 0u;
    uint32_t mot2 = 0u;
    uint32_t mot3 = 0u;

    mot0 = sb_put(mot0, &g_f_speed_l, p_part->speed_l_cpw);
    mot0 = sb_put(mot0, &g_f_speed_r, p_part->speed_r_cpw);
    /* cast: motion_state is a small code (< 2^31) */
    mot0 = sb_put(mot0, &g_f_mot_state, (int32_t) p_part->motion_state);
    mot1 = sb_put(mot1, &g_f_pos_l, p_part->pos_l_counts);
    mot2 = sb_put(mot2, &g_f_pos_r, p_part->pos_r_counts);
    /* cast: path_counts is a small code (< 2^31) */
    mot3 = sb_put(mot3, &g_f_path, (int32_t) p_part->path_counts);

    status_board_lock();
    g_sb.mot0 = mot0;
    g_sb.mot1 = mot1;
    g_sb.mot2 = mot2;
    g_sb.mot3 = mot3;
    status_board_unlock();
}

/**
 * status_board_write_imu
 * @brief   Stores the IMU part of the board.
 * @param   p_part new values in natural units
 * @context imu_task
 */
void status_board_write_imu(status_board_imu_t const * p_part)
{
    uint32_t imu0 = 0u;
    uint32_t imu1 = 0u;

    imu0 = sb_put(imu0, &g_f_pitch, p_part->pitch_ddeg);
    /* cast: hump_now_mm is a small code (< 2^31) */
    imu0 = sb_put(imu0, &g_f_hump_now, (int32_t) p_part->hump_now_mm);
    /* cast: hump_peak_mm is a small code (< 2^31) */
    imu0 = sb_put(imu0, &g_f_hump_peak, (int32_t) p_part->hump_peak_mm);
    /* cast: motion_class is a small code (< 2^31) */
    imu0 = sb_put(imu0, &g_f_mclass, (int32_t) p_part->motion_class);
    imu0 = sb_put(imu0, &g_f_turn_src, p_part->b_is_turn_from_enc ? 1 : 0);
    imu1 = sb_put(imu1, &g_f_turn_rate, p_part->turn_rate_dps);

    status_board_lock();
    g_sb.imu0 = imu0;
    g_sb.imu1 = imu1;
    status_board_unlock();
}

/**
 * status_board_write_obstacle
 * @brief   Stores the obstacle part of the board.
 * @param   p_part new values in natural units
 * @context obstacle_task
 */
void status_board_write_obstacle(status_board_obs_t const * p_part)
{
    uint32_t obs0 = 0u;
    uint32_t obs1 = 0u;
    uint32_t obs2 = 0u;

    obs0 = sb_put_mm(obs0, &g_f_distance, p_part->distance_mm);
    /* cast: scanner_state is a small code (< 2^31) */
    obs0 = sb_put(obs0, &g_f_scan_state, (int32_t) p_part->scanner_state);
    /* cast: angle_step is a small code (< 2^31) */
    obs0 = sb_put(obs0, &g_f_angle_step, (int32_t) p_part->angle_step);
    /* cast: plan_side is a small code (< 2^31) */
    obs0 = sb_put(obs0, &g_f_plan_side, (int32_t) p_part->plan_side);
    obs0 = sb_put(obs0, &g_f_prof_valid, p_part->b_is_profile_valid ? 1 : 0);
    obs1 = sb_put_mm(obs1, &g_f_width, p_part->width_mm);
    obs1 = sb_put_mm(obs1, &g_f_nearest, p_part->nearest_mm);
    obs1 = sb_put_mm(obs1, &g_f_clear_l, p_part->clear_left_mm);
    obs1 = sb_put_mm(obs1, &g_f_clear_r, p_part->clear_right_mm);
    obs2 = sb_put_mm(obs2, &g_f_leg_lat, p_part->leg_lateral_mm);
    obs2 = sb_put_mm(obs2, &g_f_leg_fwd, p_part->leg_forward_mm);

    status_board_lock();
    g_sb.obs0 = obs0;
    g_sb.obs1 = obs1;
    g_sb.obs2 = obs2;
    status_board_unlock();
}

/**
 * status_board_write_mission
 * @brief   Stores the mission part of the board.
 * @param   p_part new values in natural units
 * @context mission_task
 */
void status_board_write_mission(status_board_mission_t const * p_part)
{
    uint32_t mis0 = 0u;

    /* cast: sys_state is a small code (< 2^31) */
    mis0 = sb_put(mis0, &g_f_sys, (int32_t) p_part->sys_state);
    /* cast: run_state is a small code (< 2^31) */
    mis0 = sb_put(mis0, &g_f_run, (int32_t) p_part->run_state);
    /* cast: avoid_step is a small code (< 2^31) */
    mis0 = sb_put(mis0, &g_f_avoid, (int32_t) p_part->avoid_step);
    /* cast: end_reason is a small code (< 2^31) */
    mis0 = sb_put(mis0, &g_f_end, (int32_t) p_part->end_reason);
    mis0 = sb_put(mis0, &g_f_armed, p_part->b_is_armed ? 1 : 0);
    /* cast: armed_cmd is a small code (< 2^31) */
    mis0 = sb_put(mis0, &g_f_armed_cmd, (int32_t) p_part->armed_cmd);

    status_board_lock();
    g_sb.mis0 = mis0;
    status_board_unlock();
}

/**
 * status_board_read
 * @brief   Copies the whole board into the caller's snapshot.
 * @param   p_snapshot destination; read it with the getters below
 * @context any task
 */
void status_board_read(status_board_t * p_snapshot)
{
    status_board_lock();
    *p_snapshot = g_sb;
    status_board_unlock();
}

/**
 * status_board_steering
 * @brief   Steering error, -1000..+1000 (+ = line left).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_steering(status_board_t const * p_snap)
{
    return sb_get(p_snap->line0, &g_f_steering);
}

/**
 * status_board_line_status
 * @brief   Line status, APP_LINE_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_line_status(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->line0, &g_f_line_status);
}

/**
 * status_board_junction
 * @brief   Last junction class, APP_JCT_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_junction(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->line0, &g_f_junction);
}

/**
 * status_board_is_calibrated
 * @brief   True once IR calibration is done.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
bool status_board_is_calibrated(status_board_t const * p_snap)
{
    return (0u != sb_get_u(p_snap->line0, &g_f_calibrated));
}

/**
 * status_board_speed_l
 * @brief   Left wheel speed, counts per 50 ms window.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_speed_l(status_board_t const * p_snap)
{
    return sb_get(p_snap->mot0, &g_f_speed_l);
}

/**
 * status_board_speed_r
 * @brief   Right wheel speed, counts per 50 ms window.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_speed_r(status_board_t const * p_snap)
{
    return sb_get(p_snap->mot0, &g_f_speed_r);
}

/**
 * status_board_motion_state
 * @brief   Motion executor state, APP_MOT_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_motion_state(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mot0, &g_f_mot_state);
}

/**
 * status_board_pos_l
 * @brief   Left encoder position, counts.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_pos_l(status_board_t const * p_snap)
{
    return sb_get(p_snap->mot1, &g_f_pos_l);
}

/**
 * status_board_pos_r
 * @brief   Right encoder position, counts.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_pos_r(status_board_t const * p_snap)
{
    return sb_get(p_snap->mot2, &g_f_pos_r);
}

/**
 * status_board_path
 * @brief   Path length (sum of |travel|), counts.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_path(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mot3, &g_f_path);
}

/**
 * status_board_pitch
 * @brief   Pitch, 0.1 deg (+ = nose up).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_pitch(status_board_t const * p_snap)
{
    return sb_get(p_snap->imu0, &g_f_pitch);
}

/**
 * status_board_hump_now
 * @brief   Current hump height, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_hump_now(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->imu0, &g_f_hump_now);
}

/**
 * status_board_hump_peak
 * @brief   Highest hump this run, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_hump_peak(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->imu0, &g_f_hump_peak);
}

/**
 * status_board_motion_class
 * @brief   Motion class, APP_MCLASS_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_motion_class(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->imu0, &g_f_mclass);
}

/**
 * status_board_is_turn_from_enc
 * @brief   True if turn rate came from encoders.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
bool status_board_is_turn_from_enc(status_board_t const * p_snap)
{
    return (0u != sb_get_u(p_snap->imu0, &g_f_turn_src));
}

/**
 * status_board_turn_rate
 * @brief   Turn rate, deg/s (+ = right).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
int32_t status_board_turn_rate(status_board_t const * p_snap)
{
    return sb_get(p_snap->imu1, &g_f_turn_rate);
}

/**
 * status_board_distance
 * @brief   Forward distance, mm (APP_DIST_NONE_MM = nothing).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_distance(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs0, &g_f_distance);
}

/**
 * status_board_scanner_state
 * @brief   Scanner state, APP_SCAN_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_scanner_state(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs0, &g_f_scan_state);
}

/**
 * status_board_angle_step
 * @brief   Obstacle angle step (30 + 5 x step deg).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_angle_step(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs0, &g_f_angle_step);
}

/**
 * status_board_plan_side
 * @brief   Planned bypass side, APP_SIDE_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_plan_side(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs0, &g_f_plan_side);
}

/**
 * status_board_is_profile_valid
 * @brief   True if the profile fields are valid.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
bool status_board_is_profile_valid(status_board_t const * p_snap)
{
    return (0u != sb_get_u(p_snap->obs0, &g_f_prof_valid));
}

/**
 * status_board_width
 * @brief   Obstacle width, mm (APP_PROFILE_CLEAR_MM = clear).
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_width(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs1, &g_f_width);
}

/**
 * status_board_nearest
 * @brief   Closest point of the obstacle, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_nearest(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs1, &g_f_nearest);
}

/**
 * status_board_clear_left
 * @brief   Left clearance, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_clear_left(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs1, &g_f_clear_l);
}

/**
 * status_board_clear_right
 * @brief   Right clearance, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_clear_right(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs1, &g_f_clear_r);
}

/**
 * status_board_leg_lateral
 * @brief   Planned lateral bypass leg, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_leg_lateral(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs2, &g_f_leg_lat);
}

/**
 * status_board_leg_forward
 * @brief   Planned forward bypass leg, mm.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_leg_forward(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->obs2, &g_f_leg_fwd);
}

/**
 * status_board_sys_state
 * @brief   System state, APP_SYS_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_sys_state(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mis0, &g_f_sys);
}

/**
 * status_board_run_state
 * @brief   RUN substate, APP_RUN_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_run_state(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mis0, &g_f_run);
}

/**
 * status_board_avoid_step
 * @brief   Bypass step, APP_AVOID_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_avoid_step(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mis0, &g_f_avoid);
}

/**
 * status_board_end_reason
 * @brief   Why the run ended, APP_END_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_end_reason(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mis0, &g_f_end);
}

/**
 * status_board_is_armed
 * @brief   True if a barcode command is stored.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
bool status_board_is_armed(status_board_t const * p_snap)
{
    return (0u != sb_get_u(p_snap->mis0, &g_f_armed));
}

/**
 * status_board_armed_cmd
 * @brief   Stored barcode command, APP_BAR_CMD_*.
 * @param   p_snap snapshot filled by status_board_read()
 * @return  the field value
 * @context any task
 */
uint32_t status_board_armed_cmd(status_board_t const * p_snap)
{
    return sb_get_u(p_snap->mis0, &g_f_armed_cmd);
}

/* ---- private functions ------------------------------------------------ */

/**
 * status_board_lock
 * @brief   Takes MTX_STATUS (waits forever; holders copy a few bytes).
 * @context any task
 */
static void status_board_lock(void)
{
    (void) tk_loc_mtx(gh_sb_mtx, TMO_FEVR);  /* valid ID, task context */
}

/**
 * status_board_unlock
 * @brief   Releases MTX_STATUS.
 * @context any task holding the mutex
 */
static void status_board_unlock(void)
{
    (void) tk_unl_mtx(gh_sb_mtx);  /* valid ID, task context */
}

/**
 * sb_put
 * @brief   Stores one value in its field: clamped to min..max (the
 *          writers already keep every value in range), offset added.
 * @param   word    packed word
 * @param   p_field field descriptor
 * @param   value   natural units (not mm-scaled)
 * @return  the updated word
 * @context any task (before taking the mutex)
 */
static uint32_t sb_put(uint32_t word, sb_field_t const * p_field,
                       int32_t value)
{
    uint32_t const raw = app_bits_encode(value, p_field->min, p_field->max,
                                         p_field->offset, NULL);

    return app_bits_put(word, p_field->pos, p_field->bits, raw);
}

/**
 * sb_put_mm
 * @brief   Stores a distance in 4 mm steps, rounded, saturating at the
 *          field maximum (the "nothing" / "clear" sentinel).
 * @param   word     packed word
 * @param   p_field  field descriptor (shift = SB_MM_STEP_SHIFT)
 * @param   value_mm distance, mm
 * @return  the updated word
 * @context any task (before taking the mutex)
 */
static uint32_t sb_put_mm(uint32_t word, sb_field_t const * p_field,
                          uint32_t value_mm)
{
    /* cast: max is a positive field limit (<= 511) */
    uint32_t const steps = app_bits_clamp_u(
        (value_mm + SB_MM_HALF_STEP) >> p_field->shift,
        (uint32_t) p_field->max, NULL);

    return app_bits_put(word, p_field->pos, p_field->bits, steps);
}

/**
 * sb_get
 * @brief   Reads one signed or unsigned field back in natural units.
 * @param   word    packed word
 * @param   p_field field descriptor
 * @return  the value (offset removed)
 * @context any task
 */
static int32_t sb_get(uint32_t word, sb_field_t const * p_field)
{
    return app_bits_decode(app_bits_get(word, p_field->pos, p_field->bits),
                           p_field->offset);
}

/**
 * sb_get_u
 * @brief   Reads one unsigned field (codes, flags, counts, 4 mm steps
 *          scaled back to mm).
 * @param   word    packed word
 * @param   p_field field descriptor
 * @return  the value, natural units
 * @context any task
 */
static uint32_t sb_get_u(uint32_t word, sb_field_t const * p_field)
{
    return (app_bits_get(word, p_field->pos, p_field->bits)
            << p_field->shift);
}

/*** end of file ***/
