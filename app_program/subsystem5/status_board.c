/**
 * @file    status_board.c
 * @brief   Status board K6 and MTX_STATUS (baseline: natural types).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    min-bit item 6 (shared data), NFR11
 */

#include "status_board.h"      /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* Written only under gh_sb_mtx. Initial board: nothing seen ahead and
   every profile field "clear"; everything else zero.                   */
static status_board_t g_sb =
{
    .obs =
    {
        .distance_mm    = APP_DIST_NONE_MM,
        .width_mm       = APP_PROFILE_CLEAR_MM,
        .nearest_mm     = APP_PROFILE_CLEAR_MM,
        .clear_left_mm  = APP_PROFILE_CLEAR_MM,
        .clear_right_mm = APP_PROFILE_CLEAR_MM,
    },
};

static ID gh_sb_mtx = 0;

/* ---- private function prototypes ------------------------------------- */

static void status_board_lock(void);
static void status_board_unlock(void);

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
    status_board_lock();
    g_sb.line = *p_part;
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
    status_board_lock();
    g_sb.motion = *p_part;
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
    status_board_lock();
    g_sb.imu = *p_part;
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
    status_board_lock();
    g_sb.obs = *p_part;
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
    status_board_lock();
    g_sb.mission = *p_part;
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
    return p_snap->line.steering;
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
    return p_snap->line.line_status;
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
    return p_snap->line.junction;
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
    return p_snap->line.b_is_calibrated;
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
    return p_snap->motion.speed_l_cpw;
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
    return p_snap->motion.speed_r_cpw;
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
    return p_snap->motion.motion_state;
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
    return p_snap->motion.pos_l_counts;
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
    return p_snap->motion.pos_r_counts;
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
    return p_snap->motion.path_counts;
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
    return p_snap->imu.pitch_ddeg;
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
    return p_snap->imu.hump_now_mm;
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
    return p_snap->imu.hump_peak_mm;
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
    return p_snap->imu.motion_class;
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
    return p_snap->imu.b_is_turn_from_enc;
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
    return p_snap->imu.turn_rate_dps;
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
    return p_snap->obs.distance_mm;
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
    return p_snap->obs.scanner_state;
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
    return p_snap->obs.angle_step;
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
    return p_snap->obs.plan_side;
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
    return p_snap->obs.b_is_profile_valid;
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
    return p_snap->obs.width_mm;
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
    return p_snap->obs.nearest_mm;
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
    return p_snap->obs.clear_left_mm;
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
    return p_snap->obs.clear_right_mm;
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
    return p_snap->obs.leg_lateral_mm;
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
    return p_snap->obs.leg_forward_mm;
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
    return p_snap->mission.sys_state;
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
    return p_snap->mission.run_state;
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
    return p_snap->mission.avoid_step;
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
    return p_snap->mission.end_reason;
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
    return p_snap->mission.b_is_armed;
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
    return p_snap->mission.armed_cmd;
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

/*** end of file ***/
