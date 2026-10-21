/**
 * @file    status_board.c
 * @brief   Phase 0 STUB of the status board (K6): writes are dropped, getters
 *          return 0.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 * @reqs    Minimum-bit items 3, 6 (Phase 2 baseline, Phase 4 packed)
 */

#include "status_board.h"      /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * status_board_init
 * @brief   STUB: will create MTX_STATUS.
 * @return  E_OK
 * @context usermain
 */
ER status_board_init(void)
{
    return E_OK;
}

/**
 * status_board_write_line
 * @brief   STUB: replace the line part of the board.
 * @param   p_part  new values, natural units
 * @return  none
 * @context line_task
 */
void status_board_write_line(status_board_line_t const * p_part)
{
    (void) p_part;   /* stub: unused until the owner's version */
}

/**
 * status_board_write_motion
 * @brief   STUB: replace the motion part of the board.
 * @param   p_part  new values, natural units
 * @return  none
 * @context motion_task
 */
void status_board_write_motion(status_board_motion_t const * p_part)
{
    (void) p_part;   /* stub: unused until the owner's version */
}

/**
 * status_board_write_imu
 * @brief   STUB: replace the imu part of the board.
 * @param   p_part  new values, natural units
 * @return  none
 * @context imu_task
 */
void status_board_write_imu(status_board_imu_t const * p_part)
{
    (void) p_part;   /* stub: unused until the owner's version */
}

/**
 * status_board_write_obstacle
 * @brief   STUB: replace the obstacle part of the board.
 * @param   p_part  new values, natural units
 * @return  none
 * @context obstacle_task
 */
void status_board_write_obstacle(status_board_obs_t const * p_part)
{
    (void) p_part;   /* stub: unused until the owner's version */
}

/**
 * status_board_write_mission
 * @brief   STUB: replace the mission part of the board.
 * @param   p_part  new values, natural units
 * @return  none
 * @context mission_task
 */
void status_board_write_mission(status_board_mission_t const * p_part)
{
    (void) p_part;   /* stub: unused until the owner's version */
}

/**
 * status_board_read
 * @brief   STUB: copy the whole board under the mutex. The stub leaves the
 *          snapshot unchanged.
 * @param   p_snapshot  receives a copy
 * @return  none
 * @context any task
 */
void status_board_read(status_board_t * p_snapshot)
{
    (void) p_snapshot;   /* stub: unused until the owner's version */
}

/**
 * status_board_steering
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_steering(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_line_status
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_line_status(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_junction
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_junction(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_is_calibrated
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
bool status_board_is_calibrated(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return false;
}

/**
 * status_board_speed_l
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_speed_l(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_speed_r
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_speed_r(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_motion_state
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_motion_state(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_pos_l
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_pos_l(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_pos_r
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_pos_r(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_path
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_path(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_pitch
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_pitch(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_hump_now
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_hump_now(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_hump_peak
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_hump_peak(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_motion_class
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_motion_class(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_is_turn_from_enc
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
bool status_board_is_turn_from_enc(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return false;
}

/**
 * status_board_turn_rate
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
int32_t status_board_turn_rate(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * status_board_distance
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_distance(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_scanner_state
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_scanner_state(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_angle_step
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_angle_step(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_plan_side
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_plan_side(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_is_profile_valid
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
bool status_board_is_profile_valid(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return false;
}

/**
 * status_board_width
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_width(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_nearest
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_nearest(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_clear_left
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_clear_left(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_clear_right
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_clear_right(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_leg_lateral
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_leg_lateral(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_leg_forward
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_leg_forward(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_sys_state
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_sys_state(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_run_state
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_run_state(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_avoid_step
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_avoid_step(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_end_reason
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_end_reason(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * status_board_is_armed
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
bool status_board_is_armed(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return false;
}

/**
 * status_board_armed_cmd
 * @brief   STUB getter.
 * @param   p_snap  snapshot from status_board_read
 * @return  0 / false (stub)
 * @context any task
 */
uint32_t status_board_armed_cmd(status_board_t const * p_snap)
{
    (void) p_snap;   /* stub: unused until the owner's version */

    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
