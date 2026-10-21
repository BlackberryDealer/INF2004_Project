/**
 * @file    motion.c
 * @brief   Phase 0 STUB of the motion library: motion_start() starts
 *          motion_task; every command is refused.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR15, FR17
 */

#include "motion.h"            /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "motion_task.h"

/* ---- private constants and types ------------------------------------ */

#define MOTION_STACK_WORDS   (APP_STACK_MOTION_BYTES / 8u)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_motion_stack[MOTION_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * motion_init
 * @brief   STUB: will create K5 MBF_MOTION_CMD.
 * @return  E_OK
 * @context usermain
 */
ER motion_init(void)
{
    return E_OK;
}

/**
 * motion_start
 * @brief   Creates and starts motion_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER motion_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) motion_task, /* cast: kernel entry type */
                             APP_PRI_MOTION, g_motion_stack,
                             APP_STACK_MOTION_BYTES, "motion_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * motion_move_forward
 * @brief   STUB: drive forward a distance.
 * @param   distance_mm  1..2047 mm
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_move_forward(int32_t distance_mm)
{
    (void) distance_mm;   /* stub: unused until the owner's version */

    return false;
}

/**
 * motion_move_backward
 * @brief   STUB: reverse a distance.
 * @param   distance_mm  1..150 mm
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_move_backward(int32_t distance_mm)
{
    (void) distance_mm;   /* stub: unused until the owner's version */

    return false;
}

/**
 * motion_turn_left
 * @brief   STUB: turn left on the spot.
 * @param   angle_deg  1..360 deg
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_turn_left(int32_t angle_deg)
{
    (void) angle_deg;   /* stub: unused until the owner's version */

    return false;
}

/**
 * motion_turn_right
 * @brief   STUB: turn right on the spot.
 * @param   angle_deg  1..360 deg
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_turn_right(int32_t angle_deg)
{
    (void) angle_deg;   /* stub: unused until the owner's version */

    return false;
}

/**
 * motion_follow
 * @brief   STUB: follow the line at a speed.
 * @param   speed_mm_s  0..300 mm/s
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_follow(int32_t speed_mm_s)
{
    (void) speed_mm_s;   /* stub: unused until the owner's version */

    return false;
}

/**
 * motion_stop
 * @brief   STUB: brake now.
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_stop(void)
{
    return false;
}

/**
 * motion_reset_odometry
 * @brief   STUB: rebase positions and path to 0 (APP_CMD_ZERO).
 * @return  false (stub: refused)
 * @context mission_task
 */
bool motion_reset_odometry(void)
{
    return false;
}

/**
 * motion_last_tag
 * @brief   STUB: tag of the last accepted command.
 * @return  0 (stub)
 * @context mission_task
 */
uint32_t motion_last_tag(void)
{
    return 0u;
}

/**
 * motion_cmd_receive
 * @brief   STUB: take the next K5 command (TMO_POL).
 * @param   p_type   receives APP_CMD_*
 * @param   p_param  receives mm, deg or mm/s
 * @param   p_tag    receives the command tag
 * @return  false (stub: K5 always empty)
 * @context motion_task
 */
bool motion_cmd_receive(uint32_t * p_type, uint32_t * p_param, uint32_t * p_tag)
{
    (void) p_type;   /* stub: unused until the owner's version */
    (void) p_param;
    (void) p_tag;

    return false;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
