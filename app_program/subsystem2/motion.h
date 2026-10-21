/**
 * @file    motion.h
 * @brief   The motion library. Only mission_task calls the commands.
 * @owner   Buddy 2 (motion)
 * @kind    task
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   motion_init(void);   /* creates K5 */
extern ER   motion_start(void);  /* starts motion_task */

/* Each returns false if the argument is out of range (NFR16 caps) or K5
   is full. Commands other than STOP are accepted only from BRAKE; while
   moving, the newest one waits in a one-slot latch. STOP always acts.
   Motion drives only while APP_MODE_MOTOR_EN is set (FR17).            */
extern bool motion_move_forward(int32_t distance_mm);   /* 1..2047 mm   */
extern bool motion_move_backward(int32_t distance_mm);  /* 1..150 mm    */
extern bool motion_turn_left(int32_t angle_deg);        /* 1..360 deg   */
extern bool motion_turn_right(int32_t angle_deg);       /* 1..360 deg   */
extern bool motion_follow(int32_t speed_mm_s);          /* 0..300 mm/s  */
extern bool motion_stop(void);

/* Rebase positions and path to 0 at the start of a run. Acted on only in
   BRAKE; motion acknowledges with MOVE_DONE carrying this command's tag. */
extern bool     motion_reset_odometry(void);

/* Tag (0..3) of the last command accepted by the functions above. Every
   MOVE_DONE carries the tag of the command that finished; mission ignores
   a MOVE_DONE whose tag is not the one it is waiting for.               */
extern uint32_t motion_last_tag(void);

/* Internal: motion_task.c only. Takes the next K5 command (TMO_POL).
   Returns false when K5 is empty.                                       */
extern bool     motion_cmd_receive(uint32_t * p_type, uint32_t * p_param,
                                   uint32_t * p_tag);

#endif /* MOTION_H */

/*** end of file ***/
