/**
 * @file    speed_pid.h
 * @brief   Fixed-point PI wheel-speed controller with feed-forward and a
 *          duty ramp. Pure C, host-safe. One instance per wheel.
 * @owner   Buddy 2 (motion)
 * @kind    algo
 */

#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t integ;        /* E1: 1/256 permille, clamped +-MOTION_I_CLAMP */
} speed_pid_t;

/* Integrator to 0. */
extern void    speed_pid_reset(speed_pid_t * p_pid);

/* One 10 ms step. Set-point and measured speed in counts per 50 ms window;
   returns the new duty in permille (-1000..+1000), never more than
   MOTION_RAMP_PERMILLE_PER_TICK away from duty_prev_permille.            */
extern int32_t speed_pid_step(speed_pid_t * p_pid, int32_t setpoint_cpw,
                              int32_t speed_cpw, int32_t duty_prev_permille);

#endif /* SPEED_PID_H */

/*** end of file ***/
