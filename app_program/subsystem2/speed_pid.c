/**
 * @file    speed_pid.c
 * @brief   Wheel-speed PI controller: duty = (Kff x sp + Kp x err + I) / 256,
 *          clamped to +-1000 permille, then rate-limited (ramp).
 * @owner   Buddy 2 (motion)
 * @kind    algo
 * @reqs    NFR2 (speed loop every 10 ms, settles to +-5 % in 0.5 s)
 *
 * Gains are Q8 (value x 256) in cfg_motion.h. Runs in the 10 ms path, so
 * the only division is by 256, a power of two (Part B.7).
 * Worst-case magnitudes: |sp|, |speed| <= 315 cpw (5 slots x 63), so
 * |sp x Kff| <= 1.2e6, |err x Kp| <= 1.7e6, |I| <= 76 800: far below 2^31.
 */

#include "speed_pid.h"         /* own header first (BARR-C 4.3.c) */
#include <stddef.h>            /* NULL */
#include "cfg_motion.h"

/* ---- private constants and types ------------------------------------ */

#define SPEED_PID_Q8_ONE   (256)   /* Q8 scale */

/* ---- private function prototypes ------------------------------------- */

static int32_t speed_pid_clamp(int32_t value, int32_t limit);

/* ---- public functions ------------------------------------------------- */

/**
 * speed_pid_reset
 * @brief   Integrator to 0.
 * @param   p_pid  controller state
 * @return  none
 * @context motion_task
 */
void speed_pid_reset(speed_pid_t * p_pid)
{
    if (NULL != p_pid)
    {
        p_pid->integ = 0;
    }
}

/**
 * speed_pid_step
 * @brief   One control step. With set-point 0 the integrator is reset and
 *          the duty ramps toward 0.
 * @param   p_pid               controller state
 * @param   setpoint_cpw        wanted speed, counts per 50 ms (+ = forward)
 * @param   speed_cpw           measured speed, counts per 50 ms
 * @param   duty_prev_permille  duty applied last tick
 * @return  new duty, -1000..+1000 permille
 * @context motion_task (10 ms path)
 */
int32_t speed_pid_step(speed_pid_t * p_pid, int32_t setpoint_cpw,
                       int32_t speed_cpw, int32_t duty_prev_permille)
{
    int32_t const prev   = speed_pid_clamp(duty_prev_permille,
                                           MOTION_DUTY_MAX_PERMILLE);
    int32_t       target = 0;
    int32_t       err    = 0;
    int32_t       raw    = 0;

    if ((NULL != p_pid) && (0 != setpoint_cpw))
    {
        err          = setpoint_cpw - speed_cpw;
        p_pid->integ = speed_pid_clamp(p_pid->integ + (err * MOTION_KI_Q8),
                                       MOTION_I_CLAMP);

        /* Signed division by a power of two: the compiler emits a shift
           with a sign fix-up (truncates toward zero), no library call.  */
        raw = ((setpoint_cpw * MOTION_KFF_Q8) + (err * MOTION_KP_Q8)
               + p_pid->integ) / SPEED_PID_Q8_ONE;
        target = speed_pid_clamp(raw, MOTION_DUTY_MAX_PERMILLE);
    }
    else
    {
        speed_pid_reset(p_pid);   /* stop: forget the integral, ramp to 0 */
    }

    /* Ramp: the duty moves at most MOTION_RAMP_PERMILLE_PER_TICK per tick */
    return (prev + speed_pid_clamp(target - prev,
                                   MOTION_RAMP_PERMILLE_PER_TICK));
}

/* ---- private functions ------------------------------------------------ */

/**
 * speed_pid_clamp
 * @brief   Clamps a signed value to [-limit, +limit].
 * @param   value  value to clamp
 * @param   limit  positive limit
 * @return  clamped value
 * @context any
 */
static int32_t speed_pid_clamp(int32_t value, int32_t limit)
{
    int32_t result = value;

    if (value > limit)
    {
        result = limit;
    }
    else if (value < -limit)
    {
        result = -limit;
    }
    else
    {
        /* in range */
    }

    return result;
}

/*** end of file ***/
