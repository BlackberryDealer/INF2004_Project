/**
 * @file    drv_motor.h
 * @brief   Two DC motors through the Robo Pico driver: PWM slices 4 and 5,
 *          10 kHz, brake when idle. Called only by motion_task.
 * @owner   Buddy 2 (motion)
 * @kind    drv
 *
 * Kernel-free on purpose: drv_motor.c uses the Pico SDK, whose headers
 * cannot share a file with the kernel headers (docs/BARR_C_DEVIATIONS.md).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* Both motors braked (all four inputs low), then PWM started. */
extern void drv_motor_init(void);

/* Drive both wheels. Duty in permille, -1000..+1000 (clamped);
   + = wheel drives the robot forward, 0 = brake.                */
extern void drv_motor_set(int32_t duty_l_permille, int32_t duty_r_permille);

/* All four inputs low (brake). */
extern void drv_motor_brake(void);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
