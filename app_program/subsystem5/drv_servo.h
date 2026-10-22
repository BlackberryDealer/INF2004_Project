/**
 * @file    drv_servo.h
 * @brief   SG90 servo on GP15 (PWM slice 7 B, 50 Hz).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#define DRV_SERVO_DEG_MAX   (180u)

extern void drv_servo_init(void);               /* 50 Hz, pointing 90 deg */
extern void drv_servo_set_deg(uint32_t angle_deg);  /* 0 right .. 180 left */

#endif /* DRV_SERVO_H */

/*** end of file ***/
