/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04P trigger output on GP0 (echo is handled through R1).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

extern void drv_ultrasonic_init(void);     /* TRIG output, low */
extern void drv_ultrasonic_trigger(void);  /* 10 us high pulse */

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
