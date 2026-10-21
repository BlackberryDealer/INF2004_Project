/**
 * @file    drv_buzzer.h
 * @brief   Buzzer on GP22 (PWM slice 3 A, 2 kHz square wave).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_BUZZER_H
#define DRV_BUZZER_H

#include <stdint.h>
#include <stdbool.h>

extern void drv_buzzer_init(void);   /* PWM running, silent */
extern void drv_buzzer_on(void);     /* 2 kHz, 50 % duty    */
extern void drv_buzzer_off(void);

#endif /* DRV_BUZZER_H */

/*** end of file ***/
