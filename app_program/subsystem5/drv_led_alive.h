/**
 * @file    drv_led_alive.h
 * @brief   Alive LED on GP6 (Grove 5), SIO output only.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_LED_ALIVE_H
#define DRV_LED_ALIVE_H

#include <stdint.h>
#include <stdbool.h>

extern void drv_led_alive_init(void);     /* GP6 SIO output, LED off */
extern void drv_led_alive_toggle(void);   /* mission_task only        */

#endif /* DRV_LED_ALIVE_H */

/*** end of file ***/
