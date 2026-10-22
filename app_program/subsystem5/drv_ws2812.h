/**
 * @file    drv_ws2812.h
 * @brief   On-board WS2812 RGB LED on GP18 through a PIO state machine.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_WS2812_H
#define DRV_WS2812_H

#include <stdint.h>
#include <stdbool.h>

/* Loads the 4-instruction program into PIO1 or PIO0 (never resets a
   block). false if no space: the caller raises APP_FAULT_PIO_LOAD and the
   robot runs without RGB.                                               */
extern bool drv_ws2812_init(void);
extern void drv_ws2812_set(uint32_t red, uint32_t green, uint32_t blue);

#endif /* DRV_WS2812_H */

/*** end of file ***/
