/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04P trigger: one 10 us pulse on GP0 per ping.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR6, A8
 */

#include "drv_ultrasonic.h"    /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "hardware/gpio.h"     /* before app.h (sys_tick.h): see Q1 */
#include "sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_US_TRIG_PULSE_US   (10u)   /* HC-SR04P datasheet minimum */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_ultrasonic_init
 * @brief   Configures GP0 as an output, driven low.
 * @context usermain (obstacle_init)
 */
void drv_ultrasonic_init(void)
{
    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, true);
    gpio_put(PIN_US_TRIG, false);
}

/**
 * drv_ultrasonic_trigger
 * @brief   Drives TRIG high for 10 us (busy wait, within the 20 us rule).
 * @context obstacle_task
 */
void drv_ultrasonic_trigger(void)
{
    uint32_t const start_us = sys_tick_now_us();

    gpio_put(PIN_US_TRIG, true);
    while ((sys_tick_now_us() - start_us) < DRV_US_TRIG_PULSE_US)
    {
        /* busy: 10 us, allowed (<= 20 us) */
    }
    gpio_put(PIN_US_TRIG, false);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
