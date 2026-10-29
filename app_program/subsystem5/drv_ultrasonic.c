/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04P trigger: one 10 us pulse on GP0 per ping.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR6, A8
 */

#include "drv_ultrasonic.h"    /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "hardware/gpio.h"     /* SDK side only: no kernel headers */
#include "hardware/timer.h"    /* time_us_32(): inline timer read */

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
 *          The start time is read after TRIG goes high: an interrupt or
 *          pre-emption before that read can then only lengthen the
 *          pulse, never shorten it below 10 us (test S5-U09). Unsigned
 *          subtraction keeps the wait correct across the timer wrap.
 * @context obstacle_task
 */
void drv_ultrasonic_trigger(void)
{
    uint32_t start_us = 0u;

    gpio_put(PIN_US_TRIG, true);
    start_us = time_us_32();
    while ((time_us_32() - start_us) <= DRV_US_TRIG_PULSE_US)
    {
        /* busy: 10..11 us, allowed (<= 20 us) */
    }
    gpio_put(PIN_US_TRIG, false);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
