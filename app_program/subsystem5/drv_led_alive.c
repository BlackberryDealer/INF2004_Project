/**
 * @file    drv_led_alive.c
 * @brief   Alive LED on GP6. GP6 is on PWM channel 3A, the same channel
 *          as the buzzer (GP22), so it stays SIO and is never PWM.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR16
 */

#include "drv_led_alive.h"     /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "hardware/gpio.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

static bool gb_is_led_on = false;   /* written only by mission_task */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_led_alive_init
 * @brief   Configures GP6 as an SIO output, LED off.
 * @context usermain (init phase)
 */
void drv_led_alive_init(void)
{
    gpio_init(PIN_LED_ALIVE);              /* selects SIO, never PWM */
    gpio_set_dir(PIN_LED_ALIVE, true);
    gpio_put(PIN_LED_ALIVE, false);
    gb_is_led_on = false;
}

/**
 * drv_led_alive_toggle
 * @brief   Inverts the alive LED.
 * @context mission_task
 */
void drv_led_alive_toggle(void)
{
    gb_is_led_on = !gb_is_led_on;
    gpio_put(PIN_LED_ALIVE, gb_is_led_on);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
