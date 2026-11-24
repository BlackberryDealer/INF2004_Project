/**
 * @file    drv_button.c
 * @brief   Robo Pico push-button GPIO driver (GP20 RUN, GP21 RESET).
 *          Buttons pull their pin to GND when pressed; internal pull-ups
 *          keep idle pins high. Includes only Pico SDK headers, never
 *          app.h (docs/BARR_C_DEVIATIONS.md build rule).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR11 (buttons)
 */

#include "drv_button.h"        /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "hardware/gpio.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_button_init
 * @brief   Configures GP20 (RUN) and GP21 (RESET) as inputs with
 *          internal pull-ups.
 * @return  none
 * @context mission_task (before its loop)
 */
void drv_button_init(void)
{
    gpio_init(PIN_BTN_RUN);
    gpio_set_dir(PIN_BTN_RUN, false);
    gpio_pull_up(PIN_BTN_RUN);
    gpio_init(PIN_BTN_RESET);
    gpio_set_dir(PIN_BTN_RESET, false);
    gpio_pull_up(PIN_BTN_RESET);
}

/**
 * drv_button_run_pressed
 * @brief   Reads the active-low RUN push-button (GP20).
 * @return  true while pressed
 * @context mission_task
 */
bool drv_button_run_pressed(void)
{
    return !gpio_get(PIN_BTN_RUN);
}

/**
 * drv_button_reset_pressed
 * @brief   Reads the active-low RESET push-button (GP21).
 * @return  true while pressed
 * @context mission_task
 */
bool drv_button_reset_pressed(void)
{
    return !gpio_get(PIN_BTN_RESET);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
