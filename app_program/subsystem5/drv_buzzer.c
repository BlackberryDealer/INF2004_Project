/**
 * @file    drv_buzzer.c
 * @brief   Buzzer: 2 kHz, 50 % duty on PWM 3A. GP6 (alive LED) is the
 *          other pin of that channel and stays in SIO mode.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR16
 */

#include "drv_buzzer.h"        /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_BUZZER_CLKDIV   (125u)   /* 1 MHz PWM clock         */
#define DRV_BUZZER_WRAP     (499u)   /* 500 ticks = 2 kHz       */
#define DRV_BUZZER_LEVEL    (250u)   /* 50 % duty               */

/* ---- private data ---------------------------------------------------- */

static uint32_t g_buzzer_slice   = 0u;
static uint32_t g_buzzer_channel = 0u;

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_buzzer_init
 * @brief   Puts GP22 on PWM 3A at 2 kHz with duty 0 (silent).
 * @context mission_task (start of the task)
 */
void drv_buzzer_init(void)
{
    pwm_config cfg = pwm_get_default_config();

    g_buzzer_slice   = pwm_gpio_to_slice_num(PIN_BUZZER);
    g_buzzer_channel = pwm_gpio_to_channel(PIN_BUZZER);
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv_int(&cfg, DRV_BUZZER_CLKDIV);
    pwm_config_set_wrap(&cfg, (uint16_t) DRV_BUZZER_WRAP); /* cast: 499 */
    pwm_init(g_buzzer_slice, &cfg, false);
    pwm_set_chan_level(g_buzzer_slice, g_buzzer_channel, 0u);
    pwm_set_enabled(g_buzzer_slice, true);
}

/**
 * drv_buzzer_on
 * @brief   Starts the 2 kHz tone.
 * @context mission_task
 */
void drv_buzzer_on(void)
{
    pwm_set_chan_level(g_buzzer_slice, g_buzzer_channel,
                       (uint16_t) DRV_BUZZER_LEVEL); /* cast: 250 */
}

/**
 * drv_buzzer_off
 * @brief   Silences the buzzer (duty 0).
 * @context mission_task
 */
void drv_buzzer_off(void)
{
    pwm_set_chan_level(g_buzzer_slice, g_buzzer_channel, 0u);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
