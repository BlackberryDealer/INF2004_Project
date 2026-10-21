/**
 * @file    drv_servo.c
 * @brief   SG90 servo: 1 us PWM ticks, 20 ms frame, pulse 500..2500 us.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR7, A9
 */

#include "drv_servo.h"         /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "cfg_obstacle.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_SERVO_CLKDIV      (125u)    /* 125 MHz / 125 = 1 MHz: 1 us  */
#define DRV_SERVO_WRAP        (19999u)  /* 20000 ticks = 20 ms (50 Hz)  */
#define DRV_SERVO_CENTRE_DEG  (90u)     /* straight ahead               */

/* ---- private data ---------------------------------------------------- */

static uint32_t g_servo_slice   = 0u;
static uint32_t g_servo_channel = 0u;

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_servo_init
 * @brief   Puts GP15 on PWM slice 7 B at 50 Hz and points ahead (90 deg).
 * @context usermain (obstacle_init)
 */
void drv_servo_init(void)
{
    pwm_config cfg = pwm_get_default_config();

    g_servo_slice   = pwm_gpio_to_slice_num(PIN_SERVO);
    g_servo_channel = pwm_gpio_to_channel(PIN_SERVO);
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv_int(&cfg, DRV_SERVO_CLKDIV);
    pwm_config_set_wrap(&cfg, (uint16_t) DRV_SERVO_WRAP); /* cast: 19999 */
    pwm_init(g_servo_slice, &cfg, false);
    drv_servo_set_deg(DRV_SERVO_CENTRE_DEG);
    pwm_set_enabled(g_servo_slice, true);
}

/**
 * drv_servo_set_deg
 * @brief   Commands an angle (event time, so the division is allowed).
 * @param   angle_deg 0 = robot's right, 90 = ahead, 180 = left;
 *                    values above 180 are treated as 180
 * @context obstacle_task (only while the robot is stopped, or once
 *          before a bypass leg)
 */
void drv_servo_set_deg(uint32_t angle_deg)
{
    uint32_t const deg   = (angle_deg > DRV_SERVO_DEG_MAX)
                           ? DRV_SERVO_DEG_MAX : angle_deg;
    int32_t        pulse = 0;

    /* casts: all operands <= 2500, well inside int32_t */
    pulse = (int32_t) OBS_SERVO_MIN_US
            + (int32_t) ((deg * (OBS_SERVO_MAX_US - OBS_SERVO_MIN_US))
                         / DRV_SERVO_DEG_MAX)
            + OBS_SERVO_TRIM_US;
    if (pulse < (int32_t) OBS_SERVO_MIN_US)    /* cast: constant 500 */
    {
        pulse = (int32_t) OBS_SERVO_MIN_US;    /* cast: constant 500 */
    }
    else if (pulse > (int32_t) OBS_SERVO_MAX_US) /* cast: constant 2500 */
    {
        pulse = (int32_t) OBS_SERVO_MAX_US;    /* cast: constant 2500 */
    }
    else
    {
        /* in range */
    }
    /* cast: 500..2500 after the clamp */
    pwm_set_chan_level(g_servo_slice, g_servo_channel, (uint16_t) pulse);
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
