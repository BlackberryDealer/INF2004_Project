/**
 * @file    drv_motor.c
 * @brief   Motor PWM driver. Robo Pico truth table (Part B.1): A=PWM, B=0
 *          forward; A=0, B=PWM backward; A=0, B=0 brake. High/high
 *          (coast) is never produced.
 * @owner   Buddy 2 (motion)
 * @kind    drv
 * @reqs    FR15, NFR10c (motor inputs low from power-on)
 *
 * Pins: GP8/GP9 left (slice 4 A/B), GP10/GP11 right (slice 5 A/B).
 * Owner: motion_task only. Includes only Pico SDK headers, never app.h.
 */

#include "drv_motor.h"         /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "cfg_motion.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_MOTOR_PIN_COUNT    (4u)
#define DRV_MOTOR_CLKDIV_INT   (1u)   /* 125 MHz / 1 / 12500 = 10 kHz */
#define DRV_MOTOR_CLKDIV_FRAC  (0u)
#define DRV_MOTOR_LEVEL_NUM    (25u)  /* level = permille x 25 / 2    */
#define DRV_MOTOR_LEVEL_SHIFT  (1u)   /* = permille x 12.5, no divide */

/* ---- private data ---------------------------------------------------- */

static uint8_t const g_motor_pins[DRV_MOTOR_PIN_COUNT] =
{
    PIN_MOTOR_L_A, PIN_MOTOR_L_B, PIN_MOTOR_R_A, PIN_MOTOR_R_B
};

/* ---- private function prototypes ------------------------------------- */

static void drv_motor_drive(uint32_t pin_a, uint32_t pin_b,
                            int32_t duty_permille, bool b_is_reversed);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_motor_init
 * @brief   Sets every motor level to 0 BEFORE handing the pins to PWM, so
 *          no input ever goes high during start-up, then runs both slices
 *          at 10 kHz with every channel at 0 (brake).
 * @return  none
 * @context motion_task (start of task)
 */
void drv_motor_init(void)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < DRV_MOTOR_PIN_COUNT; idx++)
    {
        pwm_set_gpio_level(g_motor_pins[idx], 0u);
        gpio_set_function(g_motor_pins[idx], GPIO_FUNC_PWM);
    }

    for (idx = 0u; idx < DRV_MOTOR_PIN_COUNT; idx += 2u)
    {
        /* A and B of one motor share a slice: configure each slice once */
        uint32_t const slice = pwm_gpio_to_slice_num(g_motor_pins[idx]);

        pwm_set_wrap(slice, (uint16_t) MOTION_PWM_WRAP); /* cast: 12499 */
        pwm_set_clkdiv_int_frac(slice, (uint8_t) DRV_MOTOR_CLKDIV_INT,
                                (uint8_t) DRV_MOTOR_CLKDIV_FRAC); /* <16 */
        pwm_set_chan_level(slice, PWM_CHAN_A, 0u);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0u);
        pwm_set_enabled(slice, true);
    }
}

/**
 * drv_motor_set
 * @brief   Drives both wheels at the given duty (clamped to +-1000).
 * @param   duty_l_permille  left wheel, + = forward, 0 = brake
 * @param   duty_r_permille  right wheel, + = forward, 0 = brake
 * @return  none
 * @context motion_task
 */
void drv_motor_set(int32_t duty_l_permille, int32_t duty_r_permille)
{
    drv_motor_drive(PIN_MOTOR_L_A, PIN_MOTOR_L_B, duty_l_permille,
                    MOTION_L_REVERSED);
    drv_motor_drive(PIN_MOTOR_R_A, PIN_MOTOR_R_B, duty_r_permille,
                    MOTION_R_REVERSED);
}

/**
 * drv_motor_brake
 * @brief   All four motor inputs low (brake).
 * @return  none
 * @context motion_task
 */
void drv_motor_brake(void)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < DRV_MOTOR_PIN_COUNT; idx++)
    {
        pwm_set_gpio_level(g_motor_pins[idx], 0u);
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * drv_motor_drive
 * @brief   One motor: forward = A at level, B low; backward = A low, B at
 *          level; zero = both low (brake). Never both high.
 * @param   pin_a          the motor's A input
 * @param   pin_b          the motor's B input
 * @param   duty_permille  -1000..+1000 (clamped), + = forward
 * @param   b_is_reversed  true if this motor is wired the other way (A5)
 * @return  none
 * @context motion_task
 */
static void drv_motor_drive(uint32_t pin_a, uint32_t pin_b,
                            int32_t duty_permille, bool b_is_reversed)
{
    int32_t  duty      = duty_permille;
    uint32_t magnitude = 0u;
    uint16_t level     = 0u;

    if (duty > MOTION_DUTY_MAX_PERMILLE)
    {
        duty = MOTION_DUTY_MAX_PERMILLE;
    }
    else if (duty < -MOTION_DUTY_MAX_PERMILLE)
    {
        duty = -MOTION_DUTY_MAX_PERMILLE;
    }
    else
    {
        /* already in range */
    }

    if (b_is_reversed)
    {
        duty = -duty;
    }

    /* cast: |duty| <= 1000 after the clamp, so it fits uint32_t */
    magnitude = (duty < 0) ? (uint32_t) -duty : (uint32_t) duty;

    /* magnitude x 12.5 without a division; cast: <= 12500 fits uint16 */
    level = (uint16_t) ((magnitude * DRV_MOTOR_LEVEL_NUM)
                        >> DRV_MOTOR_LEVEL_SHIFT);

    if (duty > 0)
    {
        pwm_set_gpio_level(pin_b, 0u);      /* B low first: never both */
        pwm_set_gpio_level(pin_a, level);
    }
    else if (duty < 0)
    {
        pwm_set_gpio_level(pin_a, 0u);      /* A low first: never both */
        pwm_set_gpio_level(pin_b, level);
    }
    else
    {
        pwm_set_gpio_level(pin_a, 0u);      /* brake */
        pwm_set_gpio_level(pin_b, 0u);
    }
}

/*** end of file ***/
