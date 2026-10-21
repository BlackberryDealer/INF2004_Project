/**
 * @file    board_pins.h
 * @brief   The only place in the firmware that names a GPIO number.
 * @owner   Team (integrator; change only by SHARED pull request)
 * @kind    cfg
 */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <stdint.h>

#define PIN_US_TRIG        (0u)
#define PIN_US_ECHO        (1u)
#define PIN_IMU_SDA        (2u)
#define PIN_IMU_SCL        (3u)
#define PIN_ENC_L_A        (4u)   /* PIO program needs B = A + 1 */
#define PIN_ENC_L_B        (5u)
/* WARNING: GP6 shares PWM 3A with the buzzer: drive it with SIO only. */
#define PIN_LED_ALIVE      (6u)
#define PIN_BAR_DO         (7u)
#define PIN_MOTOR_L_A      (8u)
#define PIN_MOTOR_L_B      (9u)
#define PIN_MOTOR_R_A      (10u)
#define PIN_MOTOR_R_B      (11u)
#define PIN_SPARE_12       (12u)
#define PIN_SPARE_13       (13u)
#define PIN_PROBE          (14u)
#define PIN_SERVO          (15u)
#define PIN_ENC_R_A        (16u)
#define PIN_ENC_R_B        (17u)
#define PIN_RGB            (18u)
#define PIN_IMU_INT1       (19u)
#define PIN_BTN_RUN        (20u)
#define PIN_BTN_RESET      (21u)
#define PIN_BUZZER         (22u)
#define PIN_IR_LINE_L      (26u)  /* ADC0 */
#define PIN_IR_LINE_R      (27u)  /* ADC1 */
#define PIN_SPARE_28       (28u)  /* ADC2, reserved for a 2nd barcode DO */

#define BOARD_ADC_CH_LINE_L    (0u)
#define BOARD_ADC_CH_LINE_R    (1u)
#define BOARD_I2C_IMU_INST     (1u)   /* i2c1 */
#define BOARD_I2C_IMU_HZ       (400000u)
#define BOARD_PWM_HZ_MOTOR     (10000u)
#define BOARD_PWM_HZ_SERVO     (50u)
#define BOARD_IRQ_GPIO_BANK0   (13u)  /* RP2040 IO_IRQ_BANK0 */
#define BOARD_IRQ_PRI_GPIO     (2)    /* kernel interrupt level */

/* Boot check: prints any pin listed twice and the GP0/GP1 functions. */
extern void board_pins_check(void);

#endif /* BOARD_PINS_H */

/*** end of file ***/
