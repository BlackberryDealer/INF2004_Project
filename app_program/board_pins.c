/**
 * @file    board_pins.c
 * @brief   Boot-time pin check: no GPIO listed twice, GP0/GP1 not UART,
 *          and compile-time guards against kernel drivers that would claim
 *          our pins.
 * @owner   Team (integrator; change only by SHARED pull request)
 * @kind    drv
 * @reqs    NFR13
 */

#include "board_pins.h"        /* own header first (BARR-C 4.3.c) */
#include "app.h"
#include "config_device.h"

/* ---- private constants and types ------------------------------------ */

#if (DEVCNF_USE_IIC != 0)
#error "Kernel I2C driver would claim GP6-GP9: keep DEVCNF_USE_IIC 0"
#endif
#if (DEVCNF_USE_ADC != 0)
#error "Kernel ADC driver must be off: line_task owns the ADC"
#endif
#if defined(TM_CONSOLE_UART) && (TM_CONSOLE_UART != 0)
#error "UART console uses GP0/GP1, which the ultrasonic owns"
#endif

/* ---- private data ---------------------------------------------------- */

/* Every pin the firmware drives or reads (spares excluded). */
static uint8_t const g_used_pins[] =
{
    PIN_US_TRIG,   PIN_US_ECHO,   PIN_IMU_SDA,    PIN_IMU_SCL,
    PIN_ENC_L_A,   PIN_ENC_L_B,   PIN_LED_ALIVE,  PIN_BAR_DO,
    PIN_MOTOR_L_A, PIN_MOTOR_L_B, PIN_MOTOR_R_A,  PIN_MOTOR_R_B,
    PIN_PROBE,     PIN_SERVO,     PIN_ENC_R_A,    PIN_ENC_R_B,
    PIN_RGB,       PIN_IMU_INT1,  PIN_BTN_RUN,    PIN_BTN_RESET,
    PIN_BUZZER,    PIN_IR_LINE_L, PIN_IR_LINE_R
};

#define BOARD_PINS_USED_COUNT  (sizeof(g_used_pins) / sizeof(g_used_pins[0]))

/* IO_BANK0 GPIOn_CTRL.FUNCSEL, bits 0-4 (RP2040 datasheet 2.19.6.1). Read
   through the kernel's register access: the Pico SDK's gpio.h cannot be
   included next to the kernel headers (both define size_t and the
   peripheral base addresses).                                          */
#define BOARD_FUNCSEL_MASK     (0x1Fu)

/* ---- private function prototypes ------------------------------------- */

static uint32_t board_pin_function(uint32_t pin);

/* ---- public functions ------------------------------------------------- */

/**
 * board_pins_check
 * @brief   Prints "PIN CLASH GP<n>" for every pin listed twice, then the
 *          current function of GP0 and GP1 (expect 5 = SIO or 31 = NULL,
 *          never 2 = UART).
 * @return  none
 * @context usermain, before any task starts
 */
void board_pins_check(void)
{
    uint32_t idx = 0u;
    uint32_t jdx = 0u;

    for (idx = 0u; idx < BOARD_PINS_USED_COUNT; idx++)
    {
        for (jdx = idx + 1u; jdx < BOARD_PINS_USED_COUNT; jdx++)
        {
            if (g_used_pins[idx] == g_used_pins[jdx])
            {
                app_log("PIN CLASH GP%d\n",
                        (int32_t) g_used_pins[idx], 0, 0); /* cast: <= 29 */
            }
        }
    }

    app_log("GP0 func %d GP1 func %d\n",
            (int32_t) board_pin_function(PIN_US_TRIG), /* cast: 0..31 */
            (int32_t) board_pin_function(PIN_US_ECHO), /* cast: 0..31 */
            0);
}

/* ---- private functions ------------------------------------------------ */

/**
 * board_pin_function
 * @brief   Current FUNCSEL of one pin (5 = SIO, 2 = UART, 31 = NULL).
 * @param   pin  GPIO number 0..29
 * @return  0..31
 * @context any task
 */
static uint32_t board_pin_function(uint32_t pin)
{
    /* cast: register address 0x40014004 + 8*pin, kernel port type UW */
    UW const ctrl = in_w((UW) GPIO_CTRL(pin));

    return ((uint32_t) ctrl & BOARD_FUNCSEL_MASK); /* cast: UW is 32-bit */
}

/*** end of file ***/
