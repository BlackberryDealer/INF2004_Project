/**
 * @file    drv_ir_adc.c
 * @brief   ADC0/ADC1 4-sample averaging driver for IR line sensors.
 * @owner   Buddy 3 (line, barcode)
 * @kind    drv
 * @reqs    FR1, NFR1
 */

#include "drv_ir_adc.h"
#include "../board_pins.h"

#ifndef DRV_IR_ADC_USE_REGS
#define DRV_IR_ADC_USE_REGS (0)
#endif

#if (0 == DRV_IR_ADC_USE_REGS)
#include "hardware/adc.h"
#else
#include "hardware/structs/adc.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/iobank0.h"
#endif

/* ---- private constants and types ------------------------------------ */

#define DRV_IR_ADC_SAMPLES_PER_CH (4u)
#define DRV_IR_ADC_AVG_SHIFT      (2u)
#define DRV_IR_ADC_CODE_MASK      (0x0FFFu)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static uint32_t drv_ir_adc_read_channel(uint32_t channel);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_ir_adc_init
 * @brief   Initialises the RP2040 ADC block and configures GP26/GP27.
 * @return  none
 * @context line_task (startup)
 */
void drv_ir_adc_init(void)
{
#if (0 == DRV_IR_ADC_USE_REGS)
    adc_init();
    adc_gpio_init(PIN_IR_LINE_L);
    adc_gpio_init(PIN_IR_LINE_R);
#else
    adc_hw->cs = ADC_CS_EN_BITS;

    while (0u == (adc_hw->cs & ADC_CS_READY_BITS))
    {
        /* Wait for the ADC hardware block to report ready (< 2 us). */
    }

    iobank0_hw->io[PIN_IR_LINE_L].ctrl = GPIO_FUNC_NULL;
    iobank0_hw->io[PIN_IR_LINE_R].ctrl = GPIO_FUNC_NULL;
    padsbank0_hw->io[PIN_IR_LINE_L]    = PADS_BANK0_GPIO0_OD_BITS;
    padsbank0_hw->io[PIN_IR_LINE_R]    = PADS_BANK0_GPIO0_OD_BITS;
#endif
}

/**
 * drv_ir_adc_read
 * @brief   Reads 4 samples on ADC0 and 4 samples on ADC1 and averages each.
 * @param   p_left  Output pointer for left raw average (0..4095).
 * @param   p_right Output pointer for right raw average (0..4095).
 * @return  none
 * @context line_task (10 ms periodic)
 */
void drv_ir_adc_read(uint32_t * p_left, uint32_t * p_right)
{
    uint32_t const avg_left  = drv_ir_adc_read_channel(BOARD_ADC_CH_LINE_L);
    uint32_t const avg_right = drv_ir_adc_read_channel(BOARD_ADC_CH_LINE_R);

    if (((uint32_t *) 0) != p_left)
    {
        *p_left = avg_left;
    }
    else
    {
        /* Caller did not request the left channel reading. */
    }

    if (((uint32_t *) 0) != p_right)
    {
        *p_right = avg_right;
    }
    else
    {
        /* Caller did not request the right channel reading. */
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * drv_ir_adc_read_channel
 * @brief   Selects an ADC channel and returns the 4-sample average.
 * @param   channel ADC input channel (0 for left, 1 for right).
 * @return  12-bit averaged ADC code (0..4095).
 * @context line_task
 */
static uint32_t drv_ir_adc_read_channel(uint32_t channel)
{
    uint32_t sum = 0u;
    uint32_t idx = 0u;

#if (0 == DRV_IR_ADC_USE_REGS)
    adc_select_input(channel);

    for (idx = 0u; idx < DRV_IR_ADC_SAMPLES_PER_CH; idx++)
    {
        uint16_t const raw = adc_read();

        /* cast: widen 16-bit SDK return value to 32-bit accumulator */
        sum += (((uint32_t) raw) & DRV_IR_ADC_CODE_MASK);
    }
#else
    uint32_t const cs_val = (adc_hw->cs & ~ADC_CS_AINSEL_BITS);

    adc_hw->cs =
        (cs_val | ((channel << ADC_CS_AINSEL_LSB) & ADC_CS_AINSEL_BITS));

    for (idx = 0u; idx < DRV_IR_ADC_SAMPLES_PER_CH; idx++)
    {
        adc_hw->cs |= ADC_CS_START_ONCE_BITS;

        while (0u == (adc_hw->cs & ADC_CS_READY_BITS))
        {
            /* 96 ADC clocks = 2 us per conversion */
        }

        sum += (adc_hw->result & DRV_IR_ADC_CODE_MASK);
    }
#endif

    return (sum >> DRV_IR_ADC_AVG_SHIFT);
}

/*** end of file ***/
