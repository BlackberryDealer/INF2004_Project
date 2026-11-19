/**
 * @file    drv_ir_adc.c
 * @brief   ADC0/ADC1 4-sample averaging driver for IR line sensors, and
 *          (BAR_AO=1) single ADC2 samples of the barcode module's AO.
 *          With BAR_AO=1 the R4 timer ISR also uses the ADC, so every
 *          conversion runs under one SIO hardware spinlock: line_task
 *          holds it with its core's interrupts masked (4 conversions,
 *          about 8 us); R4 only tries it and skips that sample if busy.
 * @owner   Buddy 3 (line, barcode)
 * @kind    drv
 * @reqs    FR1, NFR1
 */

#include "drv_ir_adc.h"
#include "../board_pins.h"
#include "cfg_barcode.h"       /* BAR_AO */
#include "../app_bits.h"
#include "../app_layout.h"
#include "hardware/sync.h"     /* SIO spinlock, interrupt mask */

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

/* SH_ADC_RDY: set last in drv_ir_adc_init() (line_task, once); read
   by R4, which may start before the ADC is enabled and must not wait
   on a disabled block.                                               */
static volatile uint8_t g_adc_ready = 0u;

/* ---- private function prototypes ------------------------------------- */

static uint32_t drv_ir_adc_read_channel(uint32_t channel);
static uint32_t drv_ir_adc_convert(uint32_t channel, uint32_t samples);

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
    if (0u != BAR_AO)
    {
#if (0 == DRV_IR_ADC_USE_REGS)
        adc_gpio_init(PIN_BAR_AO);
#else
        iobank0_hw->io[PIN_BAR_AO].ctrl = GPIO_FUNC_NULL;
        padsbank0_hw->io[PIN_BAR_AO]    = PADS_BANK0_GPIO0_OD_BITS;
#endif
        __mem_fence_release();               /* ADC set up before flag */
        g_adc_ready = (uint8_t) app_bits_put(0u, SH_ADC_RDY_READY_POS,
                                             SH_ADC_RDY_READY_BITS, 1u);
    }
    else
    {
        /* DO build: GP28 stays unused, R4 does not run. */
    }
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

/**
 * drv_ir_adc_try_read_bar
 * @brief   One ADC2 conversion of the barcode AO, if the ADC is free.
 * @param   p_code  output: 12-bit code (0..4095), written on success only
 * @return  true when sampled; false before init or while line_task
 *          holds the ADC (the caller skips this sample)
 * @context R4 timer ISR (BAR_AO=1)
 */
bool drv_ir_adc_try_read_bar(uint32_t * p_code)
{
    bool b_is_read = false;

    uint32_t const ready = app_bits_get((uint32_t) g_adc_ready,
                                        SH_ADC_RDY_READY_POS,
                                        SH_ADC_RDY_READY_BITS);

    if ((0u != ready) && (((uint32_t *) 0) != p_code))
    {
        spin_lock_t * const p_lock = spin_lock_instance(BOARD_SPINLOCK_ADC);

        if (spin_try_lock_unsafe(p_lock))
        {
            *p_code   = drv_ir_adc_convert(BOARD_ADC_CH_BAR_AO, 1u);
            spin_unlock_unsafe(p_lock);
            b_is_read = true;
        }
        else
        {
            /* line_task is converting: skip this sample. */
        }
    }
    else
    {
        /* ADC not set up yet. */
    }

    return b_is_read;
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

    if (0u != BAR_AO)
    {
        spin_lock_t * const p_lock = spin_lock_instance(BOARD_SPINLOCK_ADC);
        uint32_t const      saved  = spin_lock_blocking(p_lock);

        sum = drv_ir_adc_convert(channel, DRV_IR_ADC_SAMPLES_PER_CH);
        spin_unlock(p_lock, saved);
    }
    else
    {
        sum = drv_ir_adc_convert(channel, DRV_IR_ADC_SAMPLES_PER_CH);
    }

    return (sum >> DRV_IR_ADC_AVG_SHIFT);
}

/**
 * drv_ir_adc_convert
 * @brief   Selects an ADC channel and sums back-to-back conversions.
 * @param   channel ADC input channel (0 left, 1 right, 2 barcode AO).
 * @param   samples number of conversions (1..4)
 * @return  sum of 12-bit codes
 * @context line_task, R4 (caller holds the ADC when BAR_AO=1)
 */
static uint32_t drv_ir_adc_convert(uint32_t channel, uint32_t samples)
{
    uint32_t sum = 0u;
    uint32_t idx = 0u;

#if (0 == DRV_IR_ADC_USE_REGS)
    adc_select_input(channel);

    for (idx = 0u; idx < samples; idx++)
    {
        uint16_t const raw = adc_read();

        /* cast: widen 16-bit SDK return value to 32-bit accumulator */
        sum += (((uint32_t) raw) & DRV_IR_ADC_CODE_MASK);
    }
#else
    uint32_t const cs_val = (adc_hw->cs & ~ADC_CS_AINSEL_BITS);

    adc_hw->cs =
        (cs_val | ((channel << ADC_CS_AINSEL_LSB) & ADC_CS_AINSEL_BITS));

    for (idx = 0u; idx < samples; idx++)
    {
        adc_hw->cs |= ADC_CS_START_ONCE_BITS;

        while (0u == (adc_hw->cs & ADC_CS_READY_BITS))
        {
            /* 96 ADC clocks = 2 us per conversion */
        }

        sum += (adc_hw->result & DRV_IR_ADC_CODE_MASK);
    }
#endif

    return sum;
}

/*** end of file ***/
