/**
 * @file    drv_bar_ao.c
 * @brief   R4 bar_ao_timer_isr (BAR_AO=1): TIMER ALARM0 fires every
 *          BAR_AO_PERIOD_US, re-arms itself on a fixed grid and passes
 *          one ADC2 sample of the barcode AO to barcode.c. A sample is
 *          skipped while line_task holds the ADC (drv_ir_adc.c).
 * @owner   Buddy 3 (line, barcode)
 * @kind    drv
 * @reqs    FR2, NFR5
 */

#include "drv_bar_ao.h"        /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "../subsystem5/sys_tick.h"
#include "cfg_barcode.h"
#include "drv_ir_adc.h"
#include "barcode.h"           /* barcode_ao_sample() */

/* ---- private constants and types ------------------------------------ */

#define DRV_BAR_AO_ALARM0_BIT  (1u)   /* TIMER INTR / INTE bit of ALARM0 */

/* ---- private data ---------------------------------------------------- */

static volatile uint32_t g_bar_ao_next = 0u;  /* E5 (E_BAR_AO_US) */

/* ---- private function prototypes ------------------------------------- */

static void bar_ao_timer_isr(UINT intno);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_bar_ao_start
 * @brief   Registers R4 and arms ALARM0 one period from now.
 * @return  E_OK or the tk_def_int error
 * @context usermain (start functions), once
 */
ER drv_bar_ao_start(void)
{
    T_DINT dint   = {0};
    ER     ercd   = E_OK;
    UW     intsts = 0u;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) bar_ao_timer_isr;   /* cast: kernel handler (D6) */
    ercd        = tk_def_int(BOARD_IRQ_TIMER0, &dint);

    if (E_OK == ercd)
    {
        DI(intsts);
        out_w(TIMER_INTR, DRV_BAR_AO_ALARM0_BIT);    /* stale alarm */
        set_w(TIMER_INTE, DRV_BAR_AO_ALARM0_BIT);
        g_bar_ao_next = sys_tick_now_us() + BAR_AO_PERIOD_US;
        out_w(TIMER_ALARM0, g_bar_ao_next);          /* arms ALARM0 */
        EnableInt(BOARD_IRQ_TIMER0, BOARD_IRQ_PRI_TIMER0);
        EI(intsts);
    }
    else
    {
        /* Not started: barcode_start() reports the error. */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * bar_ao_timer_isr
 * @brief   R4: acknowledge, re-arm on the BAR_AO_PERIOD_US grid (one
 *          period from now if the grid point already passed), then one
 *          AO sample to barcode_ao_sample().
 * @param   intno unused (always TIMER_IRQ_0)
 * @context interrupt, level 2
 */
static void bar_ao_timer_isr(UINT intno)
{
    uint32_t const t_us = g_bar_ao_next;
    uint32_t       next = t_us + BAR_AO_PERIOD_US;
    uint32_t       code = 0u;

    (void) intno;  /* unused: one source */
#if (APP_PROBE != 0)
    app_probe_set(true);         /* scope: R4 duration (PROBE=1 only) */
#endif
    out_w(TIMER_INTR, DRV_BAR_AO_ALARM0_BIT);
    /* cast: wrapping difference read as signed (late if <= 0) */
    if ((int32_t) (next - sys_tick_now_us()) <= 0)
    {
        next = sys_tick_now_us() + BAR_AO_PERIOD_US;  /* missed points */
    }
    else
    {
        /* On the grid. */
    }
    out_w(TIMER_ALARM0, next);
    /* An ALARM written for a time already past fires only after the
       32-bit wrap (71 min): if this ISR was held up past it, re-arm. */
    /* cast: wrapping difference read as signed */
    if ((int32_t) (next - sys_tick_now_us()) <= 0)
    {
        next = sys_tick_now_us() + BAR_AO_PERIOD_US;
        out_w(TIMER_ALARM0, next);
    }
    else
    {
        /* Armed in the future. */
    }
    g_bar_ao_next = next;

    if (drv_ir_adc_try_read_bar(&code))
    {
        barcode_ao_sample(code, t_us);
    }
    else
    {
        /* ADC busy or not ready: skip this sample. */
    }
#if (APP_PROBE != 0)
    app_probe_set(false);
#endif
}

/*** end of file ***/
