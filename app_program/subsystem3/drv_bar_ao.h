/**
 * @file    drv_bar_ao.h
 * @brief   R4 bar_ao_timer_isr (BAR_AO=1): samples the barcode module's
 *          AO on GP28 every BAR_AO_PERIOD_US from TIMER ALARM0.
 * @owner   Buddy 3 (line, barcode)
 * @kind    drv
 */

#ifndef DRV_BAR_AO_H
#define DRV_BAR_AO_H

#include <stdint.h>
#include "../app.h"

/* Runs inside R4 (interrupt context): keep it short. code = 12-bit ADC
   sample; t_us = the alarm time it was taken for (BAR_AO_PERIOD_US
   steps, so edge gaps are exact multiples of the period).             */
typedef void (*drv_bar_ao_cb_t)(uint32_t code, uint32_t t_us);

extern ER drv_bar_ao_start(drv_bar_ao_cb_t p_callback);

#endif /* DRV_BAR_AO_H */

/*** end of file ***/
