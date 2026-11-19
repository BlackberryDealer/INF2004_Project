/**
 * @file    drv_ir_adc.h
 * @brief   ADC0/ADC1 driver for left and right IR line sensors.
 * @owner   Buddy 3 (line, barcode)
 * @kind    drv
 */

#ifndef DRV_IR_ADC_H
#define DRV_IR_ADC_H

#include <stdint.h>
#include <stdbool.h>

extern void drv_ir_adc_init(void);
extern void drv_ir_adc_read(uint32_t * p_left, uint32_t * p_right);

#endif /* DRV_IR_ADC_H */

/*** end of file ***/