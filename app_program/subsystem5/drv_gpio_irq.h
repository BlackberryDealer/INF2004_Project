/**
 * @file    drv_gpio_irq.h
 * @brief   R1 gpio_bank0_isr: one IO_IRQ_BANK0 handler, per-pin callbacks.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

#include <stdint.h>
#include "../app.h"

#define DRV_GPIO_IRQ_FALL   (0x4u)    /* same values as the RP2040 INTR bits */
#define DRV_GPIO_IRQ_RISE   (0x8u)
#define DRV_GPIO_IRQ_BOTH   (0xCu)

/* Callback runs inside R1 (interrupt context): keep it short.
   events = which edges happened; now_us = timer read once at R1 entry. */
typedef void (*drv_gpio_irq_cb_t)(uint32_t events, uint32_t now_us);

extern ER drv_gpio_irq_init(void);  /* registers R1 with tk_def_int */
extern ER drv_gpio_irq_attach(uint32_t pin, uint32_t edges,
                              drv_gpio_irq_cb_t p_callback);  /* max 4 pins */

#endif /* DRV_GPIO_IRQ_H */

/*** end of file ***/
