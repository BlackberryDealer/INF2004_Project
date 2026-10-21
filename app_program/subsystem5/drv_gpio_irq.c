/**
 * @file    drv_gpio_irq.c
 * @brief   Phase 0 STUB of R1, the GPIO bank-0 interrupt dispatcher: nothing is
 *          registered.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    drv
 * @reqs    FR6, FR13, NFR5
 */

#include "drv_gpio_irq.h"      /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * drv_gpio_irq_init
 * @brief   STUB: will create R1 (tk_def_int).
 * @return  E_OK
 * @context usermain
 */
ER drv_gpio_irq_init(void)
{
    return E_OK;
}

/**
 * drv_gpio_irq_attach
 * @brief   STUB: register a callback for one pin.
 * @param   pin         GPIO number from board_pins.h
 * @param   edges       DRV_GPIO_IRQ_* edges
 * @param   p_callback  runs inside R1
 * @return  E_OK
 * @context usermain or a task's init
 */
ER drv_gpio_irq_attach(uint32_t pin, uint32_t edges,
                       drv_gpio_irq_cb_t p_callback)
{
    (void) pin;   /* stub: unused until the owner's version */
    (void) edges;
    (void) p_callback;

    return E_OK;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
