/**
 * @file    drv_gpio_irq.c
 * @brief   R1: the only GPIO bank 0 interrupt handler. Serves GP1 (echo),
 *          GP7 (barcode) and GP19 (IMU INT1) through a callback table.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    NFR5a (edge timing), A12 (R1 < 20 us); deviation D6
 */

/* SDK headers come before our own header here (exception to BARR-C
   4.3.c): the kernel's rp2040 sysdef.h, pulled in by app.h, defines
   GPIO_IN/GPIO_OUT and other hardware macros that break the SDK
   headers if they are parsed afterwards. See SPEC-QUESTION Q1.     */
#include "hardware/gpio.h"
#include "drv_gpio_irq.h"      /* own header, after the SDK (Q1) */
#include <stddef.h>             /* NULL */
#include "../board_pins.h"
#include "sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_GPIO_IRQ_MAX   (4u)

typedef struct
{
    uint32_t          pin;
    drv_gpio_irq_cb_t p_callback;
} drv_gpio_irq_entry_t;

/* ---- private data ---------------------------------------------------- */

/* Filled by start functions before the pin's interrupt is enabled. An
   entry is complete before g_irq_count includes it, so R1 never sees a
   half-written entry.                                                 */
static drv_gpio_irq_entry_t g_irq_table[DRV_GPIO_IRQ_MAX];
static volatile uint32_t    g_irq_count = 0u;

/* ---- private function prototypes ------------------------------------- */

static void gpio_bank0_isr(UINT intno);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_gpio_irq_init
 * @brief   Registers R1 for IO_IRQ_BANK0 and enables it at level 2.
 * @return  E_OK, or the kernel error
 * @context usermain (init phase)
 */
ER drv_gpio_irq_init(void)
{
    T_DINT dint = {0};
    ER     ercd = E_OK;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) gpio_bank0_isr;   /* cast: kernel handler type (D6) */
    ercd        = tk_def_int(BOARD_IRQ_GPIO_BANK0, &dint);

    if (E_OK == ercd)
    {
        EnableInt(BOARD_IRQ_GPIO_BANK0, BOARD_IRQ_PRI_GPIO);
    }

    return ercd;
}

/**
 * drv_gpio_irq_attach
 * @brief   Makes a pin an input and routes its edges to a callback.
 * @param   pin        GPIO number from board_pins.h
 * @param   edges      DRV_GPIO_IRQ_RISE, _FALL or _BOTH
 * @param   p_callback runs in R1 with (events, now_us)
 * @return  E_OK; E_PAR for no callback or no edge; E_LIMIT if 4 used
 * @context start functions (usermain), before the tasks need the pin
 */
ER drv_gpio_irq_attach(uint32_t pin, uint32_t edges,
                       drv_gpio_irq_cb_t p_callback)
{
    ER             ercd  = E_OK;
    uint32_t const count = g_irq_count;
    uint32_t const mask  = edges & DRV_GPIO_IRQ_BOTH;

    if ((NULL == p_callback) || (0u == mask))
    {
        ercd = E_PAR;
    }
    else if (count >= DRV_GPIO_IRQ_MAX)
    {
        ercd = E_LIMIT;
    }
    else
    {
        gpio_init(pin);
        gpio_set_dir(pin, false);
        g_irq_table[count].pin        = pin;      /* entry first ...      */
        g_irq_table[count].p_callback = p_callback;
        g_irq_count                   = count + 1u;
        gpio_set_irq_enabled(pin, mask, true);    /* ... then the IRQ    */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * gpio_bank0_isr
 * @brief   R1: reads the timer once, then acknowledges and dispatches
 *          every pending edge of each attached pin.
 * @param   intno unused (always IO_IRQ_BANK0)
 * @context interrupt, level 2
 */
static void gpio_bank0_isr(UINT intno)
{
    uint32_t const now_us = sys_tick_now_us();
    uint32_t const count  = g_irq_count;
    uint32_t       events = 0u;

    (void) intno;  /* unused: one source */
#if (APP_PROBE != 0)
    app_probe_set(true);
#endif

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        events = gpio_get_irq_event_mask(g_irq_table[idx].pin)
                 & DRV_GPIO_IRQ_BOTH;
        if (0u != events)
        {
            gpio_acknowledge_irq(g_irq_table[idx].pin, events);
            g_irq_table[idx].p_callback(events, now_us);
        }
    }

#if (APP_PROBE != 0)
    app_probe_set(false);
#endif
}

/*** end of file ***/
