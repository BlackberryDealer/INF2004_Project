/**
 * @file    drv_gpio_irq.c
 * @brief   R1: the only GPIO bank 0 interrupt handler. Serves GP1 (echo),
 *          GP7 (barcode) and GP19 (IMU INT1) through a callback table.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    NFR5a (edge timing), A12 (R1 < 20 us); deviation D6
 */

#include "drv_gpio_irq.h"      /* own header first (BARR-C 4.3.c) */
#include <bsp/libbsp.h>         /* gpio_set_pin(): SIO input */
#include "../board_pins.h"
#include "sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define DRV_GPIO_IRQ_MAX   (4u)

/* IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6; offsets as in the
   SDK's regs/io_bank0.h). 4 bits per GPIO, 8 GPIOs per register; bit 2 =
   EDGE_LOW (fall), bit 3 = EDGE_HIGH (rise), the DRV_GPIO_IRQ_* values.  */
#define DRV_GPIO_IRQ_INTR0      (IO_BANK0_BASE + 0x0F0u) /* write 1 clears */
#define DRV_GPIO_IRQ_INTE0      (IO_BANK0_BASE + 0x100u) /* proc0 enable  */
#define DRV_GPIO_IRQ_INTS0      (IO_BANK0_BASE + 0x120u) /* proc0 status  */
#define DRV_GPIO_IRQ_PROC_BYTES (0x030u) /* PROC1_INTE0 - PROC0_INTE0     */
#define DRV_GPIO_IRQ_CPUID_MASK (1u)     /* SIO CPUID: 0 = core 0, 1 = 1  */
#define DRV_GPIO_IRQ_REG_SHIFT  (3u)    /* 8 GPIOs per register: pin >> 3 */
#define DRV_GPIO_IRQ_PIN_MASK   (7u)    /* position in register: pin & 7  */
#define DRV_GPIO_IRQ_BITS_SHIFT (2u)    /* 4 bits per GPIO                */
#define DRV_GPIO_IRQ_REG_BYTES  (4u)

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

/* Byte offset of the PROCn INTE/INTS set of the core whose NVIC enabled
   IO_IRQ_BANK0 (0 = proc0, 0x30 = proc1). The NVIC is per core and
   usermain may run on either core in the SMP build, so the edges must be
   routed to the core that actually unmasked the IRQ. Written once in
   drv_gpio_irq_init(), before any pin is attached.                     */
static UW g_proc_offset = 0u;

/* ---- private function prototypes ------------------------------------- */

static void gpio_bank0_isr(UINT intno);
static UW   drv_gpio_irq_reg(UW base, uint32_t pin);
static uint32_t drv_gpio_irq_shift(uint32_t pin);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_gpio_irq_init
 * @brief   Registers R1 for IO_IRQ_BANK0 and enables it at level 2 on
 *          the calling core, and records that core so attach() and R1
 *          use its PROCn INTE/INTS registers. The core ID read and the
 *          NVIC enable run with interrupts masked, so the task cannot
 *          migrate between them (no dispatch without PendSV).
 * @return  E_OK, or the kernel error
 * @context usermain (init phase)
 */
ER drv_gpio_irq_init(void)
{
    T_DINT dint   = {0};
    ER     ercd   = E_OK;
    UW     intsts = 0u;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) gpio_bank0_isr;   /* cast: kernel handler type (D6) */
    ercd        = tk_def_int(BOARD_IRQ_GPIO_BANK0, &dint);

    if (E_OK == ercd)
    {
        DI(intsts);
        g_proc_offset = (in_w(SIO_CPUID_ADDR) & DRV_GPIO_IRQ_CPUID_MASK)
                        * DRV_GPIO_IRQ_PROC_BYTES;
        EnableInt(BOARD_IRQ_GPIO_BANK0, BOARD_IRQ_PRI_GPIO);
        EI(intsts);
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
        (void) gpio_set_pin(pin, GPIO_MODE_IN);   /* pin < 30: cannot fail */
        g_irq_table[count].pin        = pin;      /* entry first ...      */
        g_irq_table[count].p_callback = p_callback;
        g_irq_count                   = count + 1u;
        /* clear stale edges, then enable (atomic set alias) */
        out_w(drv_gpio_irq_reg(DRV_GPIO_IRQ_INTR0, pin),
              mask << drv_gpio_irq_shift(pin));
        set_w(drv_gpio_irq_reg(DRV_GPIO_IRQ_INTE0 + g_proc_offset, pin),
              mask << drv_gpio_irq_shift(pin));   /* ... then the IRQ */
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
        events = (in_w(drv_gpio_irq_reg(DRV_GPIO_IRQ_INTS0 + g_proc_offset,
                                        g_irq_table[idx].pin))
                  >> drv_gpio_irq_shift(g_irq_table[idx].pin))
                 & DRV_GPIO_IRQ_BOTH;
        if (0u != events)
        {
            out_w(drv_gpio_irq_reg(DRV_GPIO_IRQ_INTR0, g_irq_table[idx].pin),
                  events << drv_gpio_irq_shift(g_irq_table[idx].pin));
            g_irq_table[idx].p_callback(events, now_us);
        }
    }

#if (APP_PROBE != 0)
    app_probe_set(false);
#endif
}

/**
 * drv_gpio_irq_reg
 * @brief   Address of the IO_BANK0 register holding a pin's 4 bits.
 * @param   base  DRV_GPIO_IRQ_INTR0 / _INTE0 / _INTS0
 * @param   pin   GPIO 0..29
 * @return  register address
 * @context any (also R1)
 */
static UW drv_gpio_irq_reg(UW base, uint32_t pin)
{
    return (base + ((pin >> DRV_GPIO_IRQ_REG_SHIFT) * DRV_GPIO_IRQ_REG_BYTES));
}

/**
 * drv_gpio_irq_shift
 * @brief   Bit position of a pin's 4 bits inside its register.
 * @param   pin  GPIO 0..29
 * @return  0, 4, ... 28
 * @context any (also R1)
 */
static uint32_t drv_gpio_irq_shift(uint32_t pin)
{
    return ((pin & DRV_GPIO_IRQ_PIN_MASK) << DRV_GPIO_IRQ_BITS_SHIFT);
}

/*** end of file ***/
