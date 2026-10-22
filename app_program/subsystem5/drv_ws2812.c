/**
 * @file    drv_ws2812.c
 * @brief   WS2812 driver: pico-examples PIO program (ws2812.pio), set up
 *          with integer clock dividers only.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 * @reqs    FR16
 */

#include "drv_ws2812.h"        /* own header first (BARR-C 4.3.c) */
#include <stddef.h>            /* NULL */
#include "../board_pins.h"
#include "hardware/pio.h"      /* SDK side only: no kernel headers */
#include "ws2812.pio.h"

/* ---- private constants and types ------------------------------------ */

/* 800 kHz bit rate x 10 PIO cycles per bit (T1 + T2 + T3) = 8 MHz.
   clk_sys 125 MHz (B.1) / 8 MHz = 15.625 = 15 + 160/256.            */
#define DRV_WS2812_DIV_INT     (15u)
#define DRV_WS2812_DIV_FRAC8   (160u)
#define DRV_WS2812_BITS        (24u)    /* GRB, 8 bits each       */
#define DRV_WS2812_CH_MASK     (0xFFu)
#define DRV_WS2812_PIO_COUNT   (2u)

/* ---- private data ---------------------------------------------------- */

static PIO      gp_ws_pio      = NULL;  /* NULL until init succeeds */
static uint32_t g_ws_sm        = 0u;
static bool     gb_is_ws_ready = false;

/* ---- private function prototypes ------------------------------------- */

static bool drv_ws2812_try(PIO p_pio);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_ws2812_init
 * @brief   Tries PIO1 then PIO0; loads the program and starts one SM.
 * @return  true if the LED is usable
 * @context mission_task (priority 16: runs after motion_task has loaded
 *          the encoder program at offset 0)
 */
bool drv_ws2812_init(void)
{
    PIO const pio_try[DRV_WS2812_PIO_COUNT] = {pio1, pio0};

    for (uint32_t idx = 0u;
         (idx < DRV_WS2812_PIO_COUNT) && (!gb_is_ws_ready); idx++)
    {
        gb_is_ws_ready = drv_ws2812_try(pio_try[idx]);
    }

    return gb_is_ws_ready;   /* false: caller raises APP_FAULT_PIO_LOAD */
}

/**
 * drv_ws2812_set
 * @brief   Shows one colour. Does nothing if init failed.
 * @param   red   0..255
 * @param   green 0..255
 * @param   blue  0..255
 * @context mission_task (only when the colour changes, so the 8-word
 *          FIFO never fills and put_blocking never waits)
 */
void drv_ws2812_set(uint32_t red, uint32_t green, uint32_t blue)
{
    uint32_t grb = 0u;

    if (gb_is_ws_ready)
    {
        grb = ((green & DRV_WS2812_CH_MASK) << 16u)
              | ((red & DRV_WS2812_CH_MASK) << 8u)
              | (blue & DRV_WS2812_CH_MASK);
        pio_sm_put_blocking(gp_ws_pio, g_ws_sm, grb << 8u);
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * drv_ws2812_try
 * @brief   Loads and starts the program on one PIO block if it has a free
 *          state machine and instruction space. Never resets the block.
 * @param   p_pio pio0 or pio1
 * @return  true on success
 * @context mission_task
 */
static bool drv_ws2812_try(PIO p_pio)
{
    bool           b_is_ok = false;
    int            sm      = -1;
    int            offset  = -1;
    pio_sm_config  cfg     = {0};

    if (pio_can_add_program(p_pio, &ws2812_program))
    {
        sm = pio_claim_unused_sm(p_pio, false);
    }
    if (sm >= 0)
    {
        offset = pio_add_program(p_pio, &ws2812_program);
        if (offset < 0)
        {
            pio_sm_unclaim(p_pio, (uint) sm);  /* cast: sm >= 0 */
        }
    }
    if ((sm >= 0) && (offset >= 0))
    {
        g_ws_sm   = (uint32_t) sm;      /* cast: 0..3 */
        gp_ws_pio = p_pio;
        pio_gpio_init(p_pio, PIN_RGB);
        (void) pio_sm_set_consecutive_pindirs(p_pio, g_ws_sm, PIN_RGB, 1u,
                                              true); /* valid pin */
        cfg = ws2812_program_get_default_config(
            (uint) offset);                      /* cast: offset >= 0 */
        sm_config_set_sideset_pins(&cfg, PIN_RGB);
        sm_config_set_out_shift(&cfg, false, true, DRV_WS2812_BITS);
        sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
        /* cast: 160 fits a byte */
        sm_config_set_clkdiv_int_frac8(&cfg, DRV_WS2812_DIV_INT,
                                       (uint8_t) DRV_WS2812_DIV_FRAC8);
        /* cast: offset >= 0; init cannot fail with a claimed SM */
        (void) pio_sm_init(p_pio, g_ws_sm, (uint) offset, &cfg);
        pio_sm_set_enabled(p_pio, g_ws_sm, true);
        b_is_ok = true;
    }

    return b_is_ok;
}

/*** end of file ***/
