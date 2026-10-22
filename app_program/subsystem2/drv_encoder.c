/**
 * @file    drv_encoder.c
 * @brief   Quadrature encoder driver: the pico-examples quadrature_encoder
 *          PIO program (quad_enc.pio, BSD-3) at offset 0, one state machine
 *          per wheel, counting every edge of both Hall channels.
 * @owner   Buddy 2 (motion)
 * @kind    drv
 * @reqs    FR15, NFR2, NFR3 (speed and distance feedback)
 *
 * Pins: GP4/GP5 left, GP16/GP17 right (the program needs B = A + 1).
 * Owner: motion_task only. Includes only Pico SDK headers, never app.h.
 * The program uses computed jumps, so it must sit at instruction address 0
 * of its PIO block; pio1 is tried first because the template's radio
 * driver already uses a state machine of pio0.
 */

#include "drv_encoder.h"       /* own header first (BARR-C 4.3.c) */
#include <stddef.h>            /* NULL */
#include "../board_pins.h"
#include "cfg_motion.h"
#include "hardware/pio.h"

/* quad_enc.pio.h is generated third-party code (pioasm 2.2.0 from the
   pico-examples program). Its c-sdk helper functions are not written to
   this project's warning flags, so those warnings are switched off for
   that one header only (docs/BARR_C_DEVIATIONS.md, D9).               */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#include "quad_enc.pio.h"
#pragma GCC diagnostic pop

/* ---- private constants and types ------------------------------------ */

#define DRV_ENC_BLOCK_COUNT    (2u)   /* pio0 and pio1                   */
#define DRV_ENC_OFFSET         (0u)   /* computed jumps need address 0   */
#define DRV_ENC_MAX_STEP_RATE  (0)    /* 0 = sample at the full clk_sys  */
#define DRV_ENC_OK             (0)
#define DRV_ENC_NO_SPACE       (-1)

/* ---- private data ---------------------------------------------------- */

static PIO      gp_enc_pio       = NULL;   /* block in use, NULL if none */
static uint32_t g_enc_pio_index  = 0u;
static uint32_t g_enc_sm_l       = 0u;
static uint32_t g_enc_sm_r       = 0u;

/* ---- private function prototypes ------------------------------------- */

static bool     drv_encoder_try_block(PIO p_pio, uint32_t pio_index);
static uint32_t drv_encoder_raw(uint32_t sm, bool b_is_reversed);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_encoder_init
 * @brief   Loads the program into pio1 or pio0 and starts both counters.
 * @return  0 = OK, -1 = no PIO block had room for the program and two
 *          state machines (nothing is left claimed or loaded)
 * @context motion_task (start of task)
 */
int32_t drv_encoder_init(void)
{
    PIO const      blocks[DRV_ENC_BLOCK_COUNT]  = { pio1, pio0 };
    uint32_t const indexes[DRV_ENC_BLOCK_COUNT] = { 1u, 0u };
    uint32_t       idx      = 0u;
    bool           b_is_ok  = false;

    while ((!b_is_ok) && (idx < DRV_ENC_BLOCK_COUNT))
    {
        b_is_ok = drv_encoder_try_block(blocks[idx], indexes[idx]);
        idx++;
    }

    return (b_is_ok ? DRV_ENC_OK : DRV_ENC_NO_SPACE);
}

/**
 * drv_encoder_read
 * @brief   Latest raw count of each wheel (E3 image, modular).
 * @param   p_raw_l  receives the left raw count
 * @param   p_raw_r  receives the right raw count
 * @return  none
 * @context motion_task
 */
void drv_encoder_read(uint32_t * p_raw_l, uint32_t * p_raw_r)
{
    uint32_t raw_l = 0u;
    uint32_t raw_r = 0u;

    /* Never read an unloaded state machine: the SDK read would block. */
    if (NULL != gp_enc_pio)
    {
        raw_l = drv_encoder_raw(g_enc_sm_l, MOTION_ENC_L_REVERSED);
        raw_r = drv_encoder_raw(g_enc_sm_r, MOTION_ENC_R_REVERSED);
    }

    if ((NULL != p_raw_l) && (NULL != p_raw_r))
    {
        *p_raw_l = raw_l;
        *p_raw_r = raw_r;
    }
}

/**
 * drv_encoder_location
 * @brief   Which PIO block and state machines the counters use, for the
 *          boot log printed by motion_task (see the SPEC-QUESTION in the
 *          header).
 * @param   p_pio_index  receives 0 or 1
 * @param   p_sm_l       receives the left state machine 0..3
 * @param   p_sm_r       receives the right state machine 0..3
 * @return  true if the program is loaded; outputs untouched otherwise
 * @context motion_task
 */
bool drv_encoder_location(uint32_t * p_pio_index, uint32_t * p_sm_l,
                          uint32_t * p_sm_r)
{
    bool b_is_loaded = (NULL != gp_enc_pio);

    if (b_is_loaded && (NULL != p_pio_index) && (NULL != p_sm_l)
        && (NULL != p_sm_r))
    {
        *p_pio_index = g_enc_pio_index;
        *p_sm_l      = g_enc_sm_l;
        *p_sm_r      = g_enc_sm_r;
    }

    return b_is_loaded;
}

/* ---- private functions ------------------------------------------------ */

/**
 * drv_encoder_try_block
 * @brief   Tries one PIO block: program at offset 0 plus two free state
 *          machines. On partial success everything taken is given back.
 * @param   p_pio      pio0 or pio1
 * @param   pio_index  0 or 1, for the boot log
 * @return  true if both counters run in this block
 * @context motion_task (start of task)
 */
static bool drv_encoder_try_block(PIO p_pio, uint32_t pio_index)
{
    bool b_is_ok     = false;
    bool b_is_loaded = false;
    int  sm_l        = -1;
    int  sm_r        = -1;

    if (pio_can_add_program_at_offset(p_pio, &quadrature_encoder_program,
                                      DRV_ENC_OFFSET))
    {
        /* Cannot fail: the check above found room at offset 0. */
        (void) pio_add_program_at_offset(p_pio, &quadrature_encoder_program,
                                         DRV_ENC_OFFSET);
        b_is_loaded = true;
        sm_l = pio_claim_unused_sm(p_pio, false);  /* -1 if none free */
        sm_r = pio_claim_unused_sm(p_pio, false);
        b_is_ok = ((sm_l >= 0) && (sm_r >= 0));
    }

    if (b_is_ok)
    {
        /* casts: sm_l and sm_r are 0..3 here */
        g_enc_sm_l      = (uint32_t) sm_l;
        g_enc_sm_r      = (uint32_t) sm_r;
        g_enc_pio_index = pio_index;
        quadrature_encoder_program_init(p_pio, g_enc_sm_l, PIN_ENC_L_A,
                                        DRV_ENC_MAX_STEP_RATE);
        quadrature_encoder_program_init(p_pio, g_enc_sm_r, PIN_ENC_R_A,
                                        DRV_ENC_MAX_STEP_RATE);
        gp_enc_pio = p_pio;
    }
    else if (b_is_loaded)
    {
        /* Program fitted but fewer than two SMs were free: give back
           everything taken. cast: each claimed sm is 0..3              */
        if (sm_l >= 0)
        {
            pio_sm_unclaim(p_pio, (uint) sm_l);
        }
        if (sm_r >= 0)
        {
            pio_sm_unclaim(p_pio, (uint) sm_r);
        }
        pio_remove_program(p_pio, &quadrature_encoder_program,
                           DRV_ENC_OFFSET);
    }
    else
    {
        /* No room at offset 0 in this block: nothing was taken. */
    }

    return b_is_ok;
}

/**
 * drv_encoder_raw
 * @brief   Reads one counter as a modular 32-bit image (E3).
 * @param   sm             state machine 0..3
 * @param   b_is_reversed  true if this encoder counts the wrong way
 * @return  raw image; + direction = wheel driving forward
 * @context motion_task
 */
static uint32_t drv_encoder_raw(uint32_t sm, bool b_is_reversed)
{
    int32_t const count = quadrature_encoder_get_count(gp_enc_pio, sm);

    /* cast: two's-complement image, used only for modular differences
       (E3), so reinterpreting the sign is intended                     */
    uint32_t raw = (uint32_t) count;

    if (b_is_reversed)
    {
        raw = 0u - raw;   /* modular negate */
    }

    return raw;
}

/*** end of file ***/
