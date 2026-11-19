/**
 * @file    bar_ao.c
 * @brief   Analog barcode level detector (BAR_AO=1). A fixed threshold,
 *          like the module's DO comparator, loses a narrow white gap when
 *          the paper lifts: the spot blurs and the gap no longer reaches
 *          the threshold. This detector follows the lowest code while on
 *          white and the highest while on black, and switches when the
 *          signal moves back by a delta of BAR_AO_DELTA_PCT of the recent
 *          swing (at least BAR_AO_DELTA_MIN), for BAR_AO_CONFIRM samples
 *          in a row. A shallow dip between two bars is still a gap.
 *          State is one SH_BAR_AO word; no other state.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 * @reqs    FR2, NFR5
 */

#include "bar_ao.h"            /* own header first (BARR-C 4.3.c) */
#include "../app_bits.h"
#include "../app_layout.h"
#include "cfg_barcode.h"

/* ---- private constants and types ------------------------------------ */

#define BAR_AO_PCT_SCALE    (100u)
#define BAR_AO_AMP_OLD_W    (3u)    /* AMP = (3 x old + new) / 4 */
#define BAR_AO_AMP_DIV      (4u)

/* ---- private function prototypes ------------------------------------- */

static uint32_t bar_ao_get(uint32_t word, uint32_t pos, uint32_t width);
static uint32_t bar_ao_delta(uint32_t amp);

/* ---- public functions ------------------------------------------------- */

/**
 * bar_ao_init
 * @brief   Detector state at boot: white, no low seen yet, start swing.
 * @return  SH_BAR_AO word
 * @context init (usermain or host test)
 */
uint32_t bar_ao_init(void)
{
    uint32_t word = 0u;

    word = app_bits_put(word, SH_BAR_AO_EXT_POS, SH_BAR_AO_EXT_BITS,
                        BAR_AO_CODE_MAX);   /* first sample is lower */
    word = app_bits_put(word, SH_BAR_AO_AMP_POS, SH_BAR_AO_AMP_BITS,
                        BAR_AO_AMP_INIT / BAR_AO_AMP_LSB);
    return word;
}

/**
 * bar_ao_step
 * @brief   One ADC sample through the detector.
 * @param   word  SH_BAR_AO word before this sample
 * @param   code  sample, 0..4095, black-high (caller flips if needed)
 * @return  SH_BAR_AO word after it (LEVEL changes on a switch)
 * @context R4 timer ISR (BAR_AO=1), host tests
 */
uint32_t bar_ao_step(uint32_t word, uint32_t code)
{
    uint32_t       level = bar_ao_get(word, SH_BAR_AO_LEVEL_POS,
                                      SH_BAR_AO_LEVEL_BITS);
    uint32_t       pend  = bar_ao_get(word, SH_BAR_AO_PEND_POS,
                                      SH_BAR_AO_PEND_BITS);
    uint32_t       ext   = bar_ao_get(word, SH_BAR_AO_EXT_POS,
                                      SH_BAR_AO_EXT_BITS);
    uint32_t       opp   = bar_ao_get(word, SH_BAR_AO_OPP_POS,
                                      SH_BAR_AO_OPP_BITS);
    uint32_t       amp   = bar_ao_get(word, SH_BAR_AO_AMP_POS,
                                      SH_BAR_AO_AMP_BITS);
    uint32_t const v     = (code > BAR_AO_CODE_MAX) ? BAR_AO_CODE_MAX : code;
    uint32_t const delta = bar_ao_delta(amp);
    uint32_t       swing = 0u;

    if (0u == level)                       /* white: follow the lowest */
    {
        if (v <= ext)
        {
            ext  = v;
            pend = 0u;
        }
        else if (v >= (ext + delta))
        {
            if ((pend + 1u) >= BAR_AO_CONFIRM)
            {
                level = 1u;                /* white -> black */
                opp   = ext / BAR_AO_AMP_LSB;
                ext   = v;
                pend  = 0u;
            }
            else
            {
                pend++;
            }
        }
        else
        {
            pend = 0u;                     /* noise above the low */
        }
    }
    else                                   /* black: follow the highest */
    {
        if (v >= ext)
        {
            ext  = v;
            pend = 0u;
        }
        else if ((v + delta) <= ext)
        {
            if ((pend + 1u) >= BAR_AO_CONFIRM)
            {
                level = 0u;                /* black -> white */
                swing = (ext > (opp * BAR_AO_AMP_LSB))   /* peak - low */
                        ? (ext - (opp * BAR_AO_AMP_LSB)) : 0u;
                amp   = ((amp * BAR_AO_AMP_OLD_W)
                         + (swing / BAR_AO_AMP_LSB)
                         + (BAR_AO_AMP_DIV / 2u)) / BAR_AO_AMP_DIV;
                opp   = ext / BAR_AO_AMP_LSB;
                ext   = v;
                pend  = 0u;
            }
            else
            {
                pend++;
            }
        }
        else
        {
            pend = 0u;                     /* noise below the high */
        }
    }

    word = app_bits_put(0u, SH_BAR_AO_LEVEL_POS, SH_BAR_AO_LEVEL_BITS,
                        level);
    word = app_bits_put(word, SH_BAR_AO_PEND_POS, SH_BAR_AO_PEND_BITS,
                        pend);
    word = app_bits_put(word, SH_BAR_AO_EXT_POS, SH_BAR_AO_EXT_BITS, ext);
    word = app_bits_put(word, SH_BAR_AO_OPP_POS, SH_BAR_AO_OPP_BITS, opp);
    word = app_bits_put(word, SH_BAR_AO_AMP_POS, SH_BAR_AO_AMP_BITS, amp);
    return word;
}

/**
 * bar_ao_level
 * @brief   Level held in a SH_BAR_AO word.
 * @param   word  SH_BAR_AO word
 * @return  1 = black, 0 = white
 * @context any
 */
uint32_t bar_ao_level(uint32_t word)
{
    return bar_ao_get(word, SH_BAR_AO_LEVEL_POS, SH_BAR_AO_LEVEL_BITS);
}

/* ---- private functions ------------------------------------------------ */

/**
 * bar_ao_get
 * @brief   Reads one SH_BAR_AO field.
 * @context any
 */
static uint32_t bar_ao_get(uint32_t word, uint32_t pos, uint32_t width)
{
    return app_bits_get(word, pos, width);
}

/**
 * bar_ao_delta
 * @brief   Switch delta for a smoothed swing.
 * @param   amp  SH_BAR_AO AMP (ADC code / BAR_AO_AMP_LSB)
 * @return  delta in ADC codes, >= BAR_AO_DELTA_MIN
 * @context any
 */
static uint32_t bar_ao_delta(uint32_t amp)
{
    uint32_t const delta = (amp * BAR_AO_AMP_LSB * BAR_AO_DELTA_PCT)
                           / BAR_AO_PCT_SCALE;

    return (delta < BAR_AO_DELTA_MIN) ? BAR_AO_DELTA_MIN : delta;
}

/*** end of file ***/
