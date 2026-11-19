/**
 * @file    cfg_barcode.h
 * @brief   Configuration constants and compile-time checks for barcode decode.
 * @owner   Buddy 3 (line, barcode)
 * @kind    cfg
 */

#ifndef CFG_BARCODE_H
#define CFG_BARCODE_H

#include "../app_types.h"
#include "../app_layout.h"   /* SH_BAR_AO limits for the checks */

#define BAR_DO_BLACK_IS_HIGH      (true)  /* MEASURE: A3 */
/* Analog barcode input (on-car 2026-11-19): with the paper lifted off
   the floor the module's fixed DO threshold loses the narrow white gaps
   (bars merge). BAR_AO=1 (the default) reads the same module's AO on
   GP28 (ADC2) every BAR_AO_PERIOD_US instead, and switches level on a
   relative rise or dip of at least BAR_AO_DELTA_PCT of the recent swing
   (never less than BAR_AO_DELTA_MIN), confirmed on BAR_AO_CONFIRM
   samples. The edges go into the same K2 ring, so the decoder is the
   same for DO and AO. On-car bench check 19 Nov 2026 passed: AO is the
   default; BAR_AO=0 (make BAR_AO=0) builds the old DO input.         */
#ifndef BAR_AO
#define BAR_AO                    (1u)
#endif
#define BAR_AO_BLACK_IS_HIGH      (true)  /* MEASURE: like DO and line AO */
#define BAR_AO_PERIOD_US          (500u)  /* 2 kHz */
#define BAR_AO_CONFIRM            (2u)    /* samples past the delta */
#define BAR_AO_DELTA_MIN          (100u)  /* ADC codes, TUNE */
#ifndef BAR_AO_DELTA_PCT
#define BAR_AO_DELTA_PCT          (35u)   /* % of the recent swing, TUNE */
#endif
#define BAR_AO_AMP_INIT           (800u)  /* ADC codes, TUNE */
#define BAR_AO_AMP_LSB            (16u)   /* SH_BAR_AO OPP / AMP step */
#define BAR_AO_CODE_MAX           (4095u) /* 12-bit ADC */
#define BAR_EDGE_REJECT_US        (1000u) /* chatter filter */
/* LM393 chatter after a real transition (on-car 2026-11-10 21:09: echo
   edges 1.0-2.1 ms after it, with random DO levels). An edge this soon
   after an accepted transition is folded into it, and the capture takes
   bar / space from alternation, not from DO (S3-U37). The narrowest
   element (BAR_NARROW_MIN_UM) at 300 mm/s still lasts 5 ms.          */
#define BAR_SETTLE_US             (4000u)
/* LM393 chatter (on-car 2026-11-03): the module has no hysteresis and DO
   flickers for longer than the 1 ms ISR filter at a bar edge (33 edges
   for a 30-edge barcode, capture dropped "edge missed"). In barcode_task:
   an element shorter than BAR_GLITCH_UM of travel (real ones are >= 1.5
   mm) is a flicker: it is dropped and its time added to the next one;
   and up to BAR_LEVEL_FIX_MAX edges per capture whose level reads the
   same as the last one (read mid-flicker) take the alternating level.
   Code 39 still validates the result.                                  */
#define BAR_GLITCH_UM             (400u)  /* < 0.4 mm: chatter vs small bar */
#define BAR_LEVEL_FIX_MAX         (3u)
#define BAR_NARROW_MIN_UM         (1500u) /* half-scale narrow bar */
#define BAR_WIDE_MAX_UM           (9000u) /* full-scale wide bar */
#define BAR_DECODE_SPEED_MIN_MM_S (50u)
/* S3-U40 (Hanz 2026-11-10 22:48): cruise is too fast for small, thin
   barcodes. After the first bars of any barcode (S3-U45: no clean start
   guard needed), always ask mission to back up and read the whole strip
   at the slow reread speed. 0 = read at cruise and rewind only after a
   failed read.                                                      */
#define BAR_REREAD_ALWAYS         (1u)
#define BAR_ELEMENTS              (29u) /* 3 chars x 9 + 2 gaps */
#define BAR_CAPTURE_MAX           (32u)
#define BAR_QUIET_MM              (30u) /* capture ends */
#define BAR_COOLDOWN_MM           (150u)
#define BAR_BRANCH_MIN_MM         (14u) /* single long bar = branch line */
#define BAR_WIDE_RATIO_X10_MIN    (18u) /* wide >= 1.8 x narrow */
/* Last split try, bars only (S3-U51): a grey bar read wide-ish. */
#define BAR_SPLIT_RATIO_X10_LOW   (13u) /* wide bar >= 1.3 x narrow */
#define BAR_RING_ENTRIES          (64u) /* power of two */
#define BAR_MAP_ENTRIES           (8u)
/* Bring-up diagnosis (S3-U36): keep the last raw edges and print them
   after a failed capture. Set BAR_EDGE_LOG_ON to 0u for the demo. */
#define BAR_EDGE_LOG_ON           (1u)
#define BAR_EDGE_LOG_MAX          (48u) /* edges kept, ~1.5 barcodes */
#define BAR_EDGE_LOG_EVERY        (4u)  /* task cycles per printed line */

#if ((BAR_AO_CONFIRM * BAR_AO_PERIOD_US) < BAR_EDGE_REJECT_US)
#error "AO level changes must be at least the edge filter apart"
#endif
#if ((BAR_AO_CONFIRM - 1u) > SH_BAR_AO_PEND_MAX)
#error "SH_BAR_AO.PEND must count to BAR_AO_CONFIRM - 1"
#endif
#if ((BAR_AO_CODE_MAX / BAR_AO_AMP_LSB) > SH_BAR_AO_AMP_MAX) \
    || ((BAR_AO_CODE_MAX / BAR_AO_AMP_LSB) > SH_BAR_AO_OPP_MAX) \
    || (BAR_AO_CODE_MAX > SH_BAR_AO_EXT_MAX) \
    || (BAR_AO_CODE_MAX > SH_BAR_AOS_CODE_MAX)
#error "SH_BAR_AO / SH_BAR_AOS must hold every ADC code"
#endif

#if ((2u * BAR_EDGE_REJECT_US) >                                               \
     ((BAR_NARROW_MIN_UM * 1000u) / APP_SPEED_PHYS_MAX_MM_S))
#error "Edge filter would reject the narrowest bar at maximum speed"
#endif

#if (((BAR_WIDE_MAX_UM * 1000u) / BAR_DECODE_SPEED_MIN_MM_S) > (32767u * 16u))
#error "Widest element at the slowest speed no longer fits 15 bits of 16 us"
#endif

/* Part C.4b: tie configuration constants to their stored field domains. */
#if ((BAR_ELEMENTS > BAR_CAPTURE_MAX) || (BAR_CAPTURE_MAX > 63u))
#error "BAR_CAPTURE_MAX must hold BAR_ELEMENTS and fit 6-bit N_ELEM (<= 63)"
#endif

#if ((0u == BAR_RING_ENTRIES) ||                                               \
     (0u != (BAR_RING_ENTRIES & (BAR_RING_ENTRIES - 1u))) ||                   \
     (BAR_RING_ENTRIES > 64u))
#error "BAR_RING_ENTRIES must be a power of two and fit 6-bit index (<= 64)"
#endif

#if ((0u == BAR_MAP_ENTRIES) || (BAR_MAP_ENTRIES > APP_C39_CHAR_COUNT))
#error "BAR_MAP_ENTRIES must be in 1..APP_C39_CHAR_COUNT"
#endif

#endif /* CFG_BARCODE_H */

/*** end of file ***/