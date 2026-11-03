/**
 * @file    cfg_barcode.h
 * @brief   Configuration constants and compile-time checks for barcode decode.
 * @owner   Buddy 3 (line, barcode)
 * @kind    cfg
 */

#ifndef CFG_BARCODE_H
#define CFG_BARCODE_H

#include "../app_types.h"

#define BAR_DO_BLACK_IS_HIGH      (true)  /* MEASURE: A3 */
#define BAR_EDGE_REJECT_US        (1000u) /* chatter filter */
/* LM393 chatter (on-car 2026-11-03): the module has no hysteresis and DO
   flickers for longer than the 1 ms ISR filter at a bar edge (33 edges
   for a 30-edge barcode, capture dropped "edge missed"). In barcode_task:
   an element shorter than BAR_GLITCH_UM of travel (real ones are >= 1.5
   mm) is a flicker: it is dropped and its time added to the next one;
   and up to BAR_LEVEL_FIX_MAX edges per capture whose level reads the
   same as the last one (read mid-flicker) take the alternating level.
   Code 39 still validates the result.                                  */
#define BAR_GLITCH_UM             (600u)  /* < 0.4 x BAR_NARROW_MIN_UM */
#define BAR_LEVEL_FIX_MAX         (3u)
#define BAR_NARROW_MIN_UM         (1500u) /* half-scale narrow bar */
#define BAR_WIDE_MAX_UM           (9000u) /* full-scale wide bar */
#define BAR_DECODE_SPEED_MIN_MM_S (50u)
#define BAR_ELEMENTS              (29u) /* 3 chars x 9 + 2 gaps */
#define BAR_CAPTURE_MAX           (32u)
#define BAR_QUIET_MM              (30u) /* capture ends */
#define BAR_COOLDOWN_MM           (150u)
#define BAR_BRANCH_MIN_MM         (14u) /* single long bar = branch line */
#define BAR_WIDE_RATIO_X10_MIN    (18u) /* wide >= 1.8 x narrow */
#define BAR_RING_ENTRIES          (64u) /* power of two */
#define BAR_MAP_ENTRIES           (8u)

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