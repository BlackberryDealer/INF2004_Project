/**
 * @file    cfg_line.h
 * @brief   Configuration constants and compile-time checks for line sensing.
 * @owner   Buddy 3 (line, barcode)
 * @kind    cfg
 */

#ifndef CFG_LINE_H
#define CFG_LINE_H

#include "../app_types.h"

#ifndef LINE_BLACK_IS_HIGH
#define LINE_BLACK_IS_HIGH       (true) /* MEASURE: A1 */
#endif
#define LINE_CAL_MIN_SPAN        (400u) /* ADC codes; below => cal fails */
#define LINE_NORM_MAX            (1000u)
#define LINE_WHITE_THRESHOLD     (300u)  /* normalised; below = white   */
#define LINE_BLACK_THRESHOLD     (700u)  /* normalised; above = black   */
#define LINE_SAT_THRESHOLD       (900u)  /* channel fully over black    */
#define LINE_JCT_SUM_MIN         (1400u) /* TUNE: nL + nR on a junction */
#define LINE_LOST_CYCLES         (20u)   /* 200 ms, NFR4 definition     */
/* Steering (on-car 2026-11-03: the car wandered off a straight line and
   kept needing a LINE_LOST search). norm_l - norm_r is 0 whenever both
   sensors read white, so the car drove straight on with no idea where
   the line went. Now: both white -> steer back toward the side the line
   was last seen on; and the error is smoothed (1/LINE_STEER_FILT_DIV of
   the change per 10 ms cycle) so sensor noise does not jerk the wheels. */
/* Damping (on-car 2026-11-03: the car still swayed on straights): the
   output adds LINE_STEER_KD x the change of the smoothed error since the
   last cycle, so a fast swing toward the line is braked before it
   overshoots. 0 = plain smoothed error. Output clamped to +-1000.      */
#define LINE_RECOVER_STEER       (350)   /* TUNE: both white, permille  */
#define LINE_SIDE_MIN            (150)   /* TUNE: |error| that sets side */
#define LINE_STEER_FILT_DIV      (2)     /* TUNE: 1 = no smoothing      */
/* On-car 2026-11-03: of KP/KD 3/0, 3/4 and 2/6, 3/0 was the smoothest
   and the only one that never lost the line; damping is kept switchable. */
#define LINE_STEER_KD            (0)     /* TUNE: 0 = no damping        */
#define LINE_STEER_MAX           (1000)  /* SB steering field domain    */
#define LINE_JUNCTION_MIN_MM     (10u)   /* TUNE from A22 logs          */
#define LINE_JUNCTION_CONFIRM_MM (30u)   /* TUNE from A22 logs          */
/* SPEC-QUESTION: LINE_BRANCH_EARLY_MM (40u) is from spec v1.5.3 (finding F-03,
   B3.2.1) for a barcode-side branch reported ahead of the line sensors. */
#define LINE_BRANCH_EARLY_MM     (40u)   /* MEASURE: barcode sensor lead */
#define LINE_FOUND_MAX_MM        (255u)
#define LINE_BARCODE_SIDE        (APP_SIDE_RIGHT) /* MEASURE: A2 */

/* Part C.4b: tie configuration constants to their stored field domains. */
#if (LINE_LOST_CYCLES > 31u)
#error "LINE_LOST_CYCLES no longer fits the 5-bit LOST_CNT field: widen it"
#endif

#if (LINE_FOUND_MAX_MM > 255u)
#error "LINE_FOUND_MAX_MM no longer fits the 8-bit CROSS_MM field: widen it"
#endif

#if (LINE_NORM_MAX > 1000u)
#error "LINE_NORM_MAX exceeds SB_LINE0_STEERING domain (-1000..1000)"
#endif

#if ((0u == LINE_CAL_MIN_SPAN) || (LINE_CAL_MIN_SPAN > 4095u))
#error "LINE_CAL_MIN_SPAN must fit within the 12-bit ADC range (1..4095)"
#endif

#if ((LINE_WHITE_THRESHOLD >= LINE_BLACK_THRESHOLD) ||                         \
     (LINE_BLACK_THRESHOLD > LINE_NORM_MAX) ||                                 \
     (LINE_SAT_THRESHOLD > LINE_NORM_MAX))
#error "Line thresholds must satisfy WHITE < BLACK <= NORM_MAX and SAT <= 1000"
#endif

#endif /* CFG_LINE_H */

/*** end of file ***/
