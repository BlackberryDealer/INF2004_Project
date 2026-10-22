/**
 * @file    cfg_line.h
 * @brief   Configuration constants and compile-time checks for line sensing.
 * @owner   Buddy 3 (line, barcode)
 * @kind    cfg
 */

#ifndef CFG_LINE_H
#define CFG_LINE_H

#include "../app_types.h"

#define LINE_BLACK_IS_HIGH       (true)  /* MEASURE: A1 */
#define LINE_CAL_MIN_SPAN        (400u)  /* ADC codes; below => cal fails */
#define LINE_NORM_MAX            (1000u)
#define LINE_WHITE_THRESHOLD     (300u)  /* normalised; below = white   */
#define LINE_BLACK_THRESHOLD     (700u)  /* normalised; above = black   */
#define LINE_SAT_THRESHOLD       (900u)  /* channel fully over black    */
#define LINE_JCT_SUM_MIN         (1400u) /* TUNE: nL + nR on a junction */
#define LINE_LOST_CYCLES         (20u)   /* 200 ms, NFR4 definition     */
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
