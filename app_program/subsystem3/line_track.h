/**
 * @file    line_track.h
 * @brief   Pure line normalisation, steering, and junction classification.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdint.h>
#include <stdbool.h>

/* Calibration endpoints and precomputed Q16 reciprocals (E8). */
typedef struct
{
    uint32_t white_l;
    uint32_t black_l;
    uint32_t white_r;
    uint32_t black_r;
    uint32_t recip_l_q16;
    uint32_t recip_r_q16;
} line_track_cal_t;

/* Junction state machine state. */
#define LINE_JCT_STATE_IDLE      (0u)
#define LINE_JCT_STATE_CANDIDATE (1u)
#define LINE_JCT_STATE_CONFIRM   (2u)

typedef struct
{
    uint32_t state;
    uint32_t mark_counts;
    bool     b_sat_l;
    bool     b_sat_r;
    uint32_t seq0;
} line_track_jct_t;

typedef struct
{
    uint32_t norm_l;
    uint32_t norm_r;
    uint32_t path_counts;
    uint32_t branch_seq;
    uint32_t min_counts;
    uint32_t confirm_counts;
    bool     b_branch_recent;
} line_track_jct_in_t;

extern bool     line_track_cal_finish(line_track_cal_t * p_cal,
                                      uint32_t min_l, uint32_t max_l,
                                      uint32_t min_r, uint32_t max_r);
extern uint32_t line_track_normalise(uint32_t raw, uint32_t white,
                                     uint32_t black, uint32_t recip_q16);
extern int32_t  line_track_steering(uint32_t norm_l, uint32_t norm_r);
extern void     line_track_jct_reset(line_track_jct_t * p_jct);
extern uint32_t line_track_jct_step(line_track_jct_t * p_jct,
                                    line_track_jct_in_t const * p_in);

#endif /* LINE_TRACK_H */

/*** end of file ***/
