/**
 * @file    telem_bin.h
 * @brief   Binary payload builders (Phase 4). Pure, host-safe. The Phase 0
 *          skeleton holds only the two contract structs (D.9).
 * @owner   Buddy 1 (communications)
 * @kind    algo
 */

#ifndef TELEM_BIN_H
#define TELEM_BIN_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t angle_step;       /* 0..24 => 30 + 5*step deg */
    uint32_t width_mm;         /* 0..1020 (4 mm steps; 1020 = clear) */
    uint32_t nearest_mm;
    uint32_t clear_left_mm;
    uint32_t clear_right_mm;
    uint32_t side;             /* APP_SIDE_* */
} telem_bin_profile_t;

typedef struct
{
    uint32_t end_reason;       /* APP_END_* */
    uint32_t hump_peak_mm;     /* 0..127 */
    uint32_t odometry_mm;      /* 0..32767 */
    uint32_t faults;           /* APP_FAULT_* mask, 10 bits */
    uint32_t run_time_s;       /* 0..1023 */
} telem_bin_summary_t;

#endif /* TELEM_BIN_H */

/*** end of file ***/
