/**
 * @file    telem_bin.h
 * @brief   Telemetry input structs (pure, host-safe): the two contract
 *          structs (D.9) and the state snapshot telem_task builds.
 *          The binary payload builders follow in Phase 4.
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

/* One state sample, copied by telem_task from the status board getters
   and fault_get(). Natural units, as the board returns them.          */
typedef struct
{
    uint32_t sys_state;        /* APP_SYS_* */
    uint32_t run_state;        /* APP_RUN_* */
    uint32_t motion_state;     /* APP_MOT_* */
    int32_t  speed_l_cpw;      /* -127..+127 counts per 50 ms window */
    int32_t  speed_r_cpw;
    int32_t  pos_l_counts;     /* -131071..+131071 */
    int32_t  pos_r_counts;
    uint32_t line_status;      /* APP_LINE_* */
    int32_t  steering;         /* -1000..+1000 permille */
    uint32_t path_counts;      /* 0..131071, sum of |travel| */
    int32_t  pitch_ddeg;       /* -450..+450 (0.1 deg) */
    uint32_t hump_peak_mm;     /* 0..127 */
    int32_t  turn_rate_dps;    /* -360..+360 */
    uint32_t motion_class;     /* APP_MCLASS_* */
    uint32_t faults;           /* APP_FAULT_* mask, 10 bits */
} telem_bin_state_t;

#endif /* TELEM_BIN_H */

/*** end of file ***/
