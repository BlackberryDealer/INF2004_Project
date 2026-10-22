/**
 * @file    telem_bin.h
 * @brief   Binary MQTT payload builders (pure, host-safe): every payload
 *          is the MQ_HDR byte followed by its MQ_* layout group, written
 *          LSB-first with the app_bits stream writer (B1.4 step 2).
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

/* Every builder returns the payload length in bytes, or 0 if p_buf is
   too small. Values are clamped to their layout range, never wrapped. */

/* MQ_STATE (18 B). *pb_was_clamped (may be NULL) is set true when an RNG
   field was clamped; the caller then sets APP_FAULT_RANGE_CLAMP.       */
extern uint32_t telem_bin_state(telem_bin_state_t const * p_state,
                                uint8_t * p_buf, uint32_t size,
                                bool * pb_was_clamped);

/* MQ_EVT (3 B): the 16-bit app event word (MSG_EVT layout). */
extern uint32_t telem_bin_event(uint32_t evt_word, uint8_t * p_buf,
                                uint32_t size);

/* MQ_PROF (6 B); distances in mm, stored in 4 mm steps. */
extern uint32_t telem_bin_profile(telem_bin_profile_t const * p_prof,
                                  uint8_t * p_buf, uint32_t size);

/* MQ_SUMM (7 B). */
extern uint32_t telem_bin_summary(telem_bin_summary_t const * p_summ,
                                  uint8_t * p_buf, uint32_t size);

/* MQ_HB (6 B). */
extern uint32_t telem_bin_heartbeat(uint32_t uptime_s, uint32_t drops,
                                    uint32_t lat_max_us, uint8_t * p_buf,
                                    uint32_t size);

/* Byte 0 of a payload: APP_MQ_VERSION | (mq_type << 3). Also the whole
   1-byte availability payload (APP_MQ_TYPE_ONLINE / OFFLINE).          */
extern uint32_t telem_bin_header(uint32_t mq_type);

#endif /* TELEM_BIN_H */

/*** end of file ***/
