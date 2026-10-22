/**
 * @file    odometry.h
 * @brief   Pure conversions; any module may include it. Plus the 50 ms
 *          speed window used by motion_task (pure so it can be host-tested).
 * @owner   Buddy 2 (motion)
 * @kind    algo
 *
 * The six conversions are the frozen contract (Part D.6). The window type
 * and its three functions below them are private to motion (B2.2.1).
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

extern uint32_t odometry_counts_to_mm(uint32_t counts);
extern uint32_t odometry_mm_to_counts(uint32_t distance_mm);
extern int32_t  odometry_cpw_to_mm_s(int32_t speed_cpw);
extern int32_t  odometry_mm_s_to_cpw(int32_t speed_mm_s);
extern uint32_t odometry_turn_counts(uint32_t angle_deg); /* per wheel  */
extern int32_t  odometry_heading_cdeg(int32_t pos_l_counts,
                                      int32_t pos_r_counts); /* + = right */

/* ---- Speed window (B2.2.1; used by motion_task only) ----------------- */

#define ODOMETRY_WINDOW_SLOTS  (5u)   /* 5 x 10 ms ticks = 50 ms window */

typedef struct            /* Phase 2 (baseline) layout */
{
    int32_t  delta_l[ODOMETRY_WINDOW_SLOTS];  /* counts per tick, +-63 */
    int32_t  delta_r[ODOMETRY_WINDOW_SLOTS];
    uint32_t dt_ms[ODOMETRY_WINDOW_SLOTS];    /* tick length, 0..63 ms */
    uint32_t idx;         /* next slot, 0..4 */
} odometry_window_t;

extern void odometry_window_reset(odometry_window_t * p_win);
extern void odometry_window_push(odometry_window_t * p_win, int32_t delta_l,
                                 int32_t delta_r, uint32_t dt_ms);
extern void odometry_window_speed(odometry_window_t const * p_win,
                                  int32_t * p_speed_l_cpw,
                                  int32_t * p_speed_r_cpw);

#endif /* ODOMETRY_H */

/*** end of file ***/
