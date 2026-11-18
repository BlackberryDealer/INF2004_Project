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
#include <stdbool.h>
#include "../app_layout.h"

extern uint32_t odometry_hw_to_counts(uint32_t hw_image); /* edges / 4 */
extern uint32_t odometry_counts_to_mm(uint32_t counts);
extern uint32_t odometry_mm_to_counts(uint32_t distance_mm);
extern int32_t  odometry_cpw_to_mm_s(int32_t speed_cpw);
extern int32_t  odometry_mm_s_to_cpw(int32_t speed_mm_s);
extern uint32_t odometry_turn_counts(uint32_t angle_deg); /* per wheel  */
extern int32_t  odometry_heading_cdeg(int32_t pos_l_counts,
                                      int32_t pos_r_counts); /* + = right */

/* ---- Path counter (SB_MOT3 PATH, 17 bits, kind RAW) -----------------
 * motion_task adds the travel of both wheels. The track has no line end,
 * so the counter wraps at 2^17 counts (about 42.5 m) instead of stopping:
 * a stopped counter would freeze every "travelled since" rule. Distances
 * are taken modulo 2^17 and are right while the stretch is shorter than
 * ODOMETRY_PATH_AGE_MAX (2^16 counts, about 21 m). Every rule measures a
 * stretch far shorter than that; a mark that may be kept for longer is
 * aged with odometry_path_age_cap() every cycle, so its stretch stays at
 * the cap instead of wrapping back to a small value.                   */
#define ODOMETRY_PATH_MASK     ((uint32_t) SB_MOT3_PATH_MAX)
#define ODOMETRY_PATH_AGE_MAX  ((ODOMETRY_PATH_MASK + 1u) >> 1u)

#if (SB_MOT3_PATH_MAX != ((1 << SB_MOT3_PATH_BITS) - 1))
#error "PATH must use every code of its bits so that it can wrap"
#endif

/**
 * odometry_path_since
 * @brief   Path travelled from a mark to now, modulo 2^17 counts.
 * @param   path_now   current path, counts
 * @param   path_mark  path when the stretch began, counts
 * @return  counts travelled (right below ODOMETRY_PATH_AGE_MAX)
 * @context any
 */
static inline uint32_t odometry_path_since(uint32_t path_now,
                                           uint32_t path_mark)
{
    return ((path_now - path_mark) & ODOMETRY_PATH_MASK);
}

/**
 * odometry_path_add
 * @brief   The path a given distance ahead of a path value (wraps).
 * @param   path    path, counts
 * @param   counts  distance ahead, counts
 * @return  path + counts modulo 2^17
 * @context any
 */
static inline uint32_t odometry_path_add(uint32_t path, uint32_t counts)
{
    return ((path + counts) & ODOMETRY_PATH_MASK);
}

/**
 * odometry_path_sub
 * @brief   The path a given distance behind a path value (wraps).
 * @param   path    path, counts
 * @param   counts  distance behind, counts
 * @return  path - counts modulo 2^17
 * @context any
 */
static inline uint32_t odometry_path_sub(uint32_t path, uint32_t counts)
{
    return ((path - counts) & ODOMETRY_PATH_MASK);
}

/**
 * odometry_path_reached
 * @brief   True once the path has reached a target set at most
 *          ODOMETRY_PATH_AGE_MAX ahead with odometry_path_add(). Only for
 *          targets inside a stage that ends within that distance.
 * @param   path_now     current path, counts
 * @param   path_target  target path, counts
 * @return  true when path_now is at or past the target
 * @context any
 */
static inline bool odometry_path_reached(uint32_t path_now,
                                         uint32_t path_target)
{
    return (odometry_path_since(path_now, path_target)
            < ODOMETRY_PATH_AGE_MAX);
}

/**
 * odometry_path_age_cap
 * @brief   Keeps a long-held mark at most ODOMETRY_PATH_AGE_MAX behind the
 *          current path. Called every cycle for such a mark, its stretch
 *          reads as at least that cap for as long as the mark is kept.
 * @param   path_now   current path, counts
 * @param   path_mark  stored mark, counts
 * @return  the mark to store
 * @context any
 */
static inline uint32_t odometry_path_age_cap(uint32_t path_now,
                                             uint32_t path_mark)
{
    return (odometry_path_since(path_now, path_mark) > ODOMETRY_PATH_AGE_MAX)
           ? odometry_path_sub(path_now, ODOMETRY_PATH_AGE_MAX)
           : path_mark;
}

/* ---- Speed window (B2.2.1; used by motion_task only) ----------------- */

#define ODOMETRY_WINDOW_SLOTS  (5u)   /* 5 x 10 ms ticks = 50 ms window */

/* Packed (layout groups PRV_WIN_L, PRV_WIN_R, PRV_WIN_T, PRV_WIN_I): one
   byte per slot value. A delta is stored as counts + PRV_WIN_L_DELTA_OFFSET
   (7 bits), so no signed value is ever shifted or masked.              */
typedef struct
{
    uint8_t delta_l[ODOMETRY_WINDOW_SLOTS];  /* counts per tick + 64    */
    uint8_t delta_r[ODOMETRY_WINDOW_SLOTS];
    uint8_t dt_ms[ODOMETRY_WINDOW_SLOTS];    /* tick length, 0..63 ms   */
    uint8_t idx;                             /* next slot, 0..4         */
} odometry_window_t;

#if (PRV_WIN_L_DELTA_OFFSET != PRV_WIN_R_DELTA_OFFSET) \
    || (PRV_WIN_L_DELTA_MAX != PRV_WIN_R_DELTA_MAX)
#error "Both window delta rows must have the same layout"
#endif

/* Window with no movement: every delta slot holds the offset (= 0). */
#define ODOMETRY_WINDOW_SLOT0   ((uint8_t) PRV_WIN_L_DELTA_OFFSET)
#define ODOMETRY_WINDOW_EMPTY                                            \
    {                                                                    \
        {ODOMETRY_WINDOW_SLOT0, ODOMETRY_WINDOW_SLOT0,                   \
         ODOMETRY_WINDOW_SLOT0, ODOMETRY_WINDOW_SLOT0,                   \
         ODOMETRY_WINDOW_SLOT0},                                         \
        {ODOMETRY_WINDOW_SLOT0, ODOMETRY_WINDOW_SLOT0,                   \
         ODOMETRY_WINDOW_SLOT0, ODOMETRY_WINDOW_SLOT0,                   \
         ODOMETRY_WINDOW_SLOT0},                                         \
        {0u, 0u, 0u, 0u, 0u},                                            \
        0u                                                               \
    }

extern void odometry_window_reset(odometry_window_t * p_win);
extern void odometry_window_push(odometry_window_t * p_win, int32_t delta_l,
                                 int32_t delta_r, uint32_t dt_ms);
extern void odometry_window_speed(odometry_window_t const * p_win,
                                  int32_t * p_speed_l_cpw,
                                  int32_t * p_speed_r_cpw);

#endif /* ODOMETRY_H */

/*** end of file ***/
