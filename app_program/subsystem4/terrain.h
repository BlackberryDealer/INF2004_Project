/**
 * @file    terrain.h
 * @brief   Pure Terrain Monitor: integer trig, hump detection, motion
 *          classification, and the turn-rate and forward-acceleration
 *          helpers that must stay laptop-testable (S4-U03, U07, U12).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    algo
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdint.h>
#include <stdbool.h>

/* Terrain Monitor states (Figure 6.3) */
#define TERRAIN_CAL        (0u)
#define TERRAIN_FLAT       (1u)
#define TERRAIN_CLIMBING   (2u)
#define TERRAIN_DESCENDING (3u)
#define TERRAIN_FAULT      (4u)

/* ---- integer trigonometry ------------------------------------------- */

/* Absolute value as unsigned (shared by the classifier and turn-rate
   checks; no stdlib in the firmware). */
extern uint32_t terrain_abs(int32_t value);

extern int32_t terrain_atan2_ddeg(int32_t y, int32_t x); /* -1800..1799 */
extern int32_t terrain_sin_q14(int32_t ddeg);            /* any angle  */
extern int32_t terrain_cos_q14(int32_t ddeg);

/* ---- turn-rate maths ------------------------------------------------- */

/* Wraps a heading in 0.1 deg into 0..3599. */
extern int32_t terrain_heading_wrap_ddeg(int32_t heading);

/* Shortest signed difference prev -> now in 0.1 deg: -1800..1799. */
extern int32_t terrain_heading_delta_ddeg(uint32_t prev_wrap,
                                          uint32_t now_wrap);

/* ---- forward acceleration from the speed history (S4-U12) ------------ */

typedef struct
{
    int32_t  speed[5]; /* cpw, one per cycle */
    uint32_t idx;      /* next slot, 0..4   */
    uint32_t count;    /* pushes so far, 0..5 */
} terrain_speed_hist_t;

extern void terrain_speed_hist_reset(terrain_speed_hist_t * p_hist);
/* Pushes one 50 ms-window speed and returns the forward acceleration in
   mg (0 until the window has filled once).                              */
extern int32_t terrain_speed_hist_push(terrain_speed_hist_t * p_hist,
                                       int32_t                speed_cpw);

/* ---- per-cycle travel used by the Terrain Monitor -------------------- */

/* The slower wheel's delta in counts. A rebase or a glitch
   (|delta| > 4 x APP_TICK_COUNTS_MAX) returns 0 so the run-start rebase
   can never become a false climb (S4-U07, S4-U13).                      */
extern uint32_t terrain_ds_counts(int32_t delta_pos_l, int32_t delta_pos_r);

/* ---- Terrain Monitor (Figure 6.3) ------------------------------------ */

typedef struct
{
    uint32_t state;          /* TERRAIN_* */
    uint32_t hyst;           /* consecutive samples meeting the condition */
    int32_t  acc_um_mg;      /* E2: sum of ds_um x (ax - a_fwd) in um x mg */
    uint32_t climb_counts;   /* distance since CLIMBING started */
    uint32_t peak_now_mm;    /* highest point of this hump */
    int32_t  pitch_max_ddeg; /* for the geometric cross-check */
} terrain_t;

typedef struct
{
    int32_t  ax_mg; /* forward, calibrated, sign applied */
    int32_t  az_mg;
    int32_t  a_fwd_mg;  /* robot's own acceleration (from encoders) */
    uint32_t ds_counts; /* this cycle, slower wheel */
} terrain_input_t;

typedef struct
{
    int32_t  pitch_ddeg; /* corrected pitch */
    uint32_t hump_now_mm;
    uint32_t hump_done_mm; /* >0 only on the cycle a hump ends */
    uint32_t hump_geo_mm;  /* geometric estimate on that cycle */
} terrain_output_t;

extern void terrain_reset(terrain_t * p_ter);
extern void terrain_step(terrain_t * p_ter, terrain_input_t const * p_in,
                         terrain_output_t * p_out);

/* ---- motion classifier (FR18) ---------------------------------------- */

extern uint32_t terrain_classify(uint32_t terrain_state, bool b_impact_hold,
                                 int32_t turn_dps, int32_t a_fwd_mg,
                                 int32_t speed_cpw);

#endif /* TERRAIN_H */

/*** end of file ***/
