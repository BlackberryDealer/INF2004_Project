/**
 * @file    terrain.c
 * @brief   Pure Terrain Monitor: integer trig tables and interpolation,
 *          the Figure 6.3 state machine, the motion classifier and the
 *          turn-rate / forward-acceleration helpers.
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    algo
 * @reqs    FR4, FR5, FR18, NFR7
 *
 * Pure module (Part A rule 8): compiles on a laptop with
 * gcc -std=c99 and never includes app.h or the tk, tm, bsp, hardware
 * or pico headers. It may include the pure odometry module (D.6) for
 * the mm-to-counts conversion of the ramp limit.
 *
 * Phase 4 (packed): the monitor and speed-history fields are stored in
 * the PRV_IMU_* / PRV_HIST_* words generated from tools/layout.csv via
 * app_bits. The baseline build stored the same information in natural
 * types; only the storage changed, and the host tests must pass on both.
 * Clamps inside this module are silent by design: the only overflow that
 * matters to the mission (a hump peak reaching 127 mm, assumption A25)
 * is reported by imu.c when it packs the run peak and posts the event,
 * because a pure module cannot call fault_set.
 */

#include "terrain.h" /* own header first (BARR-C 4.3.c) */
#include <stddef.h>  /* NULL for the clamp-flag arguments */
#include "cfg_imu.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem2/odometry.h"

/* ---- private constants and types ------------------------------------ */

/* Rebase/glitch guard for terrain_ds_counts: a plausible per-cycle delta
   is at most APP_TICK_COUNTS_MAX counts. */
#define TERRAIN_DS_DELTA_MAX (4u * APP_TICK_COUNTS_MAX)

#define TERRAIN_DDEG_CIRCLE    (3600)
#define TERRAIN_DDEG_HALF      (1800)
#define TERRAIN_DDEG_QUADRANT  (900)
#define TERRAIN_DDEG_PER_DEG   (10)
#define TERRAIN_DEG_QUADRANT   (90)
#define TERRAIN_DEG_HALF       (180)
#define TERRAIN_DEG_THREE_Q    (270)
#define TERRAIN_DEG_CIRCLE     (360)
#define TERRAIN_Q14_ONE        (16384)
#define TERRAIN_ATAN_K_MAX     (32)
#define TERRAIN_ATAN_SCALE     (1024)
#define TERRAIN_ATAN_FRAC_MASK (31u)
#define TERRAIN_HIST_ENTRIES   (5u)

/* Height conversion: acc is ds_um x (ax - a_fwd) summed over the hump.
   h_mm = acc / 1 000 000 approx: (acc >> 10) x 1074 >> 20 = acc x
   1074 / 2^30 = acc / 999 759. Practical bound: a 127 mm peak keeps
   acc below 1.27e8, so acc >> 10 stays below 2^17 and the x 1074
   product cannot overflow (comment per Part B rule R19: never >> a
   signed value; here everything is unsigned).                          */
#define TERRAIN_ACC_SHIFT_1 (10u)
#define TERRAIN_ACC_MUL     (1074u)
#define TERRAIN_ACC_SHIFT_2 (20u)

/* ---- private data ---------------------------------------------------- */

/* atan table, 0.1 deg, atan(k/32) for k = 0..32 (B4.2.1) */
static int32_t const g_atan_tab[TERRAIN_ATAN_K_MAX + 1u] = {
    0,   18,  36,  54,  71,  89,  106, 123, 140, 157, 174,
    190, 206, 221, 236, 251, 266, 280, 294, 307, 320, 333,
    345, 357, 369, 380, 391, 402, 412, 422, 432, 441, 450};

/* sin table, Q14, sin(d) for d = 0..90 deg (B4.2.1, exact list) */
static int32_t const g_sin_tab[91] = {
    0,     286,   572,   857,   1143,  1428,  1713,  1997,  2280,  2563,  2845,
    3126,  3406,  3686,  3964,  4240,  4516,  4790,  5063,  5334,  5604,  5872,
    6138,  6402,  6664,  6924,  7182,  7438,  7692,  7943,  8192,  8438,  8682,
    8923,  9162,  9397,  9630,  9860,  10087, 10311, 10531, 10749, 10963, 11174,
    11381, 11585, 11786, 11982, 12176, 12365, 12551, 12733, 12911, 13085, 13255,
    13421, 13583, 13741, 13894, 14044, 14189, 14330, 14466, 14598, 14726, 14849,
    14968, 15082, 15191, 15296, 15396, 15491, 15582, 15668, 15749, 15826, 15897,
    15964, 16026, 16083, 16135, 16182, 16225, 16262, 16294, 16322, 16344, 16362,
    16374, 16382, 16384};

/* ---- private function prototypes ------------------------------------- */

static uint32_t terrain_state_get(terrain_t const * p_ter);
static void     terrain_state_set(terrain_t * p_ter, uint32_t state);
static uint32_t terrain_hyst_get(terrain_t const * p_ter);
static void     terrain_hyst_set(terrain_t * p_ter, uint32_t hyst);
static uint32_t terrain_climb_get(terrain_t const * p_ter);
static void     terrain_climb_set(terrain_t * p_ter, uint32_t counts);
static uint32_t terrain_peak_get(terrain_t const * p_ter);
static void     terrain_peak_set(terrain_t * p_ter, uint32_t peak_mm);
static int32_t  terrain_pitch_max_get(terrain_t const * p_ter);
static void     terrain_pitch_max_set(terrain_t * p_ter, int32_t pitch);
static uint32_t terrain_hist_idx_get(terrain_speed_hist_t const * p_hist);
static void terrain_hist_idx_set(terrain_speed_hist_t * p_hist, uint32_t idx);
static int32_t  terrain_hist_get(terrain_speed_hist_t const * p_hist,
                                 uint32_t                     slot);
static void     terrain_hist_put(terrain_speed_hist_t * p_hist, uint32_t slot,
                                 int32_t speed_cpw);
static uint32_t terrain_acc_to_mm(int32_t acc_um_mg);
static uint32_t terrain_climb_limit_counts(void);
static uint32_t terrain_sin_q14_deg(uint32_t deg, uint32_t frac_ddeg);
static uint32_t terrain_hump_report(uint32_t integ_mm, uint32_t geo_mm);
static void     terrain_flat_step(terrain_t * p_ter, int32_t pitch_ddeg);
static void terrain_climb_step(terrain_t * p_ter, terrain_input_t const * p_in,
                               int32_t pitch_ddeg, int32_t delta_a,
                               uint32_t ds_um, terrain_output_t * p_out);
static void terrain_descend_step(terrain_t * p_ter, int32_t pitch_ddeg,
                                 int32_t delta_a, uint32_t ds_um,
                                 terrain_output_t * p_out);
static void terrain_hyst_bump(terrain_t * p_ter);
static bool terrain_hyst_done(terrain_t const * p_ter);

/* ---- public functions ------------------------------------------------- */

/**
 * terrain_abs
 * @brief   Absolute value as unsigned (no stdlib in the firmware).
 * @param   value  signed value
 * @return  magnitude
 * @context any
 */
uint32_t terrain_abs(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(0 - value); /* cast: value > INT32_MIN here */
    }
    return (uint32_t)value; /* cast: value >= 0 */
}

/**
 * terrain_atan2_ddeg
 * @brief   Integer atan2 in 0.1 deg, -1800..1799, error <= 0.2 deg.
 *          Works on magnitudes, then maps back by quadrant. One division,
 *          allowed: called at most twice per 10 ms cycle.
 * @param   y_value  signed value (the spec snippet names it y)
 * @param   x_value  signed value (the spec snippet names it x)
 * @return  angle in 0.1 deg
 * @context any (imu_task, host test)
 */
int32_t terrain_atan2_ddeg(int32_t y_value, int32_t x_value)
{
    uint32_t mag_y   = terrain_abs(y_value);
    uint32_t mag_x   = terrain_abs(x_value);
    bool     b_swap  = (mag_y > mag_x);
    uint32_t small   = 0u;
    uint32_t big     = 0u;
    uint32_t ratio   = 0u;
    uint32_t tab_idx = 0u;
    uint32_t frac    = 0u;
    int32_t  angle   = 0;

    if ((0u == mag_y) && (0u == mag_x))
    {
        return 0; /* undefined angle: report 0 */
    }

    if (b_swap)
    {
        small = mag_x;
        big   = mag_y;
    }
    else
    {
        small = mag_y;
        big   = mag_x;
    }

    ratio = (small * (uint32_t)TERRAIN_ATAN_SCALE) / big;
    /* 0..1024; the one division */
    tab_idx = ratio >> 5; /* 0..32 */
    frac    = ratio & TERRAIN_ATAN_FRAC_MASK;
    if (TERRAIN_ATAN_K_MAX == tab_idx)
    {
        angle = g_atan_tab[TERRAIN_ATAN_K_MAX];
    }
    else
    {
        angle = g_atan_tab[tab_idx] +
                (((g_atan_tab[tab_idx + 1u] - g_atan_tab[tab_idx]) *
                  (int32_t)frac) /
                 TERRAIN_ATAN_K_MAX);
        /* signed division by 32, not >>:
           C99 truncates toward zero */
    }
    if (b_swap)
    {
        angle = TERRAIN_DDEG_QUADRANT - angle;
        /* complement: angle above 45 deg */
    }

    if ((x_value < 0) && (y_value >= 0))
    {
        return TERRAIN_DDEG_HALF - angle; /* second quadrant: 180 - a */
    }
    if ((x_value < 0) && (y_value < 0))
    {
        return angle - TERRAIN_DDEG_HALF; /* third quadrant: a - 180 */
    }
    if ((x_value >= 0) && (y_value < 0))
    {
        return -angle;
    }
    return angle; /* x >= 0, y >= 0 */
}

/**
 * terrain_sin_q14
 * @brief   Sine of any angle in 0.1 deg, Q14, linear interpolation
 *          between whole degrees (division by 10, event-time).
 * @param   ddeg  angle in 0.1 deg
 * @return  Q14 sine, -16384..16384
 * @context any
 */
int32_t terrain_sin_q14(int32_t ddeg)
{
    uint32_t deg       = 0u;
    uint32_t frac      = 0u;
    uint32_t q14_value = 0u;
    bool     b_is_neg  = false;

    ddeg = ddeg % TERRAIN_DDEG_CIRCLE;
    if (ddeg < 0)
    {
        ddeg += TERRAIN_DDEG_CIRCLE; /* remainder is > -3600: one add */
    }

    deg  = (uint32_t)ddeg / TERRAIN_DDEG_PER_DEG; /* cast: ddeg >= 0 */
    frac = (uint32_t)ddeg % TERRAIN_DDEG_PER_DEG;

    if (deg < TERRAIN_DEG_QUADRANT)
    {
        q14_value = terrain_sin_q14_deg(deg, frac);
    }
    else if (deg < TERRAIN_DEG_HALF)
    {
        q14_value = terrain_sin_q14_deg(TERRAIN_DEG_HALF - deg, frac);
    }
    else if (deg < TERRAIN_DEG_THREE_Q)
    {
        b_is_neg  = true;
        q14_value = terrain_sin_q14_deg(deg - TERRAIN_DEG_HALF, frac);
    }
    else
    {
        b_is_neg  = true;
        q14_value = terrain_sin_q14_deg(TERRAIN_DEG_CIRCLE - deg, frac);
    }

    if (b_is_neg)
    {
        return (int32_t)(0u - q14_value); /* cast: value <= 16384 */
    }
    return (int32_t)q14_value; /* cast: value <= 16384 */
}

/**
 * terrain_cos_q14
 * @brief   Cosine via sin(ddeg + 90 deg).
 * @param   ddeg  angle in 0.1 deg
 * @return  Q14 cosine, -16384..16384
 * @context any
 */
int32_t terrain_cos_q14(int32_t ddeg)
{
    return terrain_sin_q14(ddeg + TERRAIN_DDEG_QUADRANT);
}

/**
 * terrain_heading_wrap_ddeg
 * @brief   Wraps a heading into 0..3599 (0.1 deg).
 * @param   heading  any heading in 0.1 deg
 * @return  wrapped heading 0..3599
 * @context any
 */
int32_t terrain_heading_wrap_ddeg(int32_t heading)
{
    heading = heading % TERRAIN_DDEG_CIRCLE;
    if (heading < 0)
    {
        heading += TERRAIN_DDEG_CIRCLE;
    }
    return heading;
}

/**
 * terrain_heading_delta_ddeg
 * @brief   Shortest signed difference between two wrapped headings.
 * @param   prev_wrap  0..3599
 * @param   now_wrap   0..3599
 * @return  delta in 0.1 deg, -1800..1799 (+ = right)
 * @context any
 */
int32_t terrain_heading_delta_ddeg(uint32_t prev_wrap, uint32_t now_wrap)
{
    int32_t delta = (int32_t)now_wrap - (int32_t)prev_wrap;
    /* casts: both below 3600 */
    if (delta > TERRAIN_DDEG_HALF)
    {
        delta -= TERRAIN_DDEG_CIRCLE;
    }
    if (delta < (0 - TERRAIN_DDEG_HALF))
    {
        delta += TERRAIN_DDEG_CIRCLE;
    }
    return delta;
}

/**
 * terrain_speed_hist_reset
 * @brief   Empties the 5-entry speed history (run start): every entry
 *          reads 0 cpw and the index returns to slot 0.
 * @param   p_hist  history
 * @return  none
 * @context any
 */
void terrain_speed_hist_reset(terrain_speed_hist_t * p_hist)
{
    /* Encoded zero: the PRV_HIST_* rows store speed as value + 128, so
       a zero word would decode as -128. Each slot gets encode(0) = 128,
       and the index returns to slot 0. */
    p_hist->word_a = 0x80808080u;
    p_hist->word_b = 0x00000080u;
}

/**
 * terrain_speed_hist_push
 * @brief   Pushes one cycle's speed (cpw) and returns the forward
 *          acceleration: the difference across the FULL 50 ms window
 *          (never 10 ms), scaled by TERRAIN_AFWD_MG_PER_CPW_Q10 / 1024.
 *          A fresh history is zero-filled by reset, so the first pushes
 *          measure against standstill.
 * @param   p_hist     history
 * @param   speed_cpw  this cycle's speed, -127..127
 * @return  a_fwd in mg
 * @context any
 */
int32_t terrain_speed_hist_push(terrain_speed_hist_t * p_hist,
                                int32_t                speed_cpw)
{
    uint32_t idx       = terrain_hist_idx_get(p_hist);
    int32_t  speed_old = terrain_hist_get(p_hist, idx);

    terrain_hist_put(p_hist, idx, speed_cpw);
    idx = (idx + 1u) % TERRAIN_HIST_ENTRIES;
    terrain_hist_idx_set(p_hist, idx);

    /* The slot we just overwrote is exactly 50 ms old. The gain is
       14576 (< 2^31) and the speed difference is at most 254, so the
       product stays far below 2^31. */
    return ((speed_cpw - speed_old) * (int32_t)TERRAIN_AFWD_MG_PER_CPW_Q10) /
           1024; /* signed division by a power of two,
                    never >>: C99 truncates toward zero */
}

/**
 * terrain_ds_counts
 * @brief   This cycle's travel for the Terrain Monitor: the slower
 *          wheel's delta. A rebase or glitch (either delta beyond
 *          4 x APP_TICK_COUNTS_MAX) gives 0 so the run-start rebase
 *          cannot become a false climb.
 * @param   delta_pos_l  left position delta, counts
 * @param   delta_pos_r  right position delta, counts
 * @return  counts, 0..4 x APP_TICK_COUNTS_MAX
 * @context any
 */
uint32_t terrain_ds_counts(int32_t delta_pos_l, int32_t delta_pos_r)
{
    uint32_t mag_l = terrain_abs(delta_pos_l);
    uint32_t mag_r = terrain_abs(delta_pos_r);

    if ((mag_l > TERRAIN_DS_DELTA_MAX) || (mag_r > TERRAIN_DS_DELTA_MAX))
    {
        return 0u; /* odometry was rebased, or a glitch */
    }
    if (mag_l < mag_r)
    {
        return mag_l;
    }
    return mag_r;
}

/**
 * terrain_reset
 * @brief   Returns the monitor to FLAT with all accumulators cleared.
 * @param   p_ter  monitor state
 * @return  none
 * @context any
 */
void terrain_reset(terrain_t * p_ter)
{
    p_ter->word_state = 0u;
    p_ter->word_prog  = 0u;
    p_ter->word_pitch = 0u;
    p_ter->acc_um_mg  = 0;
    terrain_state_set(p_ter, TERRAIN_FLAT);
    terrain_pitch_max_set(p_ter, 0); /* encoded zero: value + 512 */
}

/**
 * terrain_step
 * @brief   One 10 ms cycle of the Terrain Monitor (Figure 6.3 with the
 *          review changes). pitch = atan2(ax - a_fwd, az), + = nose up.
 *          Distance, not time, decides "ramp": the climb limit is
 *          TERRAIN_MAX_CLIMB_MM converted to counts. One handler per
 *          state, as B4.2.1 requires.
 * @param   p_ter  monitor state
 * @param   p_in   this cycle's inputs
 * @param   p_out  outputs: pitch, hump now, hump done (one cycle), geo
 * @return  none
 * @context imu_task, host test
 */
void terrain_step(terrain_t * p_ter, terrain_input_t const * p_in,
                  terrain_output_t * p_out)
{
    int32_t pitch_ddeg =
        terrain_atan2_ddeg(p_in->ax_mg - p_in->a_fwd_mg, p_in->az_mg);
    int32_t  delta_a = p_in->ax_mg - p_in->a_fwd_mg;
    uint32_t ds_um   = p_in->ds_counts * APP_UM_PER_COUNT;

    p_out->pitch_ddeg   = pitch_ddeg;
    p_out->hump_now_mm  = terrain_acc_to_mm(p_ter->acc_um_mg);
    p_out->hump_done_mm = 0u;
    p_out->hump_geo_mm  = 0u;

    switch (terrain_state_get(p_ter))
    {
    case TERRAIN_FLAT:
        terrain_flat_step(p_ter, pitch_ddeg);
        break;

    case TERRAIN_CLIMBING:
        terrain_climb_step(p_ter, p_in, pitch_ddeg, delta_a, ds_um, p_out);
        break;

    case TERRAIN_DESCENDING:
        terrain_descend_step(p_ter, pitch_ddeg, delta_a, ds_um, p_out);
        break;

    case TERRAIN_CAL:
    case TERRAIN_FAULT:
    default:
        /* imu.c drives these states; the outputs stay zero here. */
        break;
    }
}

/**
 * terrain_classify
 * @brief   Motion class, first match wins: IMPACT, CLIMBING, DESCENDING,
 *          TURNING, ACCELERATING, STATIONARY, else CRUISING.
 * @param   terrain_state  TERRAIN_*
 * @param   b_impact_hold  impact class hold active
 * @param   turn_dps       turn rate, deg/s
 * @param   a_fwd_mg       forward acceleration, mg
 * @param   speed_cpw      speed, counts per 50 ms window
 * @return  APP_MCLASS_*
 * @context imu_task, host test
 */
uint32_t terrain_classify(uint32_t terrain_state, bool b_impact_hold,
                          int32_t turn_dps, int32_t a_fwd_mg, int32_t speed_cpw)
{
    if (b_impact_hold)
    {
        return APP_MCLASS_IMPACT;
    }
    if (TERRAIN_CLIMBING == terrain_state)
    {
        return APP_MCLASS_CLIMBING;
    }
    if (TERRAIN_DESCENDING == terrain_state)
    {
        return APP_MCLASS_DESCENDING;
    }
    if (terrain_abs(turn_dps) >= (uint32_t)IMU_TURNING_DPS)
    {
        return APP_MCLASS_TURNING;
    }
    if (terrain_abs(a_fwd_mg) >= (uint32_t)IMU_ACCEL_MG)
    {
        return APP_MCLASS_ACCELERATING;
    }
    if (0 == speed_cpw)
    {
        return APP_MCLASS_STATIONARY;
    }
    return APP_MCLASS_CRUISING;
}

/* ---- private functions ------------------------------------------------ */

/**
 * terrain_state_get / _set
 * @brief   PRV_IMU_A STATE field (0..4, enum).
 * @context any
 */
static uint32_t terrain_state_get(terrain_t const * p_ter)
{
    return app_bits_get(p_ter->word_state, PRV_IMU_A_STATE_POS,
                        PRV_IMU_A_STATE_BITS);
}

static void terrain_state_set(terrain_t * p_ter, uint32_t state)
{
    p_ter->word_state = app_bits_put(p_ter->word_state, PRV_IMU_A_STATE_POS,
                                     PRV_IMU_A_STATE_BITS, state);
}

/**
 * terrain_hyst_get / _set
 * @brief   PRV_IMU_A HYST field (0..15, saturating by design).
 * @context any
 */
static uint32_t terrain_hyst_get(terrain_t const * p_ter)
{
    return app_bits_get(p_ter->word_state, PRV_IMU_A_HYST_POS,
                        PRV_IMU_A_HYST_BITS);
}

static void terrain_hyst_set(terrain_t * p_ter, uint32_t hyst)
{
    p_ter->word_state = app_bits_put(p_ter->word_state, PRV_IMU_A_HYST_POS,
                                     PRV_IMU_A_HYST_BITS, hyst);
}

/**
 * terrain_climb_get / _set
 * @brief   PRV_IMU_B CLIMB_CNT field (0..2047, saturating by design:
 *          the ramp check fires far below the top code).
 * @context any
 */
static uint32_t terrain_climb_get(terrain_t const * p_ter)
{
    return app_bits_get(p_ter->word_prog, PRV_IMU_B_CLIMB_CNT_POS,
                        PRV_IMU_B_CLIMB_CNT_BITS);
}

static void terrain_climb_set(terrain_t * p_ter, uint32_t counts)
{
    p_ter->word_prog = app_bits_put(p_ter->word_prog, PRV_IMU_B_CLIMB_CNT_POS,
                                    PRV_IMU_B_CLIMB_CNT_BITS, counts);
}

/**
 * terrain_peak_get / _set
 * @brief   PRV_IMU_B PEAK_NOW field (0..127). A value that would exceed
 *          127 clamps silently here: the A25 fault is raised by imu.c
 *          when it packs the run peak and posts the HUMP event (a pure
 *          module cannot call fault_set).
 * @context any
 */
static uint32_t terrain_peak_get(terrain_t const * p_ter)
{
    return app_bits_get(p_ter->word_prog, PRV_IMU_B_PEAK_NOW_POS,
                        PRV_IMU_B_PEAK_NOW_BITS);
}

static void terrain_peak_set(terrain_t * p_ter, uint32_t peak_mm)
{
    uint32_t raw = app_bits_encode((int32_t)peak_mm, PRV_IMU_B_PEAK_NOW_MIN,
                                   PRV_IMU_B_PEAK_NOW_MAX,
                                   PRV_IMU_B_PEAK_NOW_OFFSET, NULL);
    /* silent clamp: see above */

    p_ter->word_prog = app_bits_put(p_ter->word_prog, PRV_IMU_B_PEAK_NOW_POS,
                                    PRV_IMU_B_PEAK_NOW_BITS, raw);
}

/**
 * terrain_pitch_max_get / _set
 * @brief   PRV_IMU_C PITCH_MAX field (-450..450). Clamped silently:
 *          steeper pitches only feed the geometric cross-check on a
 *          hump that ends, and course humps stay far below 45 deg.
 * @context any
 */
static int32_t terrain_pitch_max_get(terrain_t const * p_ter)
{
    return app_bits_decode(app_bits_get(p_ter->word_pitch,
                                        PRV_IMU_C_PITCH_MAX_POS,
                                        PRV_IMU_C_PITCH_MAX_BITS),
                           PRV_IMU_C_PITCH_MAX_OFFSET);
}

static void terrain_pitch_max_set(terrain_t * p_ter, int32_t pitch)
{
    uint32_t raw =
        app_bits_encode(pitch, PRV_IMU_C_PITCH_MAX_MIN, PRV_IMU_C_PITCH_MAX_MAX,
                        PRV_IMU_C_PITCH_MAX_OFFSET, NULL);
    /* silent clamp: see above */

    p_ter->word_pitch = app_bits_put(p_ter->word_pitch, PRV_IMU_C_PITCH_MAX_POS,
                                     PRV_IMU_C_PITCH_MAX_BITS, raw);
}

/**
 * terrain_hist_idx_get / _set
 * @brief   PRV_IMU_B HIST_IDX field carried in the history word_b.
 * @context any
 */
static uint32_t terrain_hist_idx_get(terrain_speed_hist_t const * p_hist)
{
    return app_bits_get(p_hist->word_b, PRV_IMU_B_HIST_IDX_POS,
                        PRV_IMU_B_HIST_IDX_BITS);
}

static void terrain_hist_idx_set(terrain_speed_hist_t * p_hist, uint32_t idx)
{
    p_hist->word_b = app_bits_put(p_hist->word_b, PRV_IMU_B_HIST_IDX_POS,
                                  PRV_IMU_B_HIST_IDX_BITS, idx);
}

/**
 * terrain_hist_get / _put
 * @brief   One speed-history slot: H0..H3 in word_a, H4 in word_b,
 *          stored with the PRV_HIST_* offset encoding (-127..127 + 128).
 * @context any
 */
static int32_t terrain_hist_get(terrain_speed_hist_t const * p_hist,
                                uint32_t                     slot)
{
    uint32_t raw = 0u;

    switch (slot)
    {
    case 0u:
        raw =
            app_bits_get(p_hist->word_a, PRV_HIST_A_H0_POS, PRV_HIST_A_H0_BITS);
        break;
    case 1u:
        raw =
            app_bits_get(p_hist->word_a, PRV_HIST_A_H1_POS, PRV_HIST_A_H1_BITS);
        break;
    case 2u:
        raw =
            app_bits_get(p_hist->word_a, PRV_HIST_A_H2_POS, PRV_HIST_A_H2_BITS);
        break;
    case 3u:
        raw =
            app_bits_get(p_hist->word_a, PRV_HIST_A_H3_POS, PRV_HIST_A_H3_BITS);
        break;
    case 4u:
        raw =
            app_bits_get(p_hist->word_b, PRV_HIST_B_H4_POS, PRV_HIST_B_H4_BITS);
        break;
    default:
        raw = 0u; /* unreachable: slots are 0..4 */
        break;
    }

    return app_bits_decode(raw, PRV_HIST_A_H0_OFFSET);
}

static void terrain_hist_put(terrain_speed_hist_t * p_hist, uint32_t slot,
                             int32_t speed_cpw)
{
    uint32_t raw =
        app_bits_encode(speed_cpw, PRV_HIST_A_H0_MIN, PRV_HIST_A_H0_MAX,
                        PRV_HIST_A_H0_OFFSET, NULL);
    /* silent: the window speed is
       -127..127 by design */

    switch (slot)
    {
    case 0u:
        p_hist->word_a = app_bits_put(p_hist->word_a, PRV_HIST_A_H0_POS,
                                      PRV_HIST_A_H0_BITS, raw);
        break;
    case 1u:
        p_hist->word_a = app_bits_put(p_hist->word_a, PRV_HIST_A_H1_POS,
                                      PRV_HIST_A_H1_BITS, raw);
        break;
    case 2u:
        p_hist->word_a = app_bits_put(p_hist->word_a, PRV_HIST_A_H2_POS,
                                      PRV_HIST_A_H2_BITS, raw);
        break;
    case 3u:
        p_hist->word_a = app_bits_put(p_hist->word_a, PRV_HIST_A_H3_POS,
                                      PRV_HIST_A_H3_BITS, raw);
        break;
    case 4u:
        p_hist->word_b = app_bits_put(p_hist->word_b, PRV_HIST_B_H4_POS,
                                      PRV_HIST_B_H4_BITS, raw);
        break;
    default:
        break; /* unreachable: slots are 0..4 */
    }
}

/**
 * terrain_acc_to_mm
 * @brief   Integrated height: acc <= 0 gives 0, else the (acc >> 10)
 *          x 1074 >> 20 approximation of acc / 1 000 000 (all unsigned,
 *          no division).
 * @param   acc_um_mg  accumulated ds_um x (ax - a_fwd)
 * @return  height in mm
 * @context any
 */
static uint32_t terrain_acc_to_mm(int32_t acc_um_mg)
{
    uint32_t acc_u = 0u;

    if (acc_um_mg <= 0)
    {
        return 0u;
    }
    acc_u = (uint32_t)acc_um_mg; /* cast: > 0 here */
    return ((acc_u >> TERRAIN_ACC_SHIFT_1) * TERRAIN_ACC_MUL) >>
           TERRAIN_ACC_SHIFT_2;
}

/**
 * terrain_climb_limit_counts
 * @brief   TERRAIN_MAX_CLIMB_MM converted once with the shared odometry
 *          conversion. Event-time division, like the atan2 above.
 * @return  counts
 * @context imu_task (CLIMBING cycles), host test
 */
static uint32_t terrain_climb_limit_counts(void)
{
    return odometry_mm_to_counts(TERRAIN_MAX_CLIMB_MM);
}

/**
 * terrain_sin_q14_deg
 * @brief   Table lookup with linear interpolation inside a degree.
 * @param   deg        0..90
 * @param   frac_ddeg  fraction of a degree, 0..9
 * @return  Q14 sine, unsigned
 * @context any
 */
static uint32_t terrain_sin_q14_deg(uint32_t deg, uint32_t frac_ddeg)
{
    uint32_t base = 0u;

    if (0u == frac_ddeg)
    {
        return (uint32_t)g_sin_tab[deg]; /* cast: entry >= 0 */
    }
    base = (uint32_t)g_sin_tab[deg]; /* cast: entry >= 0 */
    return base +
           (((uint32_t)(g_sin_tab[deg + 1u] - g_sin_tab[deg]) * frac_ddeg) /
            TERRAIN_DDEG_PER_DEG);
    /* signed division by 10, event-time */
}

/**
 * terrain_hump_report
 * @brief   Chooses the reported hump height by TERRAIN_REPORT_MODE:
 *          0 = integrated peak, 1 = geometric, 2 = mean of both.
 * @param   integ_mm  integrated peak
 * @param   geo_mm    geometric estimate
 * @return  height to report in mm
 * @context imu_task (one cycle per hump), host test
 */
static uint32_t terrain_hump_report(uint32_t integ_mm, uint32_t geo_mm)
{
    switch (TERRAIN_REPORT_MODE)
    {
    case 1u:
        return geo_mm;
    case 2u:
        return (integ_mm + geo_mm) / 2u;
    case 0u:
    default:
        return integ_mm;
    }
}

/**
 * terrain_hyst_bump
 * @brief   Counts one more consecutive sample that meets a condition.
 * @param   p_ter  monitor state
 * @return  none
 * @context any (handler helper)
 */
static void terrain_hyst_bump(terrain_t * p_ter)
{
    terrain_hyst_set(p_ter, terrain_hyst_get(p_ter) + 1u);
}

/**
 * terrain_hyst_done
 * @brief   True when the hysteresis count reached TERRAIN_HYST_SAMPLES.
 * @param   p_ter  monitor state
 * @return  true when the condition has held long enough
 * @context any (handler helper)
 */
static bool terrain_hyst_done(terrain_t const * p_ter)
{
    return (terrain_hyst_get(p_ter) >= TERRAIN_HYST_SAMPLES);
}

/**
 * terrain_flat_step
 * @brief   FLAT handler: watch for a climb, and hold the ramp latch after
 *          a rejected ramp.
 * @param   p_ter       monitor state
 * @param   pitch_ddeg  this cycle's pitch, 0.1 deg
 * @return  none
 * @context terrain_step
 */
static void terrain_flat_step(terrain_t * p_ter, int32_t pitch_ddeg)
{
    if (terrain_climb_get(p_ter) > 0u)
    {
        /* Ramp latch (S4-U02): after a climb longer than
           TERRAIN_MAX_CLIMB_MM was rejected as a ramp, do not re-arm
           CLIMBING while the surface is still pitched. The latch clears
           once the pitch stays <= TERRAIN_LEVEL_DDEG for
           TERRAIN_HYST_SAMPLES cycles. */
        if (terrain_abs(pitch_ddeg) <= (uint32_t)TERRAIN_LEVEL_DDEG)
        {
            terrain_hyst_bump(p_ter);
            if (terrain_hyst_done(p_ter))
            {
                terrain_climb_set(p_ter, 0u);
                terrain_hyst_set(p_ter, 0u);
            }
        }
        else
        {
            terrain_hyst_set(p_ter, 0u);
        }
        return;
    }

    if (pitch_ddeg >= TERRAIN_CLIMB_START_DDEG)
    {
        terrain_hyst_bump(p_ter);
        if (terrain_hyst_done(p_ter))
        {
            terrain_state_set(p_ter, TERRAIN_CLIMBING);
            terrain_hyst_set(p_ter, 0u);
            p_ter->acc_um_mg = 0;
            terrain_climb_set(p_ter, 0u);
            terrain_peak_set(p_ter, 0u);
            terrain_pitch_max_set(p_ter, 0);
        }
    }
    else
    {
        terrain_hyst_set(p_ter, 0u);
    }
}

/**
 * terrain_climb_step
 * @brief   CLIMBING handler: integrate the height, track the peak and
 *          the maximum pitch, reject a ramp by distance.
 * @param   p_ter       monitor state
 * @param   p_in        this cycle's inputs
 * @param   pitch_ddeg  this cycle's pitch, 0.1 deg
 * @param   delta_a     ax - a_fwd, mg
 * @param   ds_um       this cycle's travel, um
 * @param   p_out       outputs
 * @return  none
 * @context terrain_step
 */
static void terrain_climb_step(terrain_t * p_ter, terrain_input_t const * p_in,
                               int32_t pitch_ddeg, int32_t delta_a,
                               uint32_t ds_um, terrain_output_t * p_out)
{
    int32_t accum = p_ter->acc_um_mg + ((int32_t)ds_um * delta_a);
    /* cast: ds_um <= 5235 per cycle, so the product fits int32 */

    p_ter->acc_um_mg   = accum;
    p_out->hump_now_mm = terrain_acc_to_mm(accum);
    terrain_climb_set(p_ter, terrain_climb_get(p_ter) + p_in->ds_counts);

    if (p_out->hump_now_mm > terrain_peak_get(p_ter))
    {
        terrain_peak_set(p_ter, p_out->hump_now_mm);
    }
    if (pitch_ddeg > terrain_pitch_max_get(p_ter))
    {
        terrain_pitch_max_set(p_ter, pitch_ddeg);
    }

    if (terrain_climb_get(p_ter) > terrain_climb_limit_counts())
    {
        /* A ramp, not a hump: no report. Set the latch so the
           still-pitched surface cannot immediately re-arm a new climb
           (S4-U02). */
        terrain_reset(p_ter);
        terrain_climb_set(p_ter, 1u);
        return;
    }

    if (pitch_ddeg <= TERRAIN_LEVEL_DDEG)
    {
        terrain_hyst_bump(p_ter);
        if (terrain_hyst_done(p_ter))
        {
            terrain_state_set(p_ter, TERRAIN_DESCENDING);
            terrain_hyst_set(p_ter, 0u);
        }
    }
    else
    {
        terrain_hyst_set(p_ter, 0u);
    }
}

/**
 * terrain_descend_step
 * @brief   DESCENDING handler: keep integrating (the height falls back)
 *          and report the hump when the surface has levelled off.
 * @param   p_ter       monitor state
 * @param   pitch_ddeg  this cycle's pitch, 0.1 deg
 * @param   delta_a     ax - a_fwd, mg
 * @param   ds_um       this cycle's travel, um
 * @param   p_out       outputs
 * @return  none
 * @context terrain_step
 */
static void terrain_descend_step(terrain_t * p_ter, int32_t pitch_ddeg,
                                 int32_t delta_a, uint32_t ds_um,
                                 terrain_output_t * p_out)
{
    int32_t accum = p_ter->acc_um_mg + ((int32_t)ds_um * delta_a);
    /* cast: ds_um <= 5235 per cycle, so the product fits int32 */

    p_ter->acc_um_mg   = accum;
    p_out->hump_now_mm = terrain_acc_to_mm(accum);

    if (terrain_abs(pitch_ddeg) > (uint32_t)TERRAIN_LEVEL_DDEG)
    {
        terrain_hyst_set(p_ter, 0u);
        return;
    }

    terrain_hyst_bump(p_ter);
    if (!terrain_hyst_done(p_ter))
    {
        return;
    }

    {
        uint32_t geo_mm =
            ((uint32_t)TERRAIN_WHEELBASE_MM *
             (uint32_t)terrain_sin_q14(terrain_pitch_max_get(p_ter))) /
            (uint32_t)TERRAIN_Q14_ONE;
        /* casts: the wheelbase is positive and, because the maximum
           pitch is >= TERRAIN_CLIMB_START_DDEG > 0, the sine is >= 0;
           the division is by a power of two (never >> on a signed
           value) */

        terrain_state_set(p_ter, TERRAIN_FLAT);
        terrain_hyst_set(p_ter, 0u);
        p_ter->acc_um_mg = 0; /* keep hump_now = 0 in FLAT */
        if (terrain_peak_get(p_ter) >= TERRAIN_HUMP_MIN_MM)
        {
            p_out->hump_done_mm =
                terrain_hump_report(terrain_peak_get(p_ter), geo_mm);
            p_out->hump_geo_mm = geo_mm;
        }
        terrain_climb_set(p_ter, 0u);
        terrain_peak_set(p_ter, 0u);
        terrain_pitch_max_set(p_ter, 0);
    }
}

/*** end of file ***/
