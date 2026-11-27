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
#include "../app_types.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem2/odometry.h"

/* ---- private constants and types ------------------------------------ */
/* One-wheel tilt smoothing (1/4 per cycle, rounded) and height scale */
#define TERRAIN_ROLL_EMA_DIV    (4)
#define TERRAIN_ROLL_EMA_HALF   (TERRAIN_ROLL_EMA_DIV / 2)
#define TERRAIN_ROLL_MG_PER_G   (1000u)
#define TERRAIN_ROLL_PCT_SCALE  (100u)
#define TERRAIN_ROLL_SCALE_DEN  (TERRAIN_ROLL_MG_PER_G \
                                 * TERRAIN_ROLL_PCT_SCALE)
#define TERRAIN_ROLL_SCALE_HALF (TERRAIN_ROLL_SCALE_DEN / 2u)

/* Rebase/glitch guard for terrain_ds_counts: a plausible per-cycle delta
   is at most APP_TICK_COUNTS_MAX counts. */
#define TERRAIN_DS_DELTA_MAX (4u * APP_TICK_COUNTS_MAX)

#define TERRAIN_DDEG_CIRCLE    (3600)
#define TERRAIN_DDEG_THREE_Q   (2700)
#define TERRAIN_DDEG_HALF      (1800)
#define TERRAIN_DDEG_QUADRANT  (900)
#define TERRAIN_DEG_QUADRANT   (90u)
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

/* Lift-stop: each axis is clamped to +-8192 mg before it is squared, so
   the sum of three squares stays below 2.1e8 (int32_t safe).          */
#define TERRAIN_LIFT_AXIS_MAX (8192)

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
static int32_t  terrain_lift_clamp(int32_t accel_mg);
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

    /* cast: TERRAIN_ATAN_SCALE is a positive constant (1024) */
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
        /* cast: frac is masked to 0..31, fits int32_t */
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
    uint32_t ddeg_u    = 0u;
    uint32_t fold_ddeg = 0u;
    uint32_t deg       = 0u;
    uint32_t frac      = 0u;
    uint32_t q14_value = 0u;
    bool     b_is_neg  = false;

    ddeg = ddeg % TERRAIN_DDEG_CIRCLE;
    if (ddeg < 0)
    {
        ddeg += TERRAIN_DDEG_CIRCLE; /* remainder is > -3600: one add */
    }
    ddeg_u = (uint32_t)ddeg; /* cast: 0 <= ddeg < 3600 */

    /* Fold in 0.1 deg units before splitting degree + fraction so Q2/Q4
       interpolate downward from 90 deg and never index past 90.
       casts: quadrant angle constants (900..3600) are positive. */
    if (ddeg_u <= (uint32_t)TERRAIN_DDEG_QUADRANT)
    {
        fold_ddeg = ddeg_u;
    }
    else if (ddeg_u < (uint32_t)TERRAIN_DDEG_HALF)
    {
        /* cast: TERRAIN_DDEG_HALF (1800) > ddeg_u */
        fold_ddeg = (uint32_t)TERRAIN_DDEG_HALF - ddeg_u;
    }
    else if (ddeg_u <= (uint32_t)TERRAIN_DDEG_THREE_Q)
    {
        b_is_neg  = true;
        /* cast: TERRAIN_DDEG_HALF (1800) <= ddeg_u */
        fold_ddeg = ddeg_u - (uint32_t)TERRAIN_DDEG_HALF;
    }
    else
    {
        b_is_neg  = true;
        /* cast: TERRAIN_DDEG_CIRCLE (3600) > ddeg_u */
        fold_ddeg = (uint32_t)TERRAIN_DDEG_CIRCLE - ddeg_u;
    }

    deg       = fold_ddeg / APP_DDEG_PER_DEG;
    frac      = fold_ddeg % APP_DDEG_PER_DEG;
    q14_value = terrain_sin_q14_deg(deg, frac);

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
    /* casts: both wrapped headings are 0..3599, fit int32_t */
    int32_t delta = (int32_t)now_wrap - (int32_t)prev_wrap;
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
 * @brief   Motion class, first match wins: LIFTED, CLIMBING, DESCENDING,
 *          TURNING, ACCELERATING, STATIONARY, else CRUISING.
 * @param   terrain_state  TERRAIN_*
 * @param   b_is_lifted    SAFETY_LIFT is set (lifted or overturned)
 * @param   turn_dps       turn rate, deg/s
 * @param   a_fwd_mg       forward acceleration, mg
 * @param   speed_cpw      speed, counts per 50 ms window
 * @return  APP_MCLASS_*
 * @context imu_task, host test
 */
uint32_t terrain_classify(uint32_t terrain_state, bool b_is_lifted,
                          int32_t turn_dps, int32_t a_fwd_mg, int32_t speed_cpw)
{
    if (b_is_lifted)
    {
        return APP_MCLASS_LIFTED;
    }
    if (TERRAIN_CLIMBING == terrain_state)
    {
        return APP_MCLASS_CLIMBING;
    }
    if (TERRAIN_DESCENDING == terrain_state)
    {
        return APP_MCLASS_DESCENDING;
    }
    /* cast: IMU_TURNING_DPS is a positive threshold constant */
    if (terrain_abs(turn_dps) >= (uint32_t)IMU_TURNING_DPS)
    {
        return APP_MCLASS_TURNING;
    }
    /* cast: IMU_ACCEL_MG is a positive threshold constant */
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

/**
 * terrain_lift_step
 * @brief   Lift-stop check (Change Requirement 2): no classification,
 *          two thresholds on the calibrated averages. The car counts as
 *          lifted or overturned while az < IMU_LIFT_TILT_AZ_MG (tilted
 *          past 60 deg or upside down) or the size of the acceleration
 *          differs from 1 g by more than IMU_LIFT_DG_MG (a hand picking
 *          it up or putting it down). The size is compared squared:
 *          (1000 - DG)^2 <= ax^2 + ay^2 + az^2 <= (1000 + DG)^2 is "on
 *          the ground", whatever the slope. It must hold
 *          IMU_LIFT_CONFIRM_CYCLES cycles in a row; one cycle without it
 *          restarts the count.
 * @param   lift_cnt      cycles in a row so far (PRV_IMU_A LIFT_CNT)
 * @param   ax_mg         calibrated X average this cycle
 * @param   ay_mg         calibrated Y average this cycle
 * @param   az_mg         calibrated Z average this cycle (+1000 at rest)
 * @param   pb_is_lifted  out: true once the count reaches the confirm
 *                        value
 * @return  the new count, 0..IMU_LIFT_CONFIRM_CYCLES (saturates)
 * @context imu_task, host test
 */
uint32_t terrain_lift_step(uint32_t lift_cnt, int32_t ax_mg, int32_t ay_mg,
                           int32_t az_mg, bool * pb_is_lifted)
{
    /* Clamped to +-8192 mg (the +-4 g range plus the 12-bit offsets
       stays inside it), so each square is < 6.8e7 and the sum < 2.1e8:
       no overflow in int32_t. */
    int32_t const acc_x    = terrain_lift_clamp(ax_mg);
    int32_t const acc_y    = terrain_lift_clamp(ay_mg);
    int32_t const acc_z    = terrain_lift_clamp(az_mg);
    int32_t const mag2     = (acc_x * acc_x) + (acc_y * acc_y)
                             + (acc_z * acc_z);
    int32_t const low_mg   = IMU_REST_AZ_MG - IMU_LIFT_DG_MG;
    int32_t const high_mg  = IMU_REST_AZ_MG + IMU_LIFT_DG_MG;
    bool const    b_tilted = (az_mg < IMU_LIFT_TILT_AZ_MG);
    bool const    b_push   = (mag2 < (low_mg * low_mg))
                             || (mag2 > (high_mg * high_mg));
    uint32_t      cnt      = 0u;

    if (b_tilted || b_push)
    {
        cnt = (lift_cnt < IMU_LIFT_CONFIRM_CYCLES) ? (lift_cnt + 1u)
                                                   : IMU_LIFT_CONFIRM_CYCLES;
    }
    *pb_is_lifted = (cnt >= IMU_LIFT_CONFIRM_CYCLES);

    return cnt;
}

/**
 * terrain_ir_lift_step
 * @brief   Level-lift check (S4-U17): with no floor under them both IR
 *          sensors read near full scale, above any tape on the track.
 *          Both above IMU_LIFT_IR_RAW_MIN for IMU_LIFT_IR_CYCLES cycles
 *          in a row counts as lifted; one cycle without it restarts the
 *          count.
 * @param   ir_cnt        cycles in a row so far (PRV_IMU_A IR_LIFT_CNT)
 * @param   raw_l         left raw IR ADC, 0..4095
 * @param   raw_r         right raw IR ADC, 0..4095
 * @param   pb_is_lifted  out: true once the count reaches
 *                        IMU_LIFT_IR_CYCLES
 * @return  the new count, 0..IMU_LIFT_IR_CYCLES (saturates)
 * @context imu_task, host test
 */
uint32_t terrain_ir_lift_step(uint32_t ir_cnt, uint32_t raw_l,
                              uint32_t raw_r, bool * pb_is_lifted)
{
    uint32_t cnt = 0u;

    if ((raw_l > IMU_LIFT_IR_RAW_MIN) && (raw_r > IMU_LIFT_IR_RAW_MIN))
    {
        cnt = (ir_cnt < IMU_LIFT_IR_CYCLES) ? (ir_cnt + 1u)
                                            : IMU_LIFT_IR_CYCLES;
    }
    *pb_is_lifted = (cnt >= IMU_LIFT_IR_CYCLES);

    return cnt;
}

/**
 * terrain_roll_hump_step
 * @brief   One-wheel hump (S4-U18): the right wheel alone climbs a thin
 *          strip, so the car tilts sideways. The smoothed |ay| rises to
 *          g x h / track at the top; h = track x peak / 1000 mg (scaled
 *          by TERRAIN_ROLL_GAIN_PCT). A hump only starts while driving
 *          straight (a turn's sideways acceleration also shows in ay)
 *          and forward; on it, more than TERRAIN_ROLL_MAX_TICKS cycles
 *          is a ramp or a held tilt and is dropped without a report.
 * @param   word       PRV_IMU_R word
 * @param   ay_mg      calibrated sideways average this cycle
 * @param   dpos_l     left encoder steps this cycle
 * @param   dpos_r     right encoder steps this cycle
 * @param   p_done_mm  out: height of a hump that ended now, else 0
 * @return  the new PRV_IMU_R word
 * @context imu_task, host test
 */
uint32_t terrain_roll_hump_step(uint32_t word, int32_t ay_mg, int32_t dpos_l,
                                int32_t dpos_r, uint32_t * p_done_mm)
{
    bool const     b_on     = (0u != app_bits_get(word, PRV_IMU_R_STATE_POS,
                                                  PRV_IMU_R_STATE_BITS));
    uint32_t       hyst     = app_bits_get(word, PRV_IMU_R_HYST_POS,
                                           PRV_IMU_R_HYST_BITS);
    int32_t        filt     = app_bits_decode(
                                  app_bits_get(word, PRV_IMU_R_FILT_POS,
                                               PRV_IMU_R_FILT_BITS),
                                  PRV_IMU_R_FILT_OFFSET);
    uint32_t       peak     = app_bits_get(word, PRV_IMU_R_PEAK_POS,
                                           PRV_IMU_R_PEAK_BITS);
    uint32_t       ticks    = app_bits_get(word, PRV_IMU_R_TICKS_POS,
                                           PRV_IMU_R_TICKS_BITS);
    int32_t const  turn     = dpos_r - dpos_l;
    int32_t const  fwd      = dpos_r + dpos_l;
    bool const     b_strt   = (turn <= TERRAIN_ROLL_STRAIGHT_COUNTS)
                              && (turn >= -TERRAIN_ROLL_STRAIGHT_COUNTS)
                              && (fwd > 0);
    uint32_t       mag      = 0u;
    int32_t        diff     = 0;
    bool           b_now_on = b_on;

    *p_done_mm = 0u;

    /* EMA 1/TERRAIN_ROLL_EMA_DIV per cycle; a signed division, never
       >> (BARR-C 5.3). The step is rounded to nearest (S4-U19):
       truncation stopped the smoothed value 3 mg short of a steady tilt,
       0.25 mm of a 4 mm hump.                                         */
    diff = terrain_lift_clamp(ay_mg) - filt;
    filt = filt + ((diff + ((diff >= 0) ? TERRAIN_ROLL_EMA_HALF
                                        : -TERRAIN_ROLL_EMA_HALF))
                   / TERRAIN_ROLL_EMA_DIV);
    if (filt > PRV_IMU_R_FILT_MAX)
    {
        filt = PRV_IMU_R_FILT_MAX;
    }
    if (filt < PRV_IMU_R_FILT_MIN)
    {
        filt = PRV_IMU_R_FILT_MIN;
    }
    mag = terrain_abs(filt);

    if (!b_on)
    {
        ticks = 0u;
        peak  = 0u;
        /* cast: TERRAIN_ROLL_START_MG is a positive threshold constant */
        if (b_strt && (mag >= (uint32_t)TERRAIN_ROLL_START_MG))
        {
            hyst++;
            if (hyst >= TERRAIN_ROLL_HYST)
            {
                b_now_on = true;
                hyst     = 0u;
                peak     = mag;
            }
        }
        else
        {
            hyst = 0u;
        }
    }
    else
    {
        /* cast: PRV_IMU_R TICKS max is a positive 8-bit layout bound (255) */
        bool const b_latched = (ticks >= (uint32_t)PRV_IMU_R_TICKS_MAX);

        if (!b_latched)
        {
            if (mag > peak)
            {
                peak = mag;
            }
            ticks++;
            if (ticks > TERRAIN_ROLL_MAX_TICKS)
            {
                /* a ramp or a held tilt: latch, no report, and no new
                   hump until the car is level again */
                ticks = PRV_IMU_R_TICKS_MAX;
                peak  = 0u;
            }
        }
        /* cast: TERRAIN_ROLL_LEVEL_MG is a positive threshold constant */
        if (mag <= (uint32_t)TERRAIN_ROLL_LEVEL_MG)
        {
            hyst++;
            if (hyst >= TERRAIN_ROLL_HYST)
            {
                if (!b_latched)
                {
                    *p_done_mm = (((APP_TRACK_UM / APP_UM_PER_MM)
                                   * peak * TERRAIN_ROLL_GAIN_PCT)
                                  + TERRAIN_ROLL_SCALE_HALF)
                                 / TERRAIN_ROLL_SCALE_DEN;
                }
                b_now_on = false;
                hyst     = 0u;
                ticks    = 0u;
                peak     = 0u;
            }
        }
        else
        {
            hyst = 0u;
        }
    }
    if (hyst > PRV_IMU_R_HYST_MAX)
    {
        hyst = PRV_IMU_R_HYST_MAX;
    }
    if (peak > PRV_IMU_R_PEAK_MAX)
    {
        peak = PRV_IMU_R_PEAK_MAX;
    }
    if (ticks > PRV_IMU_R_TICKS_MAX)
    {
        ticks = PRV_IMU_R_TICKS_MAX;
    }

    word = app_bits_put(word, PRV_IMU_R_STATE_POS, PRV_IMU_R_STATE_BITS,
                        b_now_on ? 1u : 0u);
    word = app_bits_put(word, PRV_IMU_R_HYST_POS, PRV_IMU_R_HYST_BITS, hyst);
    word = app_bits_put(word, PRV_IMU_R_FILT_POS, PRV_IMU_R_FILT_BITS,
                        app_bits_encode(filt, PRV_IMU_R_FILT_MIN,
                                        PRV_IMU_R_FILT_MAX,
                                        PRV_IMU_R_FILT_OFFSET, NULL));
    word = app_bits_put(word, PRV_IMU_R_PEAK_POS, PRV_IMU_R_PEAK_BITS, peak);
    word = app_bits_put(word, PRV_IMU_R_TICKS_POS, PRV_IMU_R_TICKS_BITS,
                        ticks);

    return word;
}

/**
 * terrain_roll_hump_line
 * @brief   terrain_roll_hump_step() gated to line following (S4-U20): off
 *          the line the encoder steps are passed as 0 (no new hump can
 *          start: it needs forward travel) and a hump that ends is not
 *          reported. The smoothed tilt keeps running either way.
 * @param   word       PRV_IMU_R word
 * @param   ay_mg      calibrated sideways average this cycle
 * @param   dpos_l     left encoder steps this cycle
 * @param   dpos_r     right encoder steps this cycle
 * @param   b_on_line  true while RUN follows the line (FOLLOW, ARMED)
 * @param   p_done_mm  out: height of a hump that ended now, else 0
 * @return  the new PRV_IMU_R word
 * @context imu_task, host test
 */
uint32_t terrain_roll_hump_line(uint32_t word, int32_t ay_mg, int32_t dpos_l,
                                int32_t dpos_r, bool b_on_line,
                                uint32_t * p_done_mm)
{
    uint32_t const next = terrain_roll_hump_step(word, ay_mg,
                                                 b_on_line ? dpos_l : 0,
                                                 b_on_line ? dpos_r : 0,
                                                 p_done_mm);

    if (!b_on_line)
    {
        *p_done_mm = 0u;
    }

    return next;
}

/* ---- private functions ------------------------------------------------ */

/**
 * terrain_lift_clamp
 * @brief   Clamps one axis to +-TERRAIN_LIFT_AXIS_MAX before squaring.
 * @param   accel_mg  calibrated average, mg
 * @return  the clamped value
 * @context imu_task, host test
 */
static int32_t terrain_lift_clamp(int32_t accel_mg)
{
    int32_t out = accel_mg;

    if (out > TERRAIN_LIFT_AXIS_MAX)
    {
        out = TERRAIN_LIFT_AXIS_MAX;
    }
    else if (out < -TERRAIN_LIFT_AXIS_MAX)
    {
        out = -TERRAIN_LIFT_AXIS_MAX;
    }
    else
    {
        /* in range */
    }

    return out;
}

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
    /* cast: peak_mm <= 2148, clamped to PRV_IMU_B PEAK_NOW (0..127) */
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

    if (deg >= TERRAIN_DEG_QUADRANT)
    {
        return (uint32_t)g_sin_tab[TERRAIN_DEG_QUADRANT]; /* cast: >= 0 */
    }
    base = (uint32_t)g_sin_tab[deg]; /* cast: entry >= 0 */
    if (0u == frac_ddeg)
    {
        return base;
    }
    /* cast: g_sin_tab is non-decreasing (0..16384), so diff >= 0 */
    return base +
           (((uint32_t)(g_sin_tab[deg + 1u] - g_sin_tab[deg]) * frac_ddeg) /
            APP_DDEG_PER_DEG);
    /* unsigned division by 10, event-time */
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
           TERRAIN_HYST_SAMPLES cycles.
           cast: TERRAIN_LEVEL_DDEG is a positive threshold constant. */
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
    /* cast: ds_um <= 20736 um (64 counts x 324 um) fits int32_t;
       with |delta_a| <= 12286 mg, the product fits int32_t */
    int32_t accum = p_ter->acc_um_mg + ((int32_t)ds_um * delta_a);

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
    /* cast: ds_um <= 20736 um (64 counts x 324 um) fits int32_t;
       with |delta_a| <= 12286 mg, the product fits int32_t */
    int32_t accum = p_ter->acc_um_mg + ((int32_t)ds_um * delta_a);

    p_ter->acc_um_mg   = accum;
    p_out->hump_now_mm = terrain_acc_to_mm(accum);

    /* cast: TERRAIN_LEVEL_DDEG is a positive threshold constant */
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
        /* casts: wheelbase and APP_Q14_ONE (16384) are positive; maximum
           pitch >= TERRAIN_CLIMB_START_DDEG > 0 so sine is in 0..16384 */
        uint32_t geo_mm =
            ((uint32_t)TERRAIN_WHEELBASE_MM *
             (uint32_t)terrain_sin_q14(terrain_pitch_max_get(p_ter))) /
            (uint32_t)APP_Q14_ONE;

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
