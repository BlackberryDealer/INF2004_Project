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
 */

#include "terrain.h" /* own header first (BARR-C 4.3.c) */
#include "cfg_imu.h"
#include "../subsystem2/odometry.h"

/* ---- private constants and types ------------------------------------ */

/* Rebase/glitch guard for terrain_ds_counts: a plausible per-cycle delta
   is at most APP_TICK_COUNTS_MAX counts. */
#define TERRAIN_DS_DELTA_MAX (4u * APP_TICK_COUNTS_MAX)

#define TERRAIN_DDEG_CIRCLE (3600)
#define TERRAIN_DDEG_HALF   (1800)
#define TERRAIN_Q14_ONE     (16384)
#define TERRAIN_ATAN_K_MAX  (32)
#define TERRAIN_ATAN_SCALE  (1024)

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

static uint32_t terrain_acc_to_mm(int32_t acc_um_mg);
static uint32_t terrain_climb_limit_counts(void);
static uint32_t terrain_sin_q14_deg(uint32_t deg, uint32_t rem_ddeg);
static uint32_t terrain_hump_report(uint32_t integ_mm, uint32_t geo_mm);

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
 * @brief   Integer atan2(y, x) in 0.1 deg, -1800..1799, error <= 0.2 deg.
 *          Works on magnitudes, then maps back by quadrant. One division,
 *          allowed: called at most twice per 10 ms cycle.
 * @param   y  signed value
 * @param   x  signed value
 * @return  angle in 0.1 deg
 * @context any (imu_task, host test)
 */
int32_t terrain_atan2_ddeg(int32_t y, int32_t x)
{
    uint32_t mag_y  = terrain_abs(y);
    uint32_t mag_x  = terrain_abs(x);
    bool     b_swap = (mag_y > mag_x);
    uint32_t small  = 0u;
    uint32_t big    = 0u;
    uint32_t r      = 0u;
    uint32_t k      = 0u;
    uint32_t f      = 0u;
    int32_t  a      = 0;

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

    r = (small * (uint32_t)TERRAIN_ATAN_SCALE) / big;
    /* 0..1024; the one division */
    k = r >> 5;  /* 0..32 */
    f = r & 31u; /* fraction within a step */
    if (TERRAIN_ATAN_K_MAX == k)
    {
        a = g_atan_tab[TERRAIN_ATAN_K_MAX];
    }
    else
    {
        a = g_atan_tab[k] +
            (((g_atan_tab[k + 1u] - g_atan_tab[k]) * (int32_t)f) / 32);
        /* signed division by 32, not >>:
           C99 truncates toward zero */
    }
    if (b_swap)
    {
        a = 900 - a; /* complement: angle above 45 deg */
    }

    if ((x < 0) && (y >= 0))
    {
        return TERRAIN_DDEG_HALF - a; /* second quadrant: 180 - a */
    }
    if ((x < 0) && (y < 0))
    {
        return a - TERRAIN_DDEG_HALF; /* third quadrant: a - 180 */
    }
    if ((x >= 0) && (y < 0))
    {
        return -a;
    }
    return a; /* x >= 0, y >= 0 */
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
    uint32_t deg      = 0u;
    uint32_t rem      = 0u;
    uint32_t val      = 0u;
    bool     b_is_neg = false;

    ddeg = ddeg % TERRAIN_DDEG_CIRCLE;
    if (ddeg < 0)
    {
        ddeg += TERRAIN_DDEG_CIRCLE; /* remainder is > -3600: one add */
    }

    deg = (uint32_t)ddeg / 10u; /* cast: ddeg >= 0 here */
    rem = (uint32_t)ddeg % 10u;

    if (deg < 90u)
    {
        val = terrain_sin_q14_deg(deg, rem);
    }
    else if (deg < 180u)
    {
        val = terrain_sin_q14_deg(180u - deg, rem);
    }
    else if (deg < 270u)
    {
        b_is_neg = true;
        val      = terrain_sin_q14_deg(deg - 180u, rem);
    }
    else
    {
        b_is_neg = true;
        val      = terrain_sin_q14_deg(360u - deg, rem);
    }

    if (b_is_neg)
    {
        return (int32_t)(0u - val); /* cast: val <= 16384 */
    }
    return (int32_t)val; /* cast: val <= 16384 */
}

/**
 * terrain_cos_q14
 * @brief   Cosine via sin(ddeg + 900).
 * @param   ddeg  angle in 0.1 deg
 * @return  Q14 cosine, -16384..16384
 * @context any
 */
int32_t terrain_cos_q14(int32_t ddeg)
{
    return terrain_sin_q14(ddeg + 900);
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
 * @brief   Empties the 5-entry speed history (run start).
 * @param   p_hist  history
 * @return  none
 * @context any
 */
void terrain_speed_hist_reset(terrain_speed_hist_t * p_hist)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < 5u; idx++)
    {
        p_hist->speed[idx] = 0;
    }
    p_hist->idx   = 0u;
    p_hist->count = 0u;
}

/**
 * terrain_speed_hist_push
 * @brief   Pushes one cycle's speed (cpw) and returns the forward
 *          acceleration: the difference across the FULL 50 ms window
 *          (never 10 ms), scaled by TERRAIN_AFWD_MG_PER_CPW_Q10 / 1024.
 *          Returns 0 until the window has filled once, so no stale
 *          pre-run data can become a false acceleration.
 * @param   p_hist     history
 * @param   speed_cpw  this cycle's speed, -127..127
 * @return  a_fwd in mg
 * @context any
 */
int32_t terrain_speed_hist_push(terrain_speed_hist_t * p_hist,
                                int32_t                speed_cpw)
{
    int32_t speed_old = p_hist->speed[p_hist->idx];

    p_hist->speed[p_hist->idx] = speed_cpw;
    p_hist->idx                = (p_hist->idx + 1u) % 5u;

    if (p_hist->count < 5u)
    {
        p_hist->count++;
        return 0;
    }

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
    p_ter->state          = TERRAIN_FLAT;
    p_ter->hyst           = 0u;
    p_ter->acc_um_mg      = 0;
    p_ter->climb_counts   = 0u;
    p_ter->peak_now_mm    = 0u;
    p_ter->pitch_max_ddeg = 0;
}

/**
 * terrain_step
 * @brief   One 10 ms cycle of the Terrain Monitor (Figure 6.3 with the
 *          review changes). pitch = atan2(ax - a_fwd, az), + = nose up.
 *          Distance, not time, decides "ramp": the climb limit is
 *          TERRAIN_MAX_CLIMB_MM converted to counts.
 * @param   p_ter  monitor state
 * @param   p_in   this cycle's inputs
 * @param   p_out  outputs: pitch, hump now, hump done (one cycle), geo
 * @return  none
 * @context imu_task, host test
 */
void terrain_step(terrain_t * p_ter, terrain_input_t const * p_in,
                  terrain_output_t * p_out)
{
    int32_t pitch =
        terrain_atan2_ddeg(p_in->ax_mg - p_in->a_fwd_mg, p_in->az_mg);
    uint32_t ds_um = p_in->ds_counts * APP_UM_PER_COUNT;
    int32_t  da    = p_in->ax_mg - p_in->a_fwd_mg;
    int32_t  accum = 0;

    p_out->pitch_ddeg   = pitch;
    p_out->hump_now_mm  = terrain_acc_to_mm(p_ter->acc_um_mg);
    p_out->hump_done_mm = 0u;
    p_out->hump_geo_mm  = 0u;

    switch (p_ter->state)
    {
    case TERRAIN_FLAT:
        if (p_ter->climb_counts > 0u)
        {
            /* Ramp latch (S4-U02): after a climb longer than
               TERRAIN_MAX_CLIMB_MM was rejected as a ramp, do not
               re-arm CLIMBING while the surface is still pitched. The
               latch clears once the pitch stays <= TERRAIN_LEVEL_DDEG
               for TERRAIN_HYST_SAMPLES cycles. */
            if (terrain_abs(pitch) <= (uint32_t)TERRAIN_LEVEL_DDEG)
            {
                p_ter->hyst++;
                if (p_ter->hyst >= TERRAIN_HYST_SAMPLES)
                {
                    p_ter->climb_counts = 0u;
                    p_ter->hyst         = 0u;
                }
            }
            else
            {
                p_ter->hyst = 0u;
            }
            break;
        }
        if (pitch >= TERRAIN_CLIMB_START_DDEG)
        {
            p_ter->hyst++;
            if (p_ter->hyst >= TERRAIN_HYST_SAMPLES)
            {
                p_ter->state          = TERRAIN_CLIMBING;
                p_ter->hyst           = 0u;
                p_ter->acc_um_mg      = 0;
                p_ter->climb_counts   = 0u;
                p_ter->peak_now_mm    = 0u;
                p_ter->pitch_max_ddeg = 0;
            }
        }
        else
        {
            p_ter->hyst = 0u;
        }
        break;

    case TERRAIN_CLIMBING:
        /* Bound comment (spec): |ds_um| <= 5235 and |da| <= 4000 per
           cycle, so the product fits int32. */
        accum            = p_ter->acc_um_mg +
                           ((int32_t)ds_um * da); /* cast: <= 5235, positive */
        p_ter->acc_um_mg = accum;
        p_out->hump_now_mm = terrain_acc_to_mm(accum);
        p_ter->climb_counts += p_in->ds_counts;
        if (p_out->hump_now_mm > p_ter->peak_now_mm)
        {
            p_ter->peak_now_mm = p_out->hump_now_mm;
        }
        if (pitch > p_ter->pitch_max_ddeg)
        {
            p_ter->pitch_max_ddeg = pitch;
        }

        if (p_ter->climb_counts > terrain_climb_limit_counts())
        {
            /* A ramp, not a hump: no report. Set the latch so the
               still-pitched surface cannot immediately re-arm a new
               climb (S4-U02). */
            terrain_reset(p_ter);
            p_ter->climb_counts = 1u;
            break;
        }
        if (pitch <= TERRAIN_LEVEL_DDEG)
        {
            p_ter->hyst++;
            if (p_ter->hyst >= TERRAIN_HYST_SAMPLES)
            {
                p_ter->state = TERRAIN_DESCENDING;
                p_ter->hyst  = 0u;
            }
        }
        else
        {
            p_ter->hyst = 0u;
        }
        break;

    case TERRAIN_DESCENDING:
        /* Same accumulation: on a descent da is negative and the
           integrated height falls back toward zero. */
        accum            = p_ter->acc_um_mg +
                           ((int32_t)ds_um * da); /* cast: <= 5235, positive */
        p_ter->acc_um_mg = accum;
        p_out->hump_now_mm = terrain_acc_to_mm(accum);

        if (terrain_abs(pitch) <= (uint32_t)TERRAIN_LEVEL_DDEG)
        {
            p_ter->hyst++;
            if (p_ter->hyst >= TERRAIN_HYST_SAMPLES)
            {
                uint32_t h_geo =
                    ((uint32_t)TERRAIN_WHEELBASE_MM *
                     (uint32_t)terrain_sin_q14(p_ter->pitch_max_ddeg)) /
                    (uint32_t)TERRAIN_Q14_ONE;
                /* casts: wheelbase >= 0 and, as
                   pitch_max >= CLIMB_START > 0, the
                   sine is >= 0; the division is by a
                   power of two (never >> on a signed
                   value) */

                p_ter->state     = TERRAIN_FLAT;
                p_ter->hyst      = 0u;
                p_ter->acc_um_mg = 0; /* keep hump_now = 0 in FLAT */
                if (p_ter->peak_now_mm >= TERRAIN_HUMP_MIN_MM)
                {
                    p_out->hump_done_mm =
                        terrain_hump_report(p_ter->peak_now_mm, h_geo);
                    p_out->hump_geo_mm = h_geo;
                }
                p_ter->climb_counts   = 0u;
                p_ter->peak_now_mm    = 0u;
                p_ter->pitch_max_ddeg = 0;
            }
        }
        else
        {
            p_ter->hyst = 0u;
        }
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
 * @param   deg       0..90
 * @param   rem_ddeg  fraction of a degree, 0..9
 * @return  Q14 sine, unsigned
 * @context any
 */
static uint32_t terrain_sin_q14_deg(uint32_t deg, uint32_t rem_ddeg)
{
    uint32_t base = 0u;

    if (0u == rem_ddeg)
    {
        return (uint32_t)g_sin_tab[deg]; /* cast: entry >= 0 */
    }
    base = (uint32_t)g_sin_tab[deg]; /* cast: entry >= 0 */
    return base +
           (((uint32_t)(g_sin_tab[deg + 1u] - g_sin_tab[deg]) * rem_ddeg) /
            10u);
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

/*** end of file ***/
