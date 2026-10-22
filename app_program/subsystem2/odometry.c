/**
 * @file    odometry.c
 * @brief   Encoder counts <-> millimetres, speeds, turn arcs and heading,
 *          plus the 50 ms speed window. Pure C, host-safe.
 * @owner   Buddy 2 (motion)
 * @kind    algo
 * @reqs    NFR2 (speed feedback), NFR3 (distance and angle accuracy)
 *
 * The conversions are event-time (command start, telemetry), so they may
 * divide. The window functions run every 10 ms and divide only on the rare
 * late-tick path (Part B.7). Constants come from app_types.h, where the
 * MEASURE values (A4, wheel, track) are set by Buddy 2's bench tests.
 */

#include "odometry.h"          /* own header first (BARR-C 4.3.c) */
#include <stddef.h>            /* NULL */
#include "../app_types.h"

/* ---- private constants and types ------------------------------------ */

#define ODOMETRY_UM_PER_MM          (1000u)
#define ODOMETRY_HALF_MM_UM         (500u)
#define ODOMETRY_PI_NUM             (355u)    /* pi ~= 355/113 (3e-7 off) */
#define ODOMETRY_PI_DEN             (113u)
#define ODOMETRY_DEG_PER_REV        (360u)
#define ODOMETRY_X1000              (1000u)

/* Distance one wheel travels in a full on-the-spot turn = pi x track. */
#define ODOMETRY_TRACK_CIRC_UM  ((APP_TRACK_UM * ODOMETRY_PI_NUM) \
                                 / ODOMETRY_PI_DEN)

/* Per-wheel encoder counts per degree of turn, x1000 (~3125). */
#define ODOMETRY_COUNTS_PER_DEG_X1000 \
    (((ODOMETRY_TRACK_CIRC_UM * ODOMETRY_X1000) / ODOMETRY_DEG_PER_REV) \
     / APP_UM_PER_COUNT)

/* Heading change per count of wheel difference, in 0.01 deg x100 (~1599):
   (um per count / track) rad x (18000 / pi) cdeg/rad x 100.             */
#define ODOMETRY_CDEG_DIFF_NUM      (1800000u)
#define ODOMETRY_CDEG_PER_DIFF_X100 \
    ((int32_t) ((APP_UM_PER_COUNT * ODOMETRY_CDEG_DIFF_NUM) \
                / ODOMETRY_TRACK_CIRC_UM)) /* cast: < 2^31, checked below */
#define ODOMETRY_X100               (100)
#define ODOMETRY_POS_MAX            (131071)  /* one run <= 32 m (A23) */
#define ODOMETRY_DIFF_MAX           (262142)  /* 2 x 131071 (A23)      */

/* Speed window */
#define ODOMETRY_DELTA_MAX          (63)      /* 7-bit slot (Phase 4)   */
#define ODOMETRY_DT_MAX_MS          (63u)     /* 6-bit slot (Phase 4)   */
#define ODOMETRY_SPAN_MIN_MS        (45u)     /* normal window: 50 +- 5 */
#define ODOMETRY_SPAN_MAX_MS        (55u)

/* No intermediate may exceed 32 bits (the preprocessor computes these
   checks in 64-bit arithmetic, so the checks themselves cannot wrap).  */
#if ((APP_TRACK_UM * ODOMETRY_PI_NUM) > 0xFFFFFFFFu)
#error "Track x 355 no longer fits 32 bits"
#endif
#if ((ODOMETRY_TRACK_CIRC_UM * ODOMETRY_X1000) > 0xFFFFFFFFu)
#error "Turn circumference x 1000 no longer fits 32 bits"
#endif
#if ((APP_UM_PER_COUNT * ODOMETRY_CDEG_DIFF_NUM) > 0xFFFFFFFFu)
#error "um per count x 1800000 no longer fits 32 bits"
#endif
#if ((APP_UM_PER_COUNT * ODOMETRY_CDEG_DIFF_NUM / ODOMETRY_TRACK_CIRC_UM) \
     * ODOMETRY_DIFF_MAX > 0x7FFFFFFF)
#error "Heading product no longer fits int32_t"
#endif
#if ((2u * ((360u * ODOMETRY_COUNTS_PER_DEG_X1000 + 500u) / 1000u)) \
     > 16383u)   /* PRV_MOT_C TARGET2: 14 bits for a 360 deg turn */
#error "2 x the counts of a full turn no longer fit TARGET2: widen it"
#endif

/* ---- private function prototypes ------------------------------------- */

static int32_t odometry_clamp(int32_t value, int32_t limit);

/* ---- public functions ------------------------------------------------- */

/**
 * odometry_counts_to_mm
 * @brief   Encoder counts to millimetres, rounded to nearest.
 * @param   counts  0..12 000 000 (a run is <= 131071, A23)
 * @return  millimetres
 * @context any (event-time)
 */
uint32_t odometry_counts_to_mm(uint32_t counts)
{
    return (((counts * APP_UM_PER_COUNT) + ODOMETRY_HALF_MM_UM)
            / ODOMETRY_UM_PER_MM);
}

/**
 * odometry_mm_to_counts
 * @brief   Millimetres to encoder counts, rounded to nearest.
 * @param   distance_mm  0..4 000 000
 * @return  counts
 * @context any (event-time)
 */
uint32_t odometry_mm_to_counts(uint32_t distance_mm)
{
    return (((distance_mm * ODOMETRY_UM_PER_MM) + (APP_UM_PER_COUNT / 2u))
            / APP_UM_PER_COUNT);
}

/**
 * odometry_cpw_to_mm_s
 * @brief   Counts per 50 ms window to mm/s: counts x um per count / ms.
 * @param   speed_cpw  -127..+127 (+ = forward)
 * @return  mm/s, truncated toward zero
 * @context any (event-time)
 */
int32_t odometry_cpw_to_mm_s(int32_t speed_cpw)
{
    /* casts: both constants are small positive values (349, 50) */
    return ((speed_cpw * (int32_t) APP_UM_PER_COUNT)
            / (int32_t) APP_SPEED_WINDOW_MS);
}

/**
 * odometry_mm_s_to_cpw
 * @brief   mm/s to counts per 50 ms window, rounded to nearest.
 * @param   speed_mm_s  -500..+500 (+ = forward)
 * @return  counts per window
 * @context any (event-time)
 */
int32_t odometry_mm_s_to_cpw(int32_t speed_mm_s)
{
    /* casts: both constants are small positive values (50, 349) */
    int32_t const num  = speed_mm_s * (int32_t) APP_SPEED_WINDOW_MS;
    int32_t const half = (int32_t) APP_UM_PER_COUNT / 2;
    int32_t       cpw  = 0;

    if (num >= 0)
    {
        cpw = (num + half) / (int32_t) APP_UM_PER_COUNT;
    }
    else
    {
        cpw = (num - half) / (int32_t) APP_UM_PER_COUNT; /* round away */
    }

    return cpw;
}

/**
 * odometry_turn_counts
 * @brief   Counts each wheel travels for an on-the-spot turn.
 * @param   angle_deg  0..360
 * @return  per-wheel counts, rounded to nearest
 * @context any (event-time)
 */
uint32_t odometry_turn_counts(uint32_t angle_deg)
{
    return (((angle_deg * ODOMETRY_COUNTS_PER_DEG_X1000)
             + (ODOMETRY_X1000 / 2u)) / ODOMETRY_X1000);
}

/**
 * odometry_heading_cdeg
 * @brief   Heading change from the two wheel positions. Signed throughout
 *          (review V13-03): a negative difference must stay negative.
 * @param   pos_l_counts  left wheel position, counts
 * @param   pos_r_counts  right wheel position, counts
 * @return  0.01 degree, + = robot turned right (clockwise from above)
 * @context any
 */
int32_t odometry_heading_cdeg(int32_t pos_l_counts, int32_t pos_r_counts)
{
    /* Clamp each position to the A23 range first, so the subtraction
       cannot overflow; the difference is then within +-262142.        */
    int32_t const diff = odometry_clamp(pos_l_counts, ODOMETRY_POS_MAX)
                         - odometry_clamp(pos_r_counts, ODOMETRY_POS_MAX);

    /* Both operands int32_t; |product| <= 262142 x 1600 < 2^31 (checked
       above). C99 division truncates toward zero.                      */
    return ((diff * ODOMETRY_CDEG_PER_DIFF_X100) / ODOMETRY_X100);
}

/**
 * odometry_window_reset
 * @brief   Empties the speed window (all slots 0).
 * @param   p_win  window
 * @return  none
 * @context motion_task
 */
void odometry_window_reset(odometry_window_t * p_win)
{
    uint32_t idx = 0u;

    if (NULL != p_win)
    {
        for (idx = 0u; idx < ODOMETRY_WINDOW_SLOTS; idx++)
        {
            p_win->delta_l[idx] = 0;
            p_win->delta_r[idx] = 0;
            p_win->dt_ms[idx]   = 0u;
        }
        p_win->idx = 0u;
    }
}

/**
 * odometry_window_push
 * @brief   Stores one tick: deltas clamped to +-63, dt to 0..63 ms (the
 *          Phase 4 slot widths, so both builds hold the same information).
 * @param   p_win    window
 * @param   delta_l  left counts this tick
 * @param   delta_r  right counts this tick
 * @param   dt_ms    tick length in ms
 * @return  none
 * @context motion_task (10 ms path: no division)
 */
void odometry_window_push(odometry_window_t * p_win, int32_t delta_l,
                          int32_t delta_r, uint32_t dt_ms)
{
    if ((NULL != p_win) && (p_win->idx < ODOMETRY_WINDOW_SLOTS))
    {
        p_win->delta_l[p_win->idx] = odometry_clamp(delta_l,
                                                    ODOMETRY_DELTA_MAX);
        p_win->delta_r[p_win->idx] = odometry_clamp(delta_r,
                                                    ODOMETRY_DELTA_MAX);
        p_win->dt_ms[p_win->idx]   = (dt_ms > ODOMETRY_DT_MAX_MS)
                                     ? ODOMETRY_DT_MAX_MS : dt_ms;

        /* Wrap without % (no division in the 10 ms path). */
        if (p_win->idx >= (ODOMETRY_WINDOW_SLOTS - 1u))
        {
            p_win->idx = 0u;
        }
        else
        {
            p_win->idx++;
        }
    }
}

/**
 * odometry_window_speed
 * @brief   Wheel speeds over the last 5 ticks, in counts per 50 ms. On the
 *          normal path (window 45..55 ms) the sum IS the speed, no
 *          division. Late ticks rescale to 50 ms (rare path).
 * @param   p_win          window
 * @param   p_speed_l_cpw  receives the left speed
 * @param   p_speed_r_cpw  receives the right speed
 * @return  none
 * @context motion_task
 */
void odometry_window_speed(odometry_window_t const * p_win,
                           int32_t * p_speed_l_cpw, int32_t * p_speed_r_cpw)
{
    int32_t  sum_l = 0;
    int32_t  sum_r = 0;
    uint32_t span  = 0u;
    uint32_t idx   = 0u;

    if ((NULL != p_win) && (NULL != p_speed_l_cpw)
        && (NULL != p_speed_r_cpw))
    {
        for (idx = 0u; idx < ODOMETRY_WINDOW_SLOTS; idx++)
        {
            sum_l += p_win->delta_l[idx];
            sum_r += p_win->delta_r[idx];
            span  += p_win->dt_ms[idx];
        }

        if ((span >= ODOMETRY_SPAN_MIN_MS) && (span <= ODOMETRY_SPAN_MAX_MS))
        {
            *p_speed_l_cpw = sum_l;   /* normal path: no division */
            *p_speed_r_cpw = sum_r;
        }
        else if (0u == span)
        {
            *p_speed_l_cpw = 0;
            *p_speed_r_cpw = 0;
        }
        else
        {
            /* rare path: division OK. casts: 50 and span <= 315 are
               small positive values                                    */
            *p_speed_l_cpw = (sum_l * (int32_t) APP_SPEED_WINDOW_MS)
                             / (int32_t) span;
            *p_speed_r_cpw = (sum_r * (int32_t) APP_SPEED_WINDOW_MS)
                             / (int32_t) span;
        }
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * odometry_clamp
 * @brief   Clamps a signed value to [-limit, +limit].
 * @param   value  value to clamp
 * @param   limit  positive limit
 * @return  clamped value
 * @context any
 */
static int32_t odometry_clamp(int32_t value, int32_t limit)
{
    int32_t result = value;

    if (value > limit)
    {
        result = limit;
    }
    else if (value < -limit)
    {
        result = -limit;
    }
    else
    {
        /* in range */
    }

    return result;
}

/*** end of file ***/
