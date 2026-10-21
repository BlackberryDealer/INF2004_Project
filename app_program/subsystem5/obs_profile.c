/**
 * @file    obs_profile.c
 * @brief   Turns the 25-point servo scan into an obstacle profile and a
 *          bypass plan. Pure C: integer trig only, compiles on a laptop.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    algo
 * @reqs    FR7 (profile), FR8 (choose side), NFR16 (leg limits)
 */

#include "obs_profile.h"       /* own header first (BARR-C 4.3.c) */
#include "cfg_obstacle.h"

/* ---- private constants and types ------------------------------------ */

#define OBS_SIN_ENTRIES    (37u)   /* 0, 5, ..., 180 deg               */
#define OBS_SIN_IDX_90     (18u)   /* index of 90 deg                  */
#define OBS_STEP_IDX_BASE  (6u)    /* scan step 0 = 30 deg = index 6   */
#define OBS_Q14_SHIFT      (14u)
#define OBS_STEP_LAST      (APP_SCAN_STEPS - 1u)

typedef struct
{
    bool     b_is_near;
    int32_t  x_mm;       /* + = right of the robot's centre line */
    uint32_t y_mm;       /* forward distance                     */
} obs_point_t;

/* ---- private data ---------------------------------------------------- */

/* sin(5k deg) in Q14 for k = 0..36, taken from Buddy 4's 1-degree table
   (B4.2.1) at every fifth degree. cos(t) = sin(90 - t) by index.       */
static int16_t const g_obs_sin_q14[OBS_SIN_ENTRIES] =
{
    0, 1428, 2845, 4240, 5604, 6924, 8192, 9397, 10531, 11585,
    12551, 13421, 14189, 14849, 15396, 15826, 16135, 16322, 16384,
    16322, 16135, 15826, 15396, 14849, 14189, 13421, 12551, 11585,
    10531, 9397, 8192, 6924, 5604, 4240, 2845, 1428, 0
};

/* ---- private function prototypes ------------------------------------- */

static void     obs_make_points(uint16_t const * p_scan_mm,
                                obs_point_t * p_pts, uint32_t * p_best);
static uint32_t obs_q14_mul(uint32_t distance_mm, uint32_t sin_idx);
static void     obs_find_run(obs_point_t const * p_pts, uint32_t best,
                             uint32_t * p_lo, uint32_t * p_hi);
static void     obs_clearances(obs_point_t const * p_pts, uint32_t lo,
                               uint32_t hi, obs_profile_t * p_prof);
static uint32_t obs_min_u(uint32_t value_a, uint32_t value_b);

/* ---- public functions ------------------------------------------------- */

/**
 * obs_profile_build
 * @brief   Finds the obstacle (the contiguous run of near points holding
 *          the nearest point), its lateral extent, width and the free
 *          gaps to other objects on each side.
 * @param   p_scan_mm 25 distances (mm), step n = 30 + 5n degrees
 * @param   p_prof    result; b_is_valid false if nothing is near
 * @context obstacle_task (event time, once per scan)
 */
void obs_profile_build(uint16_t const * p_scan_mm, obs_profile_t * p_prof)
{
    obs_point_t pts[APP_SCAN_STEPS];
    uint32_t    best = APP_SCAN_STEPS;   /* "none" */
    uint32_t    lo   = 0u;
    uint32_t    hi   = 0u;

    p_prof->b_is_valid     = false;
    p_prof->angle_step     = 0u;
    p_prof->nearest_mm     = APP_PROFILE_CLEAR_MM;
    p_prof->x_left_mm      = 0;
    p_prof->x_right_mm     = 0;
    p_prof->width_mm       = APP_PROFILE_CLEAR_MM;
    p_prof->clear_left_mm  = APP_PROFILE_CLEAR_MM;
    p_prof->clear_right_mm = APP_PROFILE_CLEAR_MM;

    obs_make_points(p_scan_mm, pts, &best);

    if (best < APP_SCAN_STEPS)
    {
        obs_find_run(pts, best, &lo, &hi);
        p_prof->x_left_mm  = pts[lo].x_mm;
        p_prof->x_right_mm = pts[lo].x_mm;
        for (uint32_t idx = lo; idx <= hi; idx++)
        {
            if (pts[idx].x_mm < p_prof->x_left_mm)
            {
                p_prof->x_left_mm = pts[idx].x_mm;
            }
            if (pts[idx].x_mm > p_prof->x_right_mm)
            {
                p_prof->x_right_mm = pts[idx].x_mm;
            }
        }
        p_prof->b_is_valid = true;
        p_prof->angle_step = best;
        p_prof->nearest_mm = pts[best].y_mm;
        /* cast: x_right >= x_left, difference < 2 x OBS_NEAR_MM */
        p_prof->width_mm   = obs_min_u((uint32_t) (p_prof->x_right_mm
                                                   - p_prof->x_left_mm),
                                       APP_PROFILE_CLEAR_MM);
        obs_clearances(pts, lo, hi, p_prof);
    }
}

/**
 * obs_profile_plan
 * @brief   Chooses CONTINUE, PLAN (side + legs) or NO_PATH (FR8).
 * @param   p_prof profile from obs_profile_build()
 * @param   p_plan result; a leg over its NFR16 limit sets b_was_clamped
 *                 and forces NO_PATH (a clipped plan is never driven)
 * @context obstacle_task (event time, once per scan)
 */
void obs_profile_plan(obs_profile_t const * p_prof, obs_plan_t * p_plan)
{
    /* cast: small positive constants */
    int32_t const  corridor = (int32_t) (OBS_ROBOT_HALF_W_MM
                                         + OBS_SIDE_MARGIN_MM);
    uint32_t const gap_need = 2u * (OBS_ROBOT_HALF_W_MM
                                    + OBS_SIDE_MARGIN_MM);
    uint32_t       shift_l  = 0u;
    uint32_t       shift_r  = 0u;
    bool           b_left   = false;
    bool           b_right  = false;

    p_plan->outcome        = OBS_OUT_CONTINUE;
    p_plan->side           = APP_SIDE_LEFT;
    p_plan->leg_lateral_mm = 0u;
    p_plan->leg_forward_mm = 0u;
    p_plan->b_was_clamped  = false;

    if (p_prof->b_is_valid && (p_prof->x_right_mm >= -corridor)
        && (p_prof->x_left_mm <= corridor))
    {
        /* casts: both >= 0 because the obstacle overlaps the corridor */
        shift_l = (uint32_t) (corridor - p_prof->x_left_mm);
        shift_r = (uint32_t) (corridor + p_prof->x_right_mm); /* cast */
        b_left  = (p_prof->clear_left_mm >= gap_need);
        b_right = (p_prof->clear_right_mm >= gap_need);

        if (b_left && ((!b_right) || (shift_l <= shift_r)))
        {
            p_plan->outcome        = OBS_OUT_PLAN;
            p_plan->side           = APP_SIDE_LEFT;
            p_plan->leg_lateral_mm = shift_l;
        }
        else if (b_right)
        {
            p_plan->outcome        = OBS_OUT_PLAN;
            p_plan->side           = APP_SIDE_RIGHT;
            p_plan->leg_lateral_mm = shift_r;
        }
        else
        {
            p_plan->outcome = OBS_OUT_NO_PATH;
        }
    }

    if (OBS_OUT_PLAN == p_plan->outcome)
    {
        p_plan->leg_forward_mm = p_prof->nearest_mm + OBS_DEPTH_ASSUMED_MM
                                 + OBS_ROBOT_LEN_MM;
        if ((p_plan->leg_lateral_mm > OBS_LEG_LAT_MAX_MM)
            || (p_plan->leg_forward_mm > OBS_LEG_FWD_MAX_MM))
        {
            p_plan->b_was_clamped  = true;
            p_plan->leg_lateral_mm = obs_min_u(p_plan->leg_lateral_mm,
                                               OBS_LEG_LAT_MAX_MM);
            p_plan->leg_forward_mm = obs_min_u(p_plan->leg_forward_mm,
                                               OBS_LEG_FWD_MAX_MM);
            p_plan->outcome        = OBS_OUT_NO_PATH;
        }
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * obs_make_points
 * @brief   Converts each near scan reading to (x, y) and finds the point
 *          with the smallest forward distance.
 * @param   p_scan_mm 25 distances
 * @param   p_pts     25 points out
 * @param   p_best    step of the nearest point, APP_SCAN_STEPS if none
 * @context any
 */
static void obs_make_points(uint16_t const * p_scan_mm, obs_point_t * p_pts,
                            uint32_t * p_best)
{
    uint32_t best_y  = APP_PROFILE_CLEAR_MM;
    uint32_t sin_idx = 0u;
    uint32_t dist    = 0u;
    uint32_t x_mag   = 0u;

    for (uint32_t step = 0u; step < APP_SCAN_STEPS; step++)
    {
        dist                 = (uint32_t) p_scan_mm[step]; /* cast: widen */
        sin_idx              = step + OBS_STEP_IDX_BASE;
        p_pts[step].b_is_near = (dist < OBS_NEAR_MM);
        p_pts[step].x_mm     = 0;
        p_pts[step].y_mm     = APP_PROFILE_CLEAR_MM;

        if (p_pts[step].b_is_near)
        {
            p_pts[step].y_mm = obs_q14_mul(dist, sin_idx);
            if (sin_idx <= OBS_SIN_IDX_90)          /* right half: x >= 0 */
            {
                x_mag            = obs_q14_mul(dist, OBS_SIN_IDX_90 - sin_idx);
                p_pts[step].x_mm = (int32_t) x_mag; /* cast: < 400 */
            }
            else                                    /* left half: x < 0  */
            {
                x_mag            = obs_q14_mul(dist, sin_idx - OBS_SIN_IDX_90);
                p_pts[step].x_mm = -((int32_t) x_mag); /* cast: < 400 */
            }
            if (p_pts[step].y_mm < best_y)
            {
                best_y  = p_pts[step].y_mm;
                *p_best = step;
            }
        }
    }
}

/**
 * obs_q14_mul
 * @brief   distance x sin(5 x sin_idx deg), Q14, unsigned.
 * @param   distance_mm 0..1023
 * @param   sin_idx     0..36
 * @return  mm
 * @context any
 */
static uint32_t obs_q14_mul(uint32_t distance_mm, uint32_t sin_idx)
{
    /* cast: table entries are 0..16384 */
    uint32_t const sin_q14 = (uint32_t) g_obs_sin_q14[sin_idx];

    return ((distance_mm * sin_q14) >> OBS_Q14_SHIFT);
}

/**
 * obs_find_run
 * @brief   Widens [best, best] while the neighbouring steps are near.
 * @param   p_pts points
 * @param   best  step of the nearest point
 * @param   p_lo  first step of the run
 * @param   p_hi  last step of the run
 * @context any
 */
static void obs_find_run(obs_point_t const * p_pts, uint32_t best,
                         uint32_t * p_lo, uint32_t * p_hi)
{
    uint32_t lo = best;
    uint32_t hi = best;

    while ((lo > 0u) && p_pts[lo - 1u].b_is_near)
    {
        lo--;
    }
    while ((hi < OBS_STEP_LAST) && p_pts[hi + 1u].b_is_near)
    {
        hi++;
    }
    *p_lo = lo;
    *p_hi = hi;
}

/**
 * obs_clearances
 * @brief   Smallest lateral gap from the obstacle to any other near point
 *          on each side (APP_PROFILE_CLEAR_MM if there is none).
 * @param   p_pts  points
 * @param   lo     first step of the obstacle run
 * @param   hi     last step of the obstacle run
 * @param   p_prof profile with x_left/x_right set; clearances written
 * @context any
 */
static void obs_clearances(obs_point_t const * p_pts, uint32_t lo,
                           uint32_t hi, obs_profile_t * p_prof)
{
    uint32_t gap_mm = 0u;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        if (p_pts[idx].b_is_near && ((idx < lo) || (idx > hi)))
        {
            if (p_pts[idx].x_mm < p_prof->x_left_mm)
            {
                /* cast: positive, < 800 */
                gap_mm = (uint32_t) (p_prof->x_left_mm - p_pts[idx].x_mm);
                p_prof->clear_left_mm = obs_min_u(p_prof->clear_left_mm,
                                                  gap_mm);
            }
            else if (p_pts[idx].x_mm > p_prof->x_right_mm)
            {
                /* cast: positive, < 800 */
                gap_mm = (uint32_t) (p_pts[idx].x_mm - p_prof->x_right_mm);
                p_prof->clear_right_mm = obs_min_u(p_prof->clear_right_mm,
                                                   gap_mm);
            }
            else
            {
                /* overlaps the obstacle laterally: no gap on either side */
            }
        }
    }
}

/**
 * obs_min_u
 * @brief   Smaller of two unsigned values.
 * @param   value_a first value
 * @param   value_b second value
 * @return  the smaller one
 * @context any
 */
static uint32_t obs_min_u(uint32_t value_a, uint32_t value_b)
{
    return ((value_a < value_b) ? value_a : value_b);
}

/*** end of file ***/
