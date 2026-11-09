/**
 * @file    obs_profile.c
 * @brief   Turns the 25-point servo scan into an obstacle profile and a
 *          bypass plan. Pure C: integer trig only, compiles on a laptop.
 *          The obstacle is the nearest run of near points that reaches
 *          into the driving corridor (review R02). Its width is a
 *          beam-corrected estimate around the shortest echo, kept within
 *          the professor's 10..20 cm (S5-U45).
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
#define OBS_Q14_ONE        (16384)  /* 1.0 in Q14 (signed division) */
#define OBS_STEP_LAST      (APP_SCAN_STEPS - 1u)

typedef struct
{
    bool     b_is_near;
    int32_t  x_mm;       /* + = right of the robot's centre line */
    uint32_t y_mm;       /* forward distance                     */
    uint32_t r_mm;       /* measured range (S5-U44)              */
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
                                obs_point_t * p_pts);
static uint32_t obs_pick_blocking(obs_point_t const * p_pts);
static void     obs_find_run(obs_point_t const * p_pts, uint32_t best,
                             uint32_t * p_lo, uint32_t * p_hi);
static uint32_t obs_plateau_mid(obs_point_t const * p_pts, uint32_t best,
                                uint32_t lo, uint32_t hi);
static void     obs_find_core(obs_point_t const * p_pts, uint32_t best,
                              uint32_t lo, uint32_t hi,
                              uint32_t * p_core_lo, uint32_t * p_core_hi);
static void     obs_span(obs_point_t const * p_pts, uint32_t first,
                         uint32_t last, int32_t * p_x_l, int32_t * p_x_r);
static void     obs_estimate(obs_point_t const * p_pts, uint32_t best,
                             uint32_t core_lo, uint32_t core_hi,
                             obs_profile_t * p_prof);
static void     obs_clearances(obs_point_t const * p_pts, uint32_t inferred,
                               uint32_t best, uint32_t lo, uint32_t hi,
                               obs_profile_t * p_prof);
static bool     obs_is_smear(obs_point_t const * p_pts, uint32_t idx,
                             uint32_t inferred, uint32_t best,
                             int32_t x_edge, uint32_t y_edge);
static uint32_t obs_min_u(uint32_t value_a, uint32_t value_b);
static bool     obs_is_joined(obs_point_t const * p_pts, uint32_t step);

/* ---- public functions ------------------------------------------------- */

/**
 * obs_profile_build
 * @brief   Finds the obstacle (the run of contiguous near points that
 *          reaches into the corridor and holds the shortest echo), its
 *          measured span, a beam-corrected width estimate within
 *          OBS_WIDTH_MIN_MM..OBS_WIDTH_MAX_MM (S5-U45) and the free gaps
 *          to every other near point on each side (S5-U46).
 * @param   p_scan_mm 25 distances (mm), step n = 30 + 5n degrees
 * @param   inferred  bit n set: step n was filled, not measured (S5-U49)
 * @param   p_prof    result; b_is_valid false if no run blocks
 * @context obstacle_task (event time, once per scan)
 */
void obs_profile_build(uint16_t const * p_scan_mm, uint32_t inferred,
                       obs_profile_t * p_prof)
{
    obs_point_t pts[APP_SCAN_STEPS];
    uint32_t    best    = APP_SCAN_STEPS;   /* "none" */
    uint32_t    lo      = 0u;
    uint32_t    hi      = 0u;
    uint32_t    core_lo = 0u;
    uint32_t    core_hi = 0u;
    int32_t     raw_l   = 0;
    int32_t     raw_r   = 0;

    p_prof->b_is_valid     = false;
    p_prof->angle_step     = 0u;
    p_prof->nearest_mm     = APP_PROFILE_CLEAR_MM;
    p_prof->x_left_mm      = 0;
    p_prof->x_right_mm     = 0;
    p_prof->width_mm       = APP_PROFILE_CLEAR_MM;
    p_prof->raw_width_mm   = APP_PROFILE_CLEAR_MM;
    p_prof->clear_left_mm  = APP_PROFILE_CLEAR_MM;
    p_prof->clear_right_mm = APP_PROFILE_CLEAR_MM;
    p_prof->block_step_left  = APP_SCAN_STEPS;
    p_prof->block_step_right = APP_SCAN_STEPS;
    p_prof->plan_left_mm     = 0;
    p_prof->plan_right_mm    = 0;
    p_prof->ext_step_left    = APP_SCAN_STEPS;
    p_prof->ext_step_right   = APP_SCAN_STEPS;
    p_prof->b_extended_left  = false;
    p_prof->b_extended_right = false;

    obs_make_points(p_scan_mm, pts);
    best = obs_pick_blocking(pts);

    if (best < APP_SCAN_STEPS)
    {
        obs_find_run(pts, best, &lo, &hi);
        best = obs_plateau_mid(pts, best, lo, hi);           /* S5-U48 */
        obs_span(pts, lo, hi, &raw_l, &raw_r);
        obs_find_core(pts, best, lo, hi, &core_lo, &core_hi);
        p_prof->b_is_valid   = true;
        p_prof->angle_step   = best;
        p_prof->nearest_mm   = pts[best].y_mm;
        /* cast: raw_r >= raw_l, difference < 2 x OBS_NEAR_MM */
        p_prof->raw_width_mm = obs_min_u((uint32_t) (raw_r - raw_l),
                                         APP_PROFILE_CLEAR_MM);
        obs_estimate(pts, best, core_lo, core_hi, p_prof);
        obs_clearances(pts, inferred, best, core_lo, core_hi, p_prof);
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
    /* S5-U53: the bypass clears plan_left/right_mm (the estimate plus
       OBS_EST_MARGIN_MM, widened to every return that may be the box)
       by the half width plus OBS_SIDE_MARGIN_MM, and the free gap beyond
       it must hold the car plus OBS_SIDE_MARGIN_MM on both sides     */
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

    if (p_prof->b_is_valid)    /* only a run that blocks the corridor */
    {
        /* The estimate always holds the shortest echo, but that echo may
           lie just outside the corridor (the run blocks through another
           point): a shift below 0 is 0, then OBS_LEG_MIN_MM below.
           casts: positive differences of values < 2 x OBS_NEAR_MM     */
        shift_l = (corridor > p_prof->plan_left_mm)
                  ? (uint32_t) (corridor - p_prof->plan_left_mm) : 0u;
        shift_r = ((corridor + p_prof->plan_right_mm) > 0)
                  ? (uint32_t) (corridor + p_prof->plan_right_mm) : 0u;
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
        /* A broad planar face keeps the normal limit. Only use the
           extra travel when the chosen extent comes from a measured
           echo projecting in front of the face and no other return
           limits that lane. Its full extent still gets the margin. */
        bool const b_extended = (APP_SIDE_LEFT == p_plan->side)
            ? (p_prof->b_extended_left
               && (APP_PROFILE_CLEAR_MM == p_prof->clear_left_mm))
            : (p_prof->b_extended_right
               && (APP_PROFILE_CLEAR_MM == p_prof->clear_right_mm));
        uint32_t const limit_mm = b_extended ? OBS_LEG_LAT_MAX_MM
                                             : OBS_LEG_LAT_BASE_MM;

        /* a run that only touches the corridor edge gives shift 0, which
           motion_move_forward() would reject (review V13-04)            */
        if (p_plan->leg_lateral_mm < OBS_LEG_MIN_MM)
        {
            p_plan->leg_lateral_mm = OBS_LEG_MIN_MM;
        }
        p_plan->leg_forward_mm = p_prof->nearest_mm + OBS_DEPTH_ASSUMED_MM
                                 + OBS_ROBOT_LEN_MM;
        if ((p_plan->leg_lateral_mm > limit_mm)
            || (p_plan->leg_forward_mm > OBS_LEG_FWD_MAX_MM))
        {
            p_plan->b_was_clamped  = true;
            p_plan->leg_lateral_mm = obs_min_u(p_plan->leg_lateral_mm,
                                               limit_mm);
            p_plan->leg_forward_mm = obs_min_u(p_plan->leg_forward_mm,
                                               OBS_LEG_FWD_MAX_MM);
            p_plan->outcome        = OBS_OUT_NO_PATH;
        }
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * obs_make_points
 * @brief   Converts each near scan reading to (x, y).
 * @param   p_scan_mm 25 distances
 * @param   p_pts     25 points out
 * @context any
 */
static void obs_make_points(uint16_t const * p_scan_mm, obs_point_t * p_pts)
{
    uint32_t sin_idx = 0u;
    int32_t  dist    = 0;
    int32_t  cos_q14 = 0;

    for (uint32_t step = 0u; step < APP_SCAN_STEPS; step++)
    {
        sin_idx               = step + OBS_STEP_IDX_BASE;
        dist                  = (int32_t) p_scan_mm[step]; /* cast: widen */
        p_pts[step].b_is_near = (p_scan_mm[step] < OBS_NEAR_MM);
        p_pts[step].x_mm      = 0;
        p_pts[step].y_mm      = APP_PROFILE_CLEAR_MM;
        p_pts[step].r_mm      = (uint32_t) p_scan_mm[step]; /* cast: widen */

        if (p_pts[step].b_is_near)
        {
            if (sin_idx <= OBS_SIN_IDX_90)          /* right half: cos >= 0 */
            {
                cos_q14 = g_obs_sin_q14[OBS_SIN_IDX_90 - sin_idx];
            }
            else                                    /* left half: cos < 0  */
            {
                cos_q14 = -g_obs_sin_q14[sin_idx - OBS_SIN_IDX_90];
            }
            /* signed division by a power of two, truncates toward zero
               (never >> on a signed value, review R19); |d x cos| < 2^23 */
            p_pts[step].x_mm = (dist * cos_q14) / OBS_Q14_ONE;
            /* cast: d >= 0 and sin >= 0, so the quotient is >= 0 */
            p_pts[step].y_mm = (uint32_t) ((dist * g_obs_sin_q14[sin_idx])
                                           / OBS_Q14_ONE);
        }
    }
}

/**
 * obs_pick_blocking
 * @brief   Splits the near points into runs (one object each; a range
 *          jump > OBS_RUN_JUMP_MM between neighbours ends a run, S5-U44)
 *          and picks the shortest echo of the runs that block the
 *          corridor (-c <= x <= c for at least one point of the run).
 *          Shortest range, not forward distance: a beam-edge echo far
 *          off-axis has a small forward distance although it comes from
 *          the obstacle straight ahead (S5-U45).
 * @param   p_pts points
 * @return  step of that echo; APP_SCAN_STEPS if no run blocks
 * @context any
 */
static uint32_t obs_pick_blocking(obs_point_t const * p_pts)
{
    /* cast: small positive constants */
    int32_t const corridor = (int32_t) (OBS_ROBOT_HALF_W_MM
                                        + OBS_SIDE_MARGIN_MM);
    uint32_t      best     = APP_SCAN_STEPS;
    uint32_t      best_r   = APP_PROFILE_CLEAR_MM;
    uint32_t      run_best = APP_SCAN_STEPS;
    bool          b_blocks = false;

    for (uint32_t step = 0u; step < APP_SCAN_STEPS; step++)
    {
        if (p_pts[step].b_is_near)
        {
            if ((APP_SCAN_STEPS == run_best)
                || (p_pts[step].r_mm < p_pts[run_best].r_mm))
            {
                run_best = step;                /* shortest echo of run */
            }
            b_blocks = b_blocks || ((p_pts[step].x_mm >= -corridor)
                                    && (p_pts[step].x_mm <= corridor));
        }
        if ((APP_SCAN_STEPS != run_best)
            && ((OBS_STEP_LAST == step) || (!obs_is_joined(p_pts, step))))
        {
            if (b_blocks && (p_pts[run_best].r_mm < best_r))
            {
                best   = run_best;              /* run ends here */
                best_r = p_pts[run_best].r_mm;
            }
            run_best = APP_SCAN_STEPS;
            b_blocks = false;
        }
    }

    return best;
}

/**
 * obs_find_run
 * @brief   Widens [best, best] while the neighbouring steps belong to the
 *          same object (near, no range jump; S5-U44).
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

    while ((lo > 0u) && obs_is_joined(p_pts, lo - 1u))
    {
        lo--;
    }
    while ((hi < OBS_STEP_LAST) && obs_is_joined(p_pts, hi))
    {
        hi++;
    }
    *p_lo = lo;
    *p_hi = hi;
}

/**
 * obs_clearances
 * @brief   S5-U53 (review 2026-11-09): no return beside the estimate is
 *          ignored. Pass 1: the extent the bypass must clear starts at the
 *          estimate +- OBS_EST_MARGIN_MM and is widened to every core
 *          return that may be the box itself (obs_is_smear()). Pass 2:
 *          every other near return the bypass drives past (forward
 *          distance <= nearest + OBS_DEPTH_ASSUMED_MM + OBS_ROBOT_LEN_MM,
 *          S5-U44) is a separate object: the smallest lateral gap from the
 *          extent to it on each side, and its step (S5-U46, S5-U49).
 * @param   p_pts    points
 * @param   inferred bit n set: step n was not measured
 * @param   best     step of the obstacle's direction (shortest echo)
 * @param   lo       first step of the obstacle core
 * @param   hi       last step of the obstacle core
 * @param   p_prof   profile with the estimate set; extents and
 *                   clearances written
 * @context any
 */
static void obs_clearances(obs_point_t const * p_pts, uint32_t inferred,
                           uint32_t best, uint32_t lo, uint32_t hi,
                           obs_profile_t * p_prof)
{
    uint32_t const reach_mm = p_prof->nearest_mm + OBS_DEPTH_ASSUMED_MM
                              + OBS_ROBOT_LEN_MM;
    /* cast: small positive constant */
    int32_t const  margin   = (int32_t) OBS_EST_MARGIN_MM;
    uint32_t       gap_mm   = 0u;
    bool           b_box    = false;

    p_prof->plan_left_mm  = p_prof->x_left_mm - margin;
    p_prof->plan_right_mm = p_prof->x_right_mm + margin;

    for (uint32_t idx = lo; idx <= hi; idx++)          /* pass 1 */
    {
        if (p_pts[idx].y_mm <= reach_mm)
        {
            if ((p_pts[idx].x_mm < p_prof->plan_left_mm)
                && obs_is_smear(p_pts, idx, inferred, best,
                                p_prof->x_left_mm, p_prof->nearest_mm))
            {
                p_prof->plan_left_mm    = p_pts[idx].x_mm;
                p_prof->ext_step_left   = idx;
                p_prof->b_extended_left =
                    (p_pts[idx].y_mm + OBS_EXT_ECHO_LEAD_MM
                     <= p_prof->nearest_mm);
            }
            else if ((p_pts[idx].x_mm > p_prof->plan_right_mm)
                     && obs_is_smear(p_pts, idx, inferred, best,
                                     p_prof->x_right_mm,
                                     p_prof->nearest_mm))
            {
                p_prof->plan_right_mm    = p_pts[idx].x_mm;
                p_prof->ext_step_right   = idx;
                p_prof->b_extended_right =
                    (p_pts[idx].y_mm + OBS_EXT_ECHO_LEAD_MM
                     <= p_prof->nearest_mm);
            }
            else
            {
                /* inside the extent, or a separate object (pass 2) */
            }
        }
    }

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)   /* pass 2 */
    {
        b_box = (idx >= lo) && (idx <= hi)
                && (((p_pts[idx].x_mm >= p_prof->plan_left_mm)
                     && (p_pts[idx].x_mm <= p_prof->plan_right_mm))
                    || obs_is_smear(p_pts, idx, inferred, best,
                                    (p_pts[idx].x_mm < p_prof->x_left_mm)
                                        ? p_prof->x_left_mm
                                        : p_prof->x_right_mm,
                                    p_prof->nearest_mm));
        if (p_pts[idx].b_is_near && (p_pts[idx].y_mm <= reach_mm)
            && (!b_box))
        {
            if (p_pts[idx].x_mm < p_prof->plan_left_mm)
            {
                /* cast: positive, < 1400 */
                gap_mm = (uint32_t) (p_prof->plan_left_mm - p_pts[idx].x_mm);
                if (gap_mm < p_prof->clear_left_mm)
                {
                    p_prof->clear_left_mm   = gap_mm;
                    p_prof->block_step_left = idx;
                }
            }
            else if (p_pts[idx].x_mm > p_prof->plan_right_mm)
            {
                /* cast: positive, < 1400 */
                gap_mm = (uint32_t) (p_pts[idx].x_mm - p_prof->plan_right_mm);
                if (gap_mm < p_prof->clear_right_mm)
                {
                    p_prof->clear_right_mm   = gap_mm;
                    p_prof->block_step_right = idx;
                }
            }
            else
            {
                /* behind the obstacle, within its lateral extent: not
                   in either bypass lane (as before S5-U53)            */
            }
        }
    }
}

/**
 * obs_is_smear
 * @brief   True if the obstacle itself could have produced this return
 *          through the edge of the beam (S5-U49): the step was measured,
 *          its range is within OBS_SMEAR_RANGE_MM of the obstacle's
 *          shortest echo, and its direction is within OBS_SMEAR_DEG of
 *          the estimated edge (x_edge, y_edge):
 *          |p x e| <= sin(OBS_SMEAR_DEG) x |p| x |e|, squared.
 * @param   p_pts    points
 * @param   idx      step of the return (near)
 * @param   inferred bit n set: step n was not measured
 * @param   best     step of the obstacle's shortest echo
 * @param   x_edge   lateral position of the estimated edge on that side
 * @param   y_edge   its forward distance (the obstacle's nearest_mm)
 * @return  true = excused as beam smear
 * @context any
 */
static bool obs_is_smear(obs_point_t const * p_pts, uint32_t idx,
                         uint32_t inferred, uint32_t best,
                         int32_t x_edge, uint32_t y_edge)
{
    bool b_is_smear = false;

    if ((0u == (inferred & (1u << idx)))
        && (p_pts[idx].r_mm <= (p_pts[best].r_mm + OBS_SMEAR_RANGE_MM)))
    {
        /* casts: |x|, y, r < 1300 mm; products < 2^42, fit int64_t /
           uint64_t                                                    */
        int64_t const  x_p   = (int64_t) p_pts[idx].x_mm;
        int64_t const  y_p   = (int64_t) p_pts[idx].y_mm;
        int64_t const  x_e   = (int64_t) x_edge;
        int64_t const  y_e   = (int64_t) y_edge;
        int64_t const  cross = (x_p * y_e) - (y_p * x_e);
        uint64_t const c_sq  = (uint64_t) (cross * cross);
        uint64_t const r_sq  = (uint64_t) p_pts[idx].r_mm
                               * (uint64_t) p_pts[idx].r_mm;
        uint64_t const e_sq  = (uint64_t) ((x_e * x_e) + (y_e * y_e));

        b_is_smear = ((c_sq * (uint64_t) (OBS_SMEAR_SIN_DEN
                                          * OBS_SMEAR_SIN_DEN))
                      <= ((r_sq * e_sq) * (uint64_t) (OBS_SMEAR_SIN_NUM
                                                      * OBS_SMEAR_SIN_NUM)));
    }

    return b_is_smear;
}

/**
 * obs_plateau_mid
 * @brief   The shortest echo often reads the same over several steps
 *          (beam smear; on-car 2026-11-09: 252 mm from 65 to 100 deg).
 *          Returns the middle step of the contiguous steps within the run
 *          whose range is at most OBS_PLATEAU_MM above the shortest
 *          (S5-U48), the best guess for the obstacle's direction.
 * @param   p_pts points
 * @param   best  step of the shortest echo
 * @param   lo    first step of the run
 * @param   hi    last step of the run
 * @return  middle step of the plateau (rounded down)
 * @context any
 */
static uint32_t obs_plateau_mid(obs_point_t const * p_pts, uint32_t best,
                                uint32_t lo, uint32_t hi)
{
    uint32_t const limit_mm = p_pts[best].r_mm + OBS_PLATEAU_MM;
    uint32_t       first    = best;
    uint32_t       last     = best;

    while ((first > lo) && (p_pts[first - 1u].r_mm <= limit_mm))
    {
        first--;
    }
    while ((last < hi) && (p_pts[last + 1u].r_mm <= limit_mm))
    {
        last++;
    }

    return ((first + last) / 2u);
}

/**
 * obs_find_core
 * @brief   Background subtraction with hysteresis (S5-U45): from the
 *          shortest echo, widens the core within the run [lo, hi] over
 *          steps whose range is at most OBS_CORE_BAND_MM longer; up to
 *          OBS_CORE_GAP_STEPS out-of-band steps in a row are bridged if
 *          an in-band step follows.
 * @param   p_pts     points
 * @param   best      step of the shortest echo
 * @param   lo        first step of the run
 * @param   hi        last step of the run
 * @param   p_core_lo first step of the core
 * @param   p_core_hi last step of the core
 * @context any
 */
static void obs_find_core(obs_point_t const * p_pts, uint32_t best,
                          uint32_t lo, uint32_t hi,
                          uint32_t * p_core_lo, uint32_t * p_core_hi)
{
    uint32_t const limit_mm = p_pts[best].r_mm + OBS_CORE_BAND_MM;
    uint32_t       core_lo  = best;
    uint32_t       core_hi  = best;
    uint32_t       idx      = best;
    uint32_t       gap      = 0u;

    while ((idx > lo) && (gap <= OBS_CORE_GAP_STEPS))
    {
        idx--;
        if (p_pts[idx].r_mm <= limit_mm)
        {
            core_lo = idx;
            gap     = 0u;
        }
        else
        {
            gap++;
        }
    }

    idx = best;
    gap = 0u;
    while ((idx < hi) && (gap <= OBS_CORE_GAP_STEPS))
    {
        idx++;
        if (p_pts[idx].r_mm <= limit_mm)
        {
            core_hi = idx;
            gap     = 0u;
        }
        else
        {
            gap++;
        }
    }

    *p_core_lo = core_lo;
    *p_core_hi = core_hi;
}

/**
 * obs_span
 * @brief   Smallest and largest x of the points first..last (all near).
 * @param   p_pts points
 * @param   first first step
 * @param   last  last step (>= first)
 * @param   p_x_l smallest x out
 * @param   p_x_r largest x out
 * @context any
 */
static void obs_span(obs_point_t const * p_pts, uint32_t first,
                     uint32_t last, int32_t * p_x_l, int32_t * p_x_r)
{
    int32_t x_l = p_pts[first].x_mm;
    int32_t x_r = p_pts[first].x_mm;

    for (uint32_t idx = first; idx <= last; idx++)
    {
        if (p_pts[idx].x_mm < x_l)
        {
            x_l = p_pts[idx].x_mm;
        }
        if (p_pts[idx].x_mm > x_r)
        {
            x_r = p_pts[idx].x_mm;
        }
    }

    *p_x_l = x_l;
    *p_x_r = x_r;
}

/**
 * obs_estimate
 * @brief   Beam correction (S5-U45): each end of the core is pulled in by
 *          OBS_BEAM_HALF_STEPS (never past the shortest echo); the span of
 *          what remains is the width. Outside OBS_WIDTH_MIN_MM ..
 *          OBS_WIDTH_MAX_MM it is set to the nearer limit, centred on the
 *          shortest echo.
 * @param   p_pts   points
 * @param   best    step of the shortest echo
 * @param   core_lo first step of the core (<= best)
 * @param   core_hi last step of the core (>= best)
 * @param   p_prof  x_left_mm, x_right_mm and width_mm written
 * @context any
 */
static void obs_estimate(obs_point_t const * p_pts, uint32_t best,
                         uint32_t core_lo, uint32_t core_hi,
                         obs_profile_t * p_prof)
{
    uint32_t est_lo = core_lo + OBS_BEAM_HALF_STEPS;
    uint32_t est_hi = best;
    uint32_t width  = 0u;
    uint32_t target = 0u;
    int32_t  x_l    = 0;
    int32_t  x_r    = 0;

    if (est_lo > best)
    {
        est_lo = best;
    }
    if (core_hi >= (best + OBS_BEAM_HALF_STEPS))
    {
        est_hi = core_hi - OBS_BEAM_HALF_STEPS;
    }
    obs_span(p_pts, est_lo, est_hi, &x_l, &x_r);
    width  = (uint32_t) (x_r - x_l);    /* cast: x_r >= x_l, < 1300 */
    target = width;
    if (width < OBS_WIDTH_MIN_MM)
    {
        target = OBS_WIDTH_MIN_MM;
    }
    else if (width > OBS_WIDTH_MAX_MM)
    {
        target = OBS_WIDTH_MAX_MM;
    }
    else
    {
        /* measured estimate is plausible: keep it */
    }

    if (target != width)
    {
        /* Centred on the shortest echo, not on the span: an object beside
           the obstacle truncates the beam smear on one side only (S5-U46).
           Signed division by 2 truncates toward zero (review R19).     */
        x_l = p_pts[best].x_mm - ((int32_t) target / 2);  /* cast: <= 200 */
        x_r = x_l + (int32_t) target;                      /* cast: <= 200 */
    }

    p_prof->x_left_mm  = x_l;
    p_prof->x_right_mm = x_r;
    p_prof->width_mm   = target;
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

/**
 * obs_is_joined
 * @brief   True if step and step + 1 are both near and their ranges differ
 *          by at most OBS_RUN_JUMP_MM: the same object. A box 240 mm
 *          ahead and a wall 640 mm away beside it are two objects
 *          (on-car 2026-11-06, S5-U44).
 * @param   p_pts points
 * @param   step  0 .. OBS_STEP_LAST - 1
 * @return  true = same run
 * @context any
 */
static bool obs_is_joined(obs_point_t const * p_pts, uint32_t step)
{
    uint32_t const r_a = p_pts[step].r_mm;
    uint32_t const r_b = p_pts[step + 1u].r_mm;
    uint32_t const gap = (r_a > r_b) ? (r_a - r_b) : (r_b - r_a);

    return (p_pts[step].b_is_near && p_pts[step + 1u].b_is_near
            && (gap <= OBS_RUN_JUMP_MM));
}

/*** end of file ***/
