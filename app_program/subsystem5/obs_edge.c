/**
 * @file    obs_edge.c
 * @brief   1-degree edge pass of the obstacle scan (S5-U83). After the
 *          5-degree points the servo is stepped one degree at a time
 *          outward past each end of the obstacle core. The outermost
 *          in-band echo on each side is the box corner (heard
 *          OBS_BEAM_HALF_DDEG off the servo axis), and an edge only counts
 *          once the next degree out is background. The width is the
 *          distance between the two corners (obs_profile_edge_width()).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    algo
 * @reqs    FR7 (obstacle width within +-10 mm, 24 Nov 2026)
 */

#include "obs_edge.h"          /* own header first (BARR-C 4.3.c) */
#include "cfg_obstacle.h"

/* ---- private constants and types ------------------------------------ */

#define OBS_EDGE_STEP_DEG    (1u)   /* probe spacing                     */
#define OBS_EDGE_GAP_DEG     (2u)   /* extra probe after a non-jump miss */
#define OBS_EDGE_BEYOND_DEG  (3u)   /* first angle past that extra probe */
/* thresholds in 4 mm codes */
#define OBS_EDGE_JUMP_CODES  (OBS_EDGE_JUMP_MM / OBS_MM_STEP)
#define OBS_EDGE_RISE_CODES  (OBS_EDGE_RISE_MM / OBS_MM_STEP)
#define OBS_EDGE_SKEW_CODES  (OBS_EDGE_SKEW_MM / OBS_MM_STEP)

/* ---- private function prototypes ------------------------------------- */

static bool     obs_edge_start_side(obs_edge_t * p_edge, uint32_t side,
                                    obs_edge_probe_t * p_next);
static void     obs_edge_finish_side(obs_edge_t * p_edge, bool b_is_lo,
                                     bool b_is_ok);
static bool     obs_edge_is_jump(uint32_t code, uint32_t edge_code);
static bool     obs_edge_is_coarse_clear(bool b_is_lo, uint32_t edge_deg,
                                         uint32_t edge_code,
                                         uint8_t const * p_scan_codes);
static bool     obs_edge_at_limit(bool b_is_lo, uint32_t deg);
static uint32_t obs_edge_out(bool b_is_lo, uint32_t deg, uint32_t by);
static uint32_t obs_edge_step_deg(uint32_t step);
static uint32_t obs_edge_start_step(uint8_t const * p_scan_codes,
                                    uint32_t end_step, uint32_t mid_step);

/* ---- public functions ------------------------------------------------- */

/**
 * obs_edge_begin
 * @brief   End of the 5-degree points: takes the core and band limit from
 *          the profile, picks each side's start point (one step inside a
 *          core end that may itself be a fringe echo) and starts the pass
 *          on the low-angle side, or the high side if the low end is at
 *          the scan limit.
 * @param   p_edge       state, reset here
 * @param   p_scan_codes 25 scan points, 4 mm codes
 * @param   p_prof       profile from obs_profile_build()
 * @param   p_next       out: first probe
 * @return  true if a probe was started, false if there is nothing to do
 * @context obstacle_task (once per scan), host test
 */
bool obs_edge_begin(obs_edge_t * p_edge, uint8_t const * p_scan_codes,
                    obs_profile_t const * p_prof, obs_edge_probe_t * p_next)
{
    uint32_t lo_step = 0u;
    uint32_t hi_step = 0u;
    uint32_t limit   = 0u;
    bool     b_more  = false;

    p_edge->phase      = OBS_EDGE_OFF;
    p_edge->sub        = 0u;
    p_edge->lo_deg     = 0u;
    p_edge->hi_deg     = 0u;
    p_edge->limit_code = 0u;
    p_edge->lo_code    = 0u;
    p_edge->hi_code    = 0u;
    p_edge->b_ok_lo    = false;
    p_edge->b_ok_hi    = false;
    p_edge->b_grace    = false;

    if (p_prof->b_is_valid)
    {
        limit = (p_prof->core_limit_mm < APP_PROFILE_CLEAR_MM)
                ? p_prof->core_limit_mm : APP_PROFILE_CLEAR_MM;
        p_edge->limit_code = limit / OBS_MM_STEP;
        lo_step = obs_edge_start_step(p_scan_codes, p_prof->core_lo_step,
                                      p_prof->angle_step);
        hi_step = obs_edge_start_step(p_scan_codes, p_prof->core_hi_step,
                                      p_prof->angle_step);
        p_edge->lo_deg  = obs_edge_step_deg(lo_step);
        p_edge->hi_deg  = obs_edge_step_deg(hi_step);
        p_edge->lo_code = p_scan_codes[lo_step];
        p_edge->hi_code = p_scan_codes[hi_step];
        b_more = obs_edge_start_side(p_edge, OBS_EDGE_LO_SIDE, p_next);
    }

    return b_more;
}

/**
 * obs_edge_sample
 * @brief   One probe done (median of the pings at that degree). In band
 *          and not a fringe echo (at most OBS_EDGE_RISE_MM beyond the
 *          echo one degree inward): it becomes the edge and the next
 *          degree out is probed, up to OBS_EDGE_SUB_MAX, then one extra
 *          degree. A jump to background confirms the edge. Anything else
 *          gets one extra degree: a jump there confirms it, and so does a
 *          patchy in-band echo when the next 5-degree point beyond is
 *          background (car 24 Nov); an echo past the band (a face still
 *          receding) does not. An edge at the scan limit never counts.
 * @param   p_edge       state
 * @param   code         probe median, 4 mm code
 * @param   p_scan_codes 25 scan points, 4 mm codes
 * @param   p_next       out: next probe (when true is returned)
 * @return  true while probes remain, false when the pass is done
 * @context obstacle_task, host test
 */
bool obs_edge_sample(obs_edge_t * p_edge, uint32_t code,
                     uint8_t const * p_scan_codes, obs_edge_probe_t * p_next)
{
    bool const     b_is_lo = (OBS_EDGE_LO_SIDE == p_edge->phase);
    uint32_t const prev    = b_is_lo ? p_edge->lo_code : p_edge->hi_code;
    uint32_t const edge    = b_is_lo ? p_edge->lo_deg : p_edge->hi_deg;
    bool const     b_band  = (code <= p_edge->limit_code);
    bool const     b_in    = b_band
                             && (code <= (prev + OBS_EDGE_RISE_CODES));
    uint32_t       probed  = 0u;
    bool           b_done  = false;
    bool           b_ok    = false;
    bool           b_more  = false;

    p_next->b_is_far = false;
    if (p_edge->b_grace)
    {
        p_edge->b_grace = false;
        b_ok   = obs_edge_is_jump(code, prev)
                 || (b_band && obs_edge_is_coarse_clear(b_is_lo, edge, prev,
                                                        p_scan_codes));
        b_done = true;
    }
    else if (b_in)
    {
        probed = obs_edge_out(b_is_lo, edge, OBS_EDGE_STEP_DEG);
        if (b_is_lo)
        {
            p_edge->lo_deg  = probed;
            p_edge->lo_code = code;
        }
        else
        {
            p_edge->hi_deg  = probed;
            p_edge->hi_code = code;
        }
        if (obs_edge_at_limit(b_is_lo, probed))
        {
            b_done = true;                     /* corner not seen */
        }
        else
        {
            /* next degree out; after SUB_MAX it is the extra degree */
            p_edge->b_grace = (p_edge->sub >= OBS_EDGE_SUB_MAX);
            if (!p_edge->b_grace)
            {
                p_edge->sub++;
            }
            p_next->deg = obs_edge_out(b_is_lo, probed, OBS_EDGE_STEP_DEG);
            b_more      = true;
        }
    }
    else if (obs_edge_is_jump(code, prev))
    {
        b_ok   = true;
        b_done = true;
    }
    else if (!obs_edge_at_limit(b_is_lo,
                                obs_edge_out(b_is_lo, edge,
                                             OBS_EDGE_STEP_DEG)))
    {
        p_edge->b_grace = true;                /* one degree further out */
        p_next->deg     = obs_edge_out(b_is_lo, edge, OBS_EDGE_GAP_DEG);
        b_more          = true;
    }
    else
    {
        b_done = true;                         /* no room left */
    }

    if (b_done)
    {
        obs_edge_finish_side(p_edge, b_is_lo, b_ok);
        if (b_is_lo)
        {
            b_more = obs_edge_start_side(p_edge, OBS_EDGE_HI_SIDE, p_next);
        }
    }

    return b_more;
}

/**
 * obs_edge_width
 * @brief   The reported width: the corner-to-corner distance when both
 *          edges were confirmed and the corner ranges are within
 *          OBS_EDGE_SKEW_MM (a box turned further may have its far corner
 *          beyond the band); otherwise 0 = unavailable.
 * @param   p_edge  state after the pass
 * @param   p_pm_mm out: +- error of the 1-degree and 4 mm steps (0 if 0)
 * @return  width in mm, 0 = unavailable
 * @context obstacle_task (once per scan), host test
 */
uint32_t obs_edge_width(obs_edge_t const * p_edge, uint32_t * p_pm_mm)
{
    uint32_t const skew  = (p_edge->lo_code > p_edge->hi_code)
                           ? (p_edge->lo_code - p_edge->hi_code)
                           : (p_edge->hi_code - p_edge->lo_code);
    uint32_t       width = 0u;

    *p_pm_mm = 0u;
    if (p_edge->b_ok_lo && p_edge->b_ok_hi && (skew <= OBS_EDGE_SKEW_CODES))
    {
        width = obs_profile_edge_width(p_edge->lo_deg,
                                       p_edge->lo_code * OBS_MM_STEP,
                                       p_edge->hi_deg,
                                       p_edge->hi_code * OBS_MM_STEP,
                                       p_pm_mm);
    }

    return width;
}

/* ---- private functions ------------------------------------------------ */

/**
 * obs_edge_start_side
 * @brief   Starts the pass on one side; a low side whose start point is
 *          at the scan limit is skipped for the high side, and a high side
 *          at the limit ends the pass (that edge stays unconfirmed).
 * @param   p_edge state
 * @param   side   OBS_EDGE_LO_SIDE or OBS_EDGE_HI_SIDE
 * @param   p_next out: first probe of the side (a long swing)
 * @return  true if a probe was started, false if the pass is done
 * @context any
 */
static bool obs_edge_start_side(obs_edge_t * p_edge, uint32_t side,
                                obs_edge_probe_t * p_next)
{
    uint32_t side_now = side;
    bool     b_is_lo  = false;
    uint32_t start    = 0u;
    bool     b_more   = false;

    /* a low side that starts at the limit goes straight to the high side */
    if ((OBS_EDGE_LO_SIDE == side_now)
        && obs_edge_at_limit(true, p_edge->lo_deg))
    {
        side_now = OBS_EDGE_HI_SIDE;
    }
    b_is_lo = (OBS_EDGE_LO_SIDE == side_now);
    start   = b_is_lo ? p_edge->lo_deg : p_edge->hi_deg;

    p_edge->phase = OBS_EDGE_OFF;
    if (!obs_edge_at_limit(b_is_lo, start))
    {
        p_edge->phase    = side_now;
        p_edge->sub      = 1u;
        p_next->deg      = obs_edge_out(b_is_lo, start, OBS_EDGE_STEP_DEG);
        p_next->b_is_far = true;
        b_more           = true;
    }

    return b_more;
}

/**
 * obs_edge_finish_side
 * @brief   Records whether one side's edge was confirmed.
 * @param   p_edge  state
 * @param   b_is_lo true = low-angle side
 * @param   b_is_ok true = confirmed
 * @context any
 */
static void obs_edge_finish_side(obs_edge_t * p_edge, bool b_is_lo,
                                 bool b_is_ok)
{
    if (b_is_lo)
    {
        p_edge->b_ok_lo = b_is_ok;
    }
    else
    {
        p_edge->b_ok_hi = b_is_ok;
        p_edge->phase   = OBS_EDGE_OFF;
    }
}

/**
 * obs_edge_is_jump
 * @brief   True if a reading past an edge is background: clear, or more
 *          than OBS_EDGE_JUMP_MM beyond the edge echo (a face that keeps
 *          receding changes far less per degree).
 * @param   code      reading past the edge, 4 mm code
 * @param   edge_code echo at the edge, 4 mm code
 * @return  true = background
 * @context any
 */
static bool obs_edge_is_jump(uint32_t code, uint32_t edge_code)
{
    return (code > (edge_code + OBS_EDGE_JUMP_CODES));
}

/**
 * obs_edge_is_coarse_clear
 * @brief   True if the 5-degree scan point just beyond the extra probe
 *          (OBS_EDGE_BEYOND_DEG or more past the edge) is background.
 *          Past the 30/150 deg limit there is no such point.
 * @param   b_is_lo      true = low-angle side
 * @param   edge_deg     edge angle, deg
 * @param   edge_code    echo at the edge, 4 mm code
 * @param   p_scan_codes 25 scan points, 4 mm codes
 * @return  true = background there
 * @context any
 */
static bool obs_edge_is_coarse_clear(bool b_is_lo, uint32_t edge_deg,
                                     uint32_t edge_code,
                                     uint8_t const * p_scan_codes)
{
    bool     b_clear = false;
    uint32_t step    = 0u;

    if (b_is_lo
        && (edge_deg >= (APP_SCAN_ANGLE_MIN_DEG + OBS_EDGE_BEYOND_DEG)))
    {
        /* largest 5-degree point at or below edge - BEYOND */
        step    = (edge_deg - OBS_EDGE_BEYOND_DEG - APP_SCAN_ANGLE_MIN_DEG)
                  / APP_SCAN_ANGLE_STEP_DEG;
        b_clear = obs_edge_is_jump(p_scan_codes[step], edge_code);
    }
    else if ((!b_is_lo)
             && ((edge_deg + OBS_EDGE_BEYOND_DEG) <= OBS_SCAN_LAST_DEG))
    {
        /* smallest 5-degree point at or above edge + BEYOND */
        step    = ((edge_deg + OBS_EDGE_BEYOND_DEG) - APP_SCAN_ANGLE_MIN_DEG
                   + (APP_SCAN_ANGLE_STEP_DEG - 1u))
                  / APP_SCAN_ANGLE_STEP_DEG;
        b_clear = (step < APP_SCAN_STEPS)
                  && obs_edge_is_jump(p_scan_codes[step], edge_code);
    }
    else
    {
        /* no 5-degree point beyond: not confirmed */
    }

    return b_clear;
}

/**
 * obs_edge_at_limit
 * @brief   True if an angle is at (or past) the scan limit on its side.
 * @param   b_is_lo true = low-angle side (limit 30 deg)
 * @param   deg     angle, deg
 * @return  true at the limit
 * @context any
 */
static bool obs_edge_at_limit(bool b_is_lo, uint32_t deg)
{
    return b_is_lo ? (deg <= APP_SCAN_ANGLE_MIN_DEG)
                   : (deg >= OBS_SCAN_LAST_DEG);
}

/**
 * obs_edge_out
 * @brief   An angle moved outward (away from the obstacle) on a side.
 * @param   b_is_lo true = low-angle side (outward = smaller angles)
 * @param   deg     angle, deg (> by on the low side)
 * @param   by      degrees
 * @return  the moved angle
 * @context any
 */
static uint32_t obs_edge_out(bool b_is_lo, uint32_t deg, uint32_t by)
{
    return b_is_lo ? (deg - by) : (deg + by);
}

/**
 * obs_edge_step_deg
 * @brief   Servo angle of a 5-degree scan step.
 * @param   step 0..APP_SCAN_STEPS - 1
 * @return  deg
 * @context any
 */
static uint32_t obs_edge_step_deg(uint32_t step)
{
    return APP_SCAN_ANGLE_MIN_DEG + (APP_SCAN_ANGLE_STEP_DEG * step);
}

/**
 * obs_edge_start_step
 * @brief   A core end that rises more than OBS_EDGE_RISE_MM past the
 *          point inside it may be a fringe echo: start from that inner
 *          point (never past the plateau middle).
 * @param   p_scan_codes 25 scan points, 4 mm codes
 * @param   end_step     core end (first or last core step)
 * @param   mid_step     plateau middle (obs_profile angle_step)
 * @return  the step the pass starts from
 * @context any
 */
static uint32_t obs_edge_start_step(uint8_t const * p_scan_codes,
                                    uint32_t end_step, uint32_t mid_step)
{
    uint32_t start = end_step;
    uint32_t inner = end_step;

    if (end_step < mid_step)
    {
        inner = end_step + 1u;
    }
    else if (end_step > mid_step)
    {
        inner = end_step - 1u;
    }
    else
    {
        /* the end is the middle: nothing inside */
    }
    if ((inner != end_step)
        && (p_scan_codes[end_step] > (p_scan_codes[inner]
                                      + OBS_EDGE_RISE_CODES)))
    {
        start = inner;
    }

    return start;
}

/*** end of file ***/
