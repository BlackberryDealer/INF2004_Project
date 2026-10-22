/**
 * @file    line_track.c
 * @brief   Pure line normalisation, steering error, and junction classifier.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 * @reqs    FR1, FR19, NFR1, NFR4
 */

#include "line_track.h"
#include <stddef.h>
#include "cfg_line.h"

/* ---- private constants and types ------------------------------------ */

#define LINE_RECIP_SHIFT (16u)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static void line_track_jct_set_barcode_side(line_track_jct_t * p_jct);
static void line_track_jct_idle(line_track_jct_t * p_jct,
                                line_track_jct_in_t const * p_in);
static void line_track_jct_candidate(line_track_jct_t * p_jct,
                                     line_track_jct_in_t const * p_in);
static uint32_t line_track_jct_confirm(line_track_jct_t * p_jct,
                                       line_track_jct_in_t const * p_in);

/* ---- public functions ------------------------------------------------- */

/**
 * line_track_cal_finish
 * @brief   Validates IR calibration min/max spans and precomputes Q16
 *          reciprocals with ordered endpoints (lowest in white_*, highest
 *          in black_*).
 * @param   p_cal  Output calibration structure.
 * @param   min_l  Minimum raw ADC seen on left channel (0..4095).
 * @param   max_l  Maximum raw ADC seen on left channel (0..4095).
 * @param   min_r  Minimum raw ADC seen on right channel (0..4095).
 * @param   max_r  Maximum raw ADC seen on right channel (0..4095).
 * @return  true if both channel spans >= LINE_CAL_MIN_SPAN, else false.
 * @context line_task (calibration end, event time)
 */
bool line_track_cal_finish(line_track_cal_t * p_cal,
                           uint32_t min_l, uint32_t max_l,
                           uint32_t min_r, uint32_t max_r)
{
    bool     b_is_ok = false;
    uint32_t lo_l    = min_l;
    uint32_t hi_l    = max_l;
    uint32_t lo_r    = min_r;
    uint32_t hi_r    = max_r;
    uint32_t span_l  = 0u;
    uint32_t span_r  = 0u;

    if (min_l > max_l)
    {
        lo_l = max_l;
        hi_l = min_l;
    }
    else
    {
        /* Left endpoints already ordered. */
    }

    if (min_r > max_r)
    {
        lo_r = max_r;
        hi_r = min_r;
    }
    else
    {
        /* Right endpoints already ordered. */
    }

    span_l = hi_l - lo_l;
    span_r = hi_r - lo_r;

    if ((NULL != p_cal) && (span_l >= LINE_CAL_MIN_SPAN) &&
        (span_r >= LINE_CAL_MIN_SPAN))
    {
        p_cal->white_l = lo_l;
        p_cal->black_l = hi_l;
        p_cal->white_r = lo_r;
        p_cal->black_r = hi_r;
        /* Rounded-up reciprocal so the top endpoint reaches 1000 (V13-09). */
        p_cal->recip_l_q16 =
            (((LINE_NORM_MAX << LINE_RECIP_SHIFT) + span_l) - 1u) / span_l;
        p_cal->recip_r_q16 =
            (((LINE_NORM_MAX << LINE_RECIP_SHIFT) + span_r) - 1u) / span_r;
        b_is_ok = true;
    }
    else
    {
        /* Calibration rejected: null pointer or insufficient contrast span. */
    }

    return b_is_ok;
}

/**
 * line_track_normalise
 * @brief   Normalises a raw 12-bit ADC reading into 0 (white) .. 1000 (black)
 *          without runtime division.
 * @param   raw        Raw ADC code (0..4095).
 * @param   white      Ordered low endpoint lo (or white level).
 * @param   black      Ordered high endpoint hi (or black level).
 * @param   recip_q16  Precomputed ((1000 << 16) + span - 1) / span.
 * @return  Normalised reflectance 0..1000 (0 = white, 1000 = black).
 * @context line_task (10 ms hot path)
 */
uint32_t line_track_normalise(uint32_t raw, uint32_t white,
                              uint32_t black, uint32_t recip_q16)
{
    uint32_t lo            = white;
    uint32_t hi            = black;
    bool     b_high_black  = LINE_BLACK_IS_HIGH;
    uint32_t clamped       = raw;
    uint32_t norm          = 0u;
    uint32_t result        = 0u;

    if (white > black)
    {
        lo           = black;
        hi           = white;
        b_high_black = false;
    }
    else
    {
        /* Endpoints are in standard ascending order [lo, hi]. */
    }

    if (clamped <= lo)
    {
        norm = 0u;
    }
    else if (clamped >= hi)
    {
        norm = LINE_NORM_MAX;
    }
    else
    {
        /* Product < 2^32 because span >= 400 so recip_q16 <= 163840 and
           (clamped - lo) < span, giving product <= 65536000. */
        norm = ((clamped - lo) * recip_q16) >> LINE_RECIP_SHIFT;
        if (norm > LINE_NORM_MAX)
        {
            norm = LINE_NORM_MAX;
        }
        else
        {
            /* Within 0..1000. */
        }
    }

    if (b_high_black)
    {
        result = norm;
    }
    else
    {
        result = LINE_NORM_MAX - norm;
    }

    return result;
}

/**
 * line_track_steering
 * @brief   Computes steering error in permille (-1000..+1000).
 *          +1000 means line is fully under the left sensor (left of centre).
 * @param   norm_l  Left normalised reading (0..1000).
 * @param   norm_r  Right normalised reading (0..1000).
 * @return  Signed steering error (norm_l - norm_r) in -1000..+1000.
 * @context line_task (10 ms hot path)
 */
int32_t line_track_steering(uint32_t norm_l, uint32_t norm_r)
{
    int32_t const left_s  = (int32_t) norm_l; /* cast: <= 1000 fits int32_t */
    int32_t const right_s = (int32_t) norm_r; /* cast: <= 1000 fits int32_t */

    return (left_s - right_s);
}

/**
 * line_track_jct_reset
 * @brief   Resets the junction classifier state machine to IDLE.
 * @param   p_jct  Junction state structure.
 * @return  none
 * @context line_task
 */
void line_track_jct_reset(line_track_jct_t * p_jct)
{
    if (NULL != p_jct)
    {
        p_jct->state       = LINE_JCT_STATE_IDLE;
        p_jct->mark_counts = 0u;
        p_jct->b_sat_l     = false;
        p_jct->b_sat_r     = false;
        p_jct->seq0        = 0u;
    }
    else
    {
        /* Null pointer ignored. */
    }
}

/**
 * line_track_jct_step
 * @brief   Advances the junction state machine by one 10 ms cycle (Table J1).
 * @param   p_jct  Junction state machine structure.
 * @param   p_in   Cycle inputs (normalised sensors, path, branch info).
 * @return  APP_JCT_NONE, or the confirmed junction class (APP_JCT_*).
 * @context line_task (10 ms periodic)
 */
uint32_t line_track_jct_step(line_track_jct_t * p_jct,
                             line_track_jct_in_t const * p_in)
{
    uint32_t jct_class = APP_JCT_NONE;

    if ((NULL != p_jct) && (NULL != p_in))
    {
        switch (p_jct->state)
        {
            case LINE_JCT_STATE_IDLE:
            {
                line_track_jct_idle(p_jct, p_in);
                break;
            }
            case LINE_JCT_STATE_CANDIDATE:
            {
                line_track_jct_candidate(p_jct, p_in);
                break;
            }
            case LINE_JCT_STATE_CONFIRM:
            {
                jct_class = line_track_jct_confirm(p_jct, p_in);
                break;
            }
            default:
            {
                line_track_jct_reset(p_jct);
                break;
            }
        }
    }
    else
    {
        /* Invalid arguments: return APP_JCT_NONE. */
    }

    return jct_class;
}

/* ---- private functions ------------------------------------------------ */

/**
 * line_track_jct_set_barcode_side
 * @brief   Sets the saturation flag corresponding to LINE_BARCODE_SIDE.
 * @param   p_jct  Junction state machine structure.
 * @return  none
 * @context line_task
 */
static void line_track_jct_set_barcode_side(line_track_jct_t * p_jct)
{
    if (APP_SIDE_LEFT == LINE_BARCODE_SIDE)
    {
        p_jct->b_sat_l = true;
    }
    else
    {
        p_jct->b_sat_r = true;
    }
}

/**
 * line_track_jct_idle
 * @brief   IDLE state handler: enters CANDIDATE when nL + nR >= JCT_SUM_MIN.
 * @param   p_jct  Junction state machine structure.
 * @param   p_in   Cycle inputs.
 * @return  none
 * @context line_task
 */
static void line_track_jct_idle(line_track_jct_t * p_jct,
                                line_track_jct_in_t const * p_in)
{
    uint32_t const sum = p_in->norm_l + p_in->norm_r;

    if (sum >= LINE_JCT_SUM_MIN)
    {
        p_jct->mark_counts = p_in->path_counts;
        p_jct->b_sat_l     = (p_in->norm_l >= LINE_SAT_THRESHOLD);
        p_jct->b_sat_r     = (p_in->norm_r >= LINE_SAT_THRESHOLD);
        p_jct->seq0        = p_in->branch_seq;

        if (p_in->b_branch_recent)
        {
            line_track_jct_set_barcode_side(p_jct);
        }
        else
        {
            /* No early branch seen before candidate start. */
        }

        p_jct->state = LINE_JCT_STATE_CANDIDATE;
    }
    else
    {
        /* Stay in IDLE. */
    }
}

/**
 * line_track_jct_candidate
 * @brief   CANDIDATE state handler: latches saturation and checks width.
 * @param   p_jct  Junction state machine structure.
 * @param   p_in   Cycle inputs.
 * @return  none
 * @context line_task
 */
static void line_track_jct_candidate(line_track_jct_t * p_jct,
                                     line_track_jct_in_t const * p_in)
{
    uint32_t const sum = p_in->norm_l + p_in->norm_r;

    p_jct->b_sat_l =
        (p_jct->b_sat_l || (p_in->norm_l >= LINE_SAT_THRESHOLD));
    p_jct->b_sat_r =
        (p_jct->b_sat_r || (p_in->norm_r >= LINE_SAT_THRESHOLD));

    if (sum < LINE_JCT_SUM_MIN)
    {
        if ((p_in->path_counts - p_jct->mark_counts) >= p_in->min_counts)
        {
            p_jct->mark_counts = p_in->path_counts;
            p_jct->state       = LINE_JCT_STATE_CONFIRM;
        }
        else
        {
            line_track_jct_reset(p_jct);
        }
    }
    else
    {
        /* Still crossing extra-black region. */
    }
}

/**
 * line_track_jct_confirm
 * @brief   CONFIRM state handler: waits confirm_counts and classifies.
 * @param   p_jct  Junction state machine structure.
 * @param   p_in   Cycle inputs.
 * @return  APP_JCT_NONE or confirmed class when confirm distance reached.
 * @context line_task
 */
static uint32_t line_track_jct_confirm(line_track_jct_t * p_jct,
                                       line_track_jct_in_t const * p_in)
{
    uint32_t jct_class = APP_JCT_NONE;

    if (p_in->branch_seq != p_jct->seq0)
    {
        line_track_jct_set_barcode_side(p_jct);
    }
    else
    {
        /* No new barcode branch sequence reported during confirm window. */
    }

    if ((p_in->path_counts - p_jct->mark_counts) >= p_in->confirm_counts)
    {
        bool const b_is_present =
            ((p_in->norm_l >= LINE_WHITE_THRESHOLD) ||
             (p_in->norm_r >= LINE_WHITE_THRESHOLD));

        if (p_jct->b_sat_l && p_jct->b_sat_r)
        {
            jct_class = b_is_present ? APP_JCT_CROSS : APP_JCT_T;
        }
        else if (p_jct->b_sat_l)
        {
            jct_class = APP_JCT_SIDE_L;
        }
        else if (p_jct->b_sat_r)
        {
            jct_class = APP_JCT_SIDE_R;
        }
        else
        {
            jct_class = APP_JCT_NONE;
        }

        line_track_jct_reset(p_jct);
    }
    else
    {
        /* Waiting for confirmation travel distance. */
    }

    return jct_class;
}

/*** end of file ***/
