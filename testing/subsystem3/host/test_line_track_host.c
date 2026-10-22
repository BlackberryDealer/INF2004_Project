/*
 *  test_line_track_host.c
 *  Host unit tests for subsystem 3's pure line_track module:
 *    S3-U01  Normalise & steering on fixed values (both polarities)
 *    S3-U02  line_track_jct_step replaying junction traces (T, cross,
 *            side L, side R, curve, noise)
 *    S3-U06  Exact endpoints 0 and 1000 (incl. span 3000 V13-09) and
 *            calibration with reversed polarity (R13)
 *    S3-U14  Barcode-side branch classified in both early (b_branch_recent)
 *            and late (branch_seq != seq0 in CONFIRM) orders (F-03)
 */

#include "host_test.h"
#include "cfg_line.h"
#include "line_track.h"

#define JCT_MIN_C     (29u)  /* ~10 mm at 349 um/count */
#define JCT_CONFIRM_C (86u)  /* ~30 mm at 349 um/count */

typedef struct
{
    uint32_t norm_l;
    uint32_t norm_r;
    uint32_t path_counts;
    uint32_t branch_seq;
    bool     b_branch_recent;
} jct_sample_t;

static uint32_t replay_trace(jct_sample_t const * p_trace, uint32_t len,
                             uint32_t * p_fire_count)
{
    line_track_jct_t jct       = {0u, 0u, false, false, 0u};
    uint32_t         last_cls  = APP_JCT_NONE;
    uint32_t         fires     = 0u;
    uint32_t         idx       = 0u;

    line_track_jct_reset(&jct);

    for (idx = 0u; idx < len; idx++)
    {
        line_track_jct_in_t in =
        {
            p_trace[idx].norm_l,
            p_trace[idx].norm_r,
            p_trace[idx].path_counts,
            p_trace[idx].branch_seq,
            JCT_MIN_C,
            JCT_CONFIRM_C,
            p_trace[idx].b_branch_recent
        };
        uint32_t const cls = line_track_jct_step(&jct, &in);

        if (APP_JCT_NONE != cls)
        {
            last_cls = cls;
            fires++;
        }
    }

    *p_fire_count = fires;
    return last_cls;
}

static void test_s3_u01_u06(void)
{
    line_track_cal_t cal  = {0u, 0u, 0u, 0u, 0u, 0u};
    line_track_cal_t cal2 = {0u, 0u, 0u, 0u, 0u, 0u};
    bool             b_ok = false;

    /* Span 3000 (500..3500): truncated reciprocal would give 999 at hi;
       rounded-up reciprocal (V13-09) must give exact 0 and 1000. */
    b_ok = line_track_cal_finish(&cal, 500u, 3500u, 400u, 2400u);
    HOST_CHECK("S3-U06", b_ok, "cal_finish succeeds with valid spans");
    HOST_CHECK("S3-U06",
               0u == line_track_normalise(500u, cal.white_l, cal.black_l,
                                          cal.recip_l_q16),
               "high-is-black: raw == lo gives exact 0");
    HOST_CHECK("S3-U06",
               1000u == line_track_normalise(3500u, cal.white_l, cal.black_l,
                                             cal.recip_l_q16),
               "high-is-black: raw == hi gives exact 1000 (span 3000)");
    HOST_CHECK("S3-U06",
               1000u == line_track_normalise(3499u, cal.white_l, cal.black_l,
                                             cal.recip_l_q16) ||
               999u == line_track_normalise(3499u, cal.white_l, cal.black_l,
                                            cal.recip_l_q16),
               "high-is-black: raw == hi-1 does not overshoot 1000");

    /* Reversed polarity passed to normalise (white=3500, black=500) */
    HOST_CHECK("S3-U06",
               0u == line_track_normalise(3500u, 3500u, 500u, cal.recip_l_q16),
               "low-is-black: raw == hi (white) gives exact 0");
    HOST_CHECK("S3-U06",
               1000u == line_track_normalise(500u, 3500u, 500u,
                                             cal.recip_l_q16),
               "low-is-black: raw == lo (black) gives exact 1000");
    HOST_CHECK("S3-U01",
               500u == line_track_normalise(2000u, 3500u, 500u,
                                            cal.recip_l_q16),
               "low-is-black: midpoint gives 500");

    /* Reversed min/max order passed to cal_finish (R13) */
    b_ok = line_track_cal_finish(&cal2, 3500u, 500u, 2400u, 400u);
    HOST_CHECK("S3-U06", b_ok && (500u == cal2.white_l) &&
                         (3500u == cal2.black_l),
               "cal_finish orders reversed endpoints without underflow");
    HOST_CHECK("S3-U06",
               !line_track_cal_finish(&cal2, 1000u, 1399u, 400u, 2400u),
               "cal_finish rejects span < LINE_CAL_MIN_SPAN (399 < 400)");

    /* Out-of-range clamping and midpoint (S3-U01) */
    HOST_CHECK("S3-U01",
               0u == line_track_normalise(100u, cal.white_l, cal.black_l,
                                          cal.recip_l_q16),
               "raw below white clamps to 0");
    HOST_CHECK("S3-U01",
               1000u == line_track_normalise(4095u, cal.white_l, cal.black_l,
                                             cal.recip_l_q16),
               "raw above black clamps to 1000");
    HOST_CHECK("S3-U01",
               500u == line_track_normalise(2000u, cal.white_l, cal.black_l,
                                            cal.recip_l_q16),
               "midpoint normalises to 500");

    /* Steering conventions: +1000 = line under left sensor */
    HOST_CHECK("S3-U01", 0 == line_track_steering(500u, 500u),
               "equal sensors give 0 steering");
    HOST_CHECK("S3-U01", 1000 == line_track_steering(1000u, 0u),
               "left full black, right white gives +1000");
    HOST_CHECK("S3-U01", -1000 == line_track_steering(0u, 1000u),
               "right full black, left white gives -1000");
}

static void test_s3_u02_u14(void)
{
    uint32_t fires = 0u;
    uint32_t cls   = APP_JCT_NONE;

    /* 1. T-junction: both sensors saturate over bar, white after confirm */
    static jct_sample_t const trace_t[] =
    {
        { 500u, 500u,  10u, 0u, false },
        { 950u, 950u,  20u, 0u, false }, /* IDLE -> CANDIDATE (mark=20) */
        { 960u, 940u,  40u, 0u, false },
        { 100u, 100u,  55u, 0u, false }, /* sum < 1400, 55-20 >= 29 -> CONFIRM */
        {  80u,  90u, 100u, 0u, false },
        {  50u,  60u, 145u, 0u, false }  /* 145-55 >= 86, both < 300 -> T */
    };

    /* 2. Cross-junction: both saturate, line still present after confirm */
    static jct_sample_t const trace_cross[] =
    {
        { 500u, 500u,  10u, 0u, false },
        { 920u, 930u,  20u, 0u, false },
        { 950u, 950u,  45u, 0u, false },
        { 500u, 500u,  55u, 0u, false }, /* -> CONFIRM (mark=55) */
        { 510u, 490u, 100u, 0u, false },
        { 500u, 500u, 145u, 0u, false }  /* present -> CROSS */
    };

    /* 3. Side-left junction: left saturates (950), right stays at 500 */
    static jct_sample_t const trace_side_l[] =
    {
        { 500u, 500u,  10u, 0u, false },
        { 950u, 500u,  20u, 0u, false }, /* sum = 1450 >= 1400, sat_l = 1 */
        { 940u, 510u,  45u, 0u, false },
        { 500u, 500u,  55u, 0u, false }, /* -> CONFIRM */
        { 500u, 500u, 145u, 0u, false }  /* only sat_l -> SIDE_L */
    };

    /* 4. Side-right junction: right saturates (950), left stays at 500 */
    static jct_sample_t const trace_side_r[] =
    {
        { 500u, 500u,  10u, 0u, false },
        { 500u, 950u,  20u, 0u, false }, /* sum = 1450 >= 1400, sat_r = 1 */
        { 510u, 940u,  45u, 0u, false },
        { 500u, 500u,  55u, 0u, false }, /* -> CONFIRM */
        { 500u, 500u, 145u, 0u, false }  /* only sat_r -> SIDE_R */
    };

    /* 5. Smooth curve: difference shifts (850 vs 150) but sum = 1000 < 1400 */
    static jct_sample_t const trace_curve[] =
    {
        { 500u, 500u,  10u, 0u, false },
        { 850u, 150u,  40u, 0u, false },
        { 880u, 120u,  80u, 0u, false },
        { 200u, 800u, 120u, 0u, false },
        { 500u, 500u, 160u, 0u, false }
    };

    /* 6. Short noise spike (< min_counts) resets to IDLE without firing */
    static jct_sample_t const trace_noise[] =
    {
        { 950u, 950u,  20u, 0u, false },
        { 500u, 500u,  35u, 0u, false }, /* 35 - 20 = 15 < 29 -> IDLE */
        { 500u, 500u, 150u, 0u, false }
    };

    /* 7. S3-U14 early order: barcode sensor ahead of line sensors sets
       b_branch_recent = true before CANDIDATE starts; left sensor saturates */
    static jct_sample_t const trace_branch_early[] =
    {
        { 950u, 500u,  20u, 1u, true  }, /* sat_l + b_branch_recent (sat_r) */
        { 950u, 500u,  45u, 1u, false },
        { 500u, 500u,  55u, 1u, false }, /* -> CONFIRM */
        { 500u, 500u, 145u, 1u, false }  /* both sat + present -> CROSS */
    };

    /* 8. S3-U14 late order: barcode sensor reports branch_seq increment
       during CONFIRM window (after seq0 was captured) */
    static jct_sample_t const trace_branch_late[] =
    {
        { 950u, 500u,  20u, 3u, false }, /* seq0 = 3, only sat_l */
        { 950u, 500u,  45u, 3u, false },
        { 100u, 100u,  55u, 3u, false }, /* -> CONFIRM */
        { 100u, 100u, 100u, 4u, false }, /* branch_seq (4) != seq0 (3) -> sat_r */
        { 100u, 100u, 145u, 4u, false }  /* both sat + white -> T */
    };

    cls = replay_trace(trace_t, 6u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_T == cls) && (1u == fires),
               "T-junction trace classified as APP_JCT_T once");

    cls = replay_trace(trace_cross, 6u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_CROSS == cls) && (1u == fires),
               "Cross-junction trace classified as APP_JCT_CROSS once");

    cls = replay_trace(trace_side_l, 5u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_SIDE_L == cls) && (1u == fires),
               "Left branch trace classified as APP_JCT_SIDE_L once");

    cls = replay_trace(trace_side_r, 5u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_SIDE_R == cls) && (1u == fires),
               "Right branch trace classified as APP_JCT_SIDE_R once");

    cls = replay_trace(trace_curve, 5u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_NONE == cls) && (0u == fires),
               "Curve trace produces no junction event");

    cls = replay_trace(trace_noise, 3u, &fires);
    HOST_CHECK("S3-U02", (APP_JCT_NONE == cls) && (0u == fires),
               "Short extra-black noise (< min_counts) ignored");

    cls = replay_trace(trace_branch_early, 4u, &fires);
    HOST_CHECK("S3-U14", (APP_JCT_CROSS == cls) && (1u == fires),
               "Early barcode branch (b_branch_recent) sets barcode-side flag");

    cls = replay_trace(trace_branch_late, 5u, &fires);
    HOST_CHECK("S3-U14", (APP_JCT_T == cls) && (1u == fires),
               "Late barcode branch (in CONFIRM) sets barcode-side flag");
}

int main(void)
{
    HOST_INIT();
    test_s3_u01_u06();
    test_s3_u02_u14();
    return (0 == HOST_SUMMARY("line_track")) ? 0 : 1;
}
