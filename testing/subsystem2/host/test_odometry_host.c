/**
 * @file    test_odometry_host.c
 * @brief   Laptop tests of odometry.c: S2-U01 (conversions against hand
 *          calculations) and S2-U11 (speed window).
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    NFR2, NFR3
 *
 * Hand calculations use the constants in app_types.h (A4/wheel measured
 * on the car 2026-10-29; track still a placeholder):
 *   um per count  = (207345 + 320) / 640            = 324
 *   turn circ.    = 125000 x 355 / 113              = 392 699 um
 *   counts/deg    = 392 699 x 1000 / 360 / 324      = 3366 (x1000)
 *   cdeg per diff = 324 x 1 800 000 / 392 699       = 1485 (x100)
 * If a MEASURE value in app_types.h changes, recompute these numbers.
 */

#include "host_test.h"
#include "odometry.h"
#include "app_types.h"

#include <stdint.h>
#include <stdbool.h>

/* ---- private function prototypes ------------------------------------- */

static bool test_within_pct(int32_t value, int32_t expect, int32_t pct);

/* ---- public functions ------------------------------------------------- */

/**
 * main
 * @brief   Runs S2-U01 and S2-U11.
 * @return  0 if every check passed, 1 otherwise
 * @context host
 */
int main(void)
{
    odometry_window_t win      = {{0}, {0}, {0}, 0u};
    int32_t           speed_l  = 0;
    int32_t           speed_r  = 0;
    uint32_t          idx      = 0u;

    HOST_INIT();

    /* ---- S2-U01: conversions ---------------------------------------- */
    HOST_CHECK("S2-U01", 324u == APP_UM_PER_COUNT,
               "um per count is 324 (hand calc basis)");
    HOST_CHECK("S2-U01", 0u == odometry_counts_to_mm(0u),
               "0 counts = 0 mm");
    HOST_CHECK("S2-U01", 207u == odometry_counts_to_mm(640u),
               "640 counts (one rev) = (640x324+500)/1000 = 207 mm");
    HOST_CHECK("S2-U01", 42467u == odometry_counts_to_mm(131071u),
               "131071 counts = 42467 mm");
    HOST_CHECK("S2-U01", 3086u == odometry_mm_to_counts(1000u),
               "1000 mm = (1000000+162)/324 = 3086 counts");
    HOST_CHECK("S2-U01", 6318u == odometry_mm_to_counts(2047u),
               "2047 mm (longest move) = 6318 counts");
    HOST_CHECK("S2-U01", 298 == odometry_cpw_to_mm_s(46),
               "46 cpw = 46x324/50 = 298 mm/s");
    HOST_CHECK("S2-U01", -298 == odometry_cpw_to_mm_s(-46),
               "-46 cpw = -298 mm/s (truncates toward zero)");
    HOST_CHECK("S2-U01", 39 == odometry_mm_s_to_cpw(250),
               "250 mm/s = (12500+162)/324 = 39 cpw");
    HOST_CHECK("S2-U01", -39 == odometry_mm_s_to_cpw(-250),
               "-250 mm/s = -39 cpw (rounded, sign kept)");
    HOST_CHECK("S2-U01", 23 == odometry_mm_s_to_cpw(150),
               "150 mm/s = 7500/324 = 23.1 -> 23 cpw");
    HOST_CHECK("S2-U01", 303u == odometry_turn_counts(90u),
               "90 deg = (90x3366+500)/1000 = 303 counts per wheel");
    HOST_CHECK("S2-U01", 1212u == odometry_turn_counts(360u),
               "360 deg = 1212 counts per wheel");
    HOST_CHECK("S2-U01",
               test_within_pct(odometry_heading_cdeg(303, -303), 8999, 1),
               "heading(+303, -303) ~= +8999 cdeg (+-1 %)");
    HOST_CHECK("S2-U01",
               test_within_pct(odometry_heading_cdeg(-303, 303), -8999, 1),
               "heading(-303, +303) ~= -8999 cdeg (+-1 %), sign kept");
    HOST_CHECK("S2-U01", 0 == odometry_heading_cdeg(0, 0),
               "heading(0, 0) = 0");
    HOST_CHECK("S2-U01", -14 == odometry_heading_cdeg(-1, 0),
               "heading(-1, 0) = -1485/100 = -14 cdeg: stays negative");

    /* ---- S2-U21: PIO edges -> counts (car 2026-10-29, A4) ------------- */
    {
        bool     b_mono  = true;
        bool     b_step  = true;
        uint32_t prev    = odometry_hw_to_counts(0xFFFFFFF0u); /* -16 */
        uint32_t steps   = 0u;
        int32_t  edge    = 0;

        for (edge = -15; edge <= 16; edge++)
        {
            /* cast: two's-complement image of a signed edge count */
            uint32_t const now = odometry_hw_to_counts((uint32_t) edge);

            if ((now != prev) && (now != (prev + 1u)))
            {
                b_mono = false;
            }
            steps += (now - prev);
            prev = now;
        }
        b_step = (8u == steps);   /* 32 edges -> 8 counts */
        HOST_CHECK("S2-U21", b_mono && b_step,
                   "-16..+16 edges: rises by exactly 1 every 4 edges, also through 0");
        HOST_CHECK("S2-U21",
                   (odometry_hw_to_counts(0u) - odometry_hw_to_counts(0xFFFFFFFFu)) == 1u,
                   "edge -1 -> 0 crosses a count boundary (floor, not truncation)");
        HOST_CHECK("S2-U21",
                   (odometry_hw_to_counts(25600u) - odometry_hw_to_counts(0u)) == 6400u,
                   "10 wheel turns x 2560 edges = 6400 counts (640 per turn)");
        HOST_CHECK("S2-U21",
                   (odometry_hw_to_counts(0u) - odometry_hw_to_counts((uint32_t) -25600)) == 6400u,
                   "same 10 turns backward: 6400 counts the other way");
    }

    /* ---- S2-U22: 500 mm/s physical ceiling vs the packed fields ------
       Hand values, 640 counts per 207.345 mm turn:
       one read spans (1 + 3 late) x 10 ms = 40 ms -> 20 mm -> 61.7 -> 62;
       one tick 10 ms -> 5 mm -> 15.4 -> 16 (rounding each tick first
       would give 4 x 16 = 64 and wrongly fail the 7-bit slot);
       50 ms window -> 25 mm -> 77.2 -> 78. */
    HOST_CHECK("S2-U22", 500u == APP_SPEED_PHYS_MAX_MM_S,
               "physical speed ceiling is 500 mm/s");
    HOST_CHECK("S2-U22", 62u == APP_SPAN_COUNTS_MAX,
               "40 ms read span at 500 mm/s = 62 counts (<= 63: 7-bit delta)");
    HOST_CHECK("S2-U22", 16u == APP_TICK_COUNTS_MAX,
               "one 10 ms tick at 500 mm/s = 16 counts");
    HOST_CHECK("S2-U22", 78u == APP_WINDOW_COUNTS_MAX,
               "50 ms window at 500 mm/s = 78 counts (<= 127: 8-bit speed)");

    /* ---- S2-U11: speed window ---------------------------------------- */
    odometry_window_reset(&win);
    odometry_window_push(&win, 63, -63, 10u);
    odometry_window_push(&win, 64, -64, 10u);
    HOST_CHECK("S2-U11", (63 == win.delta_l[0]) && (-63 == win.delta_r[0]),
               "deltas +-63 stored unchanged");
    HOST_CHECK("S2-U11", (63 == win.delta_l[1]) && (-63 == win.delta_r[1]),
               "deltas +-64 clamped to +-63");

    odometry_window_reset(&win);
    for (idx = 0u; idx < ODOMETRY_WINDOW_SLOTS; idx++)
    {
        odometry_window_push(&win, 10, -7, 10u);
    }
    odometry_window_speed(&win, &speed_l, &speed_r);
    HOST_CHECK("S2-U11", (50 == speed_l) && (-35 == speed_r),
               "normal 50 ms window: speed = sum (50, -35)");
    HOST_CHECK("S2-U11", 0u == win.idx, "index wraps 4 -> 0 without %");

    odometry_window_reset(&win);
    odometry_window_push(&win, 10, 10, 10u);
    odometry_window_push(&win, 10, 10, 10u);
    odometry_window_push(&win, 10, 10, 40u);
    odometry_window_push(&win, 10, 10, 40u);
    odometry_window_push(&win, 10, 10, 40u);
    odometry_window_speed(&win, &speed_l, &speed_r);
    HOST_CHECK("S2-U11", (17 == speed_l) && (17 == speed_r),
               "3 late ticks (span 140 ms): 50 x 50 / 140 = 17 cpw");

    odometry_window_reset(&win);
    odometry_window_speed(&win, &speed_l, &speed_r);
    HOST_CHECK("S2-U11", (0 == speed_l) && (0 == speed_r),
               "empty window (span 0) = speed 0");

    odometry_window_push(&win, 5, 5, 200u);
    HOST_CHECK("S2-U11", 63u == win.dt_ms[0], "dt 200 ms clamped to 63");

    return (0 == HOST_SUMMARY("odometry")) ? 0 : 1;
}

/* ---- private functions ------------------------------------------------ */

/**
 * test_within_pct
 * @brief   |value - expect| <= |expect| x pct / 100.
 * @param   value   measured value
 * @param   expect  hand-calculated value (non-zero)
 * @param   pct     tolerance in percent
 * @return  true if within tolerance
 * @context host
 */
static bool test_within_pct(int32_t value, int32_t expect, int32_t pct)
{
    int32_t const diff  = (value > expect) ? (value - expect)
                                           : (expect - value);
    int32_t const scale = (expect < 0) ? -expect : expect;

    return ((diff * 100) <= (scale * pct));
}

/*** end of file ***/
