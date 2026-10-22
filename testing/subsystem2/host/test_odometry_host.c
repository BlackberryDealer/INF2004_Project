/**
 * @file    test_odometry_host.c
 * @brief   Laptop tests of odometry.c: S2-U01 (conversions against hand
 *          calculations) and S2-U11 (speed window).
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    NFR2, NFR3
 *
 * Hand calculations use the placeholder constants in app_types.h:
 *   um per count  = (204000 + 292) / 585            = 349
 *   turn circ.    = 125000 x 355 / 113              = 392 699 um
 *   counts/deg    = 392 699 x 1000 / 360 / 349      = 3125 (x1000)
 *   cdeg per diff = 349 x 1 800 000 / 392 699       = 1599 (x100)
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
    HOST_CHECK("S2-U01", 349u == APP_UM_PER_COUNT,
               "placeholder um per count is 349 (hand calc basis)");
    HOST_CHECK("S2-U01", 0u == odometry_counts_to_mm(0u),
               "0 counts = 0 mm");
    HOST_CHECK("S2-U01", 204u == odometry_counts_to_mm(585u),
               "585 counts (one rev) = (585x349+500)/1000 = 204 mm");
    HOST_CHECK("S2-U01", 45744u == odometry_counts_to_mm(131071u),
               "131071 counts = 45744 mm");
    HOST_CHECK("S2-U01", 2865u == odometry_mm_to_counts(1000u),
               "1000 mm = (1000000+174)/349 = 2865 counts");
    HOST_CHECK("S2-U01", 5865u == odometry_mm_to_counts(2047u),
               "2047 mm (longest move) = 5865 counts");
    HOST_CHECK("S2-U01", 251 == odometry_cpw_to_mm_s(36),
               "36 cpw = 36x349/50 = 251 mm/s");
    HOST_CHECK("S2-U01", -251 == odometry_cpw_to_mm_s(-36),
               "-36 cpw = -251 mm/s (truncates toward zero)");
    HOST_CHECK("S2-U01", 36 == odometry_mm_s_to_cpw(250),
               "250 mm/s = (12500+174)/349 = 36 cpw");
    HOST_CHECK("S2-U01", -36 == odometry_mm_s_to_cpw(-250),
               "-250 mm/s = -36 cpw (rounded, sign kept)");
    HOST_CHECK("S2-U01", 21 == odometry_mm_s_to_cpw(150),
               "150 mm/s = 7500/349 = 21.49 -> 21 cpw");
    HOST_CHECK("S2-U01", 281u == odometry_turn_counts(90u),
               "90 deg = (90x3125+500)/1000 = 281 counts per wheel");
    HOST_CHECK("S2-U01", 1125u == odometry_turn_counts(360u),
               "360 deg = 1125 counts per wheel");
    HOST_CHECK("S2-U01",
               test_within_pct(odometry_heading_cdeg(313, -313), 10009, 1),
               "heading(+313, -313) ~= +10009 cdeg (+-1 %)");
    HOST_CHECK("S2-U01",
               test_within_pct(odometry_heading_cdeg(-313, 313), -10009, 1),
               "heading(-313, +313) ~= -10009 cdeg (+-1 %), sign kept");
    HOST_CHECK("S2-U01", 0 == odometry_heading_cdeg(0, 0),
               "heading(0, 0) = 0");
    HOST_CHECK("S2-U01", -15 == odometry_heading_cdeg(-1, 0),
               "heading(-1, 0) = -1599/100 = -15 cdeg: stays negative");

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
