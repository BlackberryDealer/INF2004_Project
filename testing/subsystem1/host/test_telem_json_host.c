/**
 * @file    test_telem_json_host.c
 * @brief   Laptop tests of the baseline JSON builders (S1-U01, S1-U02).
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    FR10, FR12, NFR8
 */

#include "host_test.h"
#include "telem_json.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---- private constants and types ------------------------------------ */

#define OUT_BYTES  (200u)

/* ---- private function prototypes ------------------------------------- */

static bool text_is(char const * p_out, uint32_t len, char const * p_expect);

/* ---- public functions ------------------------------------------------- */

int main(void)
{
    char              out[OUT_BYTES];
    uint32_t          len   = 0u;
    uint32_t          need  = 0u;
    telem_bin_state_t state = {0};
    telem_bin_profile_t prof = {0};
    telem_bin_summary_t summ = {0};
    char const * const p_state_txt =
        "{\"st\":4,\"run\":0,\"mot\":1,\"spd\":[139,-139],"
        "\"enc\":[1000,-1000],\"line\":0,\"steer\":-250,\"odo\":1000,"
        "\"pitch\":45,\"hump_pk\":12,\"turn\":-90,\"cls\":6,\"flt\":16}";

    HOST_INIT();

    /* S1-U01: state from a fixed struct (20 cpw = 139 mm/s truncated;
       2865 counts = 1000 mm rounded).                                 */
    state.sys_state     = 4u;
    state.run_state     = 0u;
    state.motion_state  = 1u;
    state.speed_l_cpw   = 20;
    state.speed_r_cpw   = -20;
    state.pos_l_counts  = 1000;
    state.pos_r_counts  = -1000;
    state.line_status   = 0u;
    state.steering      = -250;
    state.path_counts   = 2865u;
    state.pitch_ddeg    = 45;
    state.hump_peak_mm  = 12u;
    state.turn_rate_dps = -90;
    state.motion_class  = 6u;
    state.faults        = 0x10u;
    len = telem_json_state(&state, out, OUT_BYTES);
    HOST_CHECK("S1-U01", text_is(out, len, p_state_txt),
               "state JSON equals the expected string exactly");
    HOST_CHECK("S1-U01", len < 160u, "state JSON fits one K11 message");

    state.path_counts = 131071u;   /* 45743 mm: beyond A23 */
    (void) memset(out, 0, sizeof(out));
    len = telem_json_state(&state, out, OUT_BYTES);
    HOST_CHECK("S1-U01", (NULL != strstr(out, "\"odo\":32767,"))
                         && (len > 0u),
               "odometry text clamped to 32767 mm");

    state.pos_l_counts = INT32_MIN;
    state.pos_r_counts = INT32_MAX;
    (void) memset(out, 0, sizeof(out));
    len = telem_json_state(&state, out, OUT_BYTES);
    HOST_CHECK("S1-U01", (len > 0u) && (NULL != strstr(out,
               "\"enc\":[-2147483648,2147483647]")),
               "int32 extremes print correctly");

    /* S1-U02: event, profile, summary, heartbeat */
    len = telem_json_event(3u, 2u, out, OUT_BYTES);
    HOST_CHECK("S1-U02", text_is(out, len, "{\"ev\":3,\"v\":2}"),
               "event JSON");

    prof.angle_step     = 6u;
    prof.width_mm       = 200u;
    prof.nearest_mm     = 400u;
    prof.clear_left_mm  = 1020u;
    prof.clear_right_mm = 0u;
    prof.side           = 1u;
    len = telem_json_profile(&prof, out, OUT_BYTES);
    HOST_CHECK("S1-U02", text_is(out, len,
               "{\"ev\":10,\"ang\":60,\"w\":200,\"near\":400,"
               "\"cl_l\":1020,\"cl_r\":0,\"side\":1}"),
               "profile JSON (angle = 30 + 5 x step)");

    summ.end_reason   = 1u;
    summ.hump_peak_mm = 25u;
    summ.odometry_mm  = 5000u;
    summ.faults       = 0u;
    summ.run_time_s   = 123u;
    len = telem_json_summary(&summ, out, OUT_BYTES);
    HOST_CHECK("S1-U02", text_is(out, len,
               "{\"ev\":22,\"reason\":1,\"hump_pk\":25,\"odo\":5000,"
               "\"flt\":0,\"t\":123}"),
               "summary JSON");

    len = telem_json_heartbeat(65535u, 255u, 4095u, out, OUT_BYTES);
    HOST_CHECK("S1-U02", text_is(out, len,
               "{\"up\":65535,\"drops\":255,\"lat\":4095}"),
               "heartbeat JSON");

    len = telem_json_heartbeat(0u, 0u, 0u, out, OUT_BYTES);
    HOST_CHECK("S1-U02", text_is(out, len,
               "{\"up\":0,\"drops\":0,\"lat\":0}"),
               "heartbeat JSON with zeros");

    /* too-small buffers return 0; the exact size fits */
    need = telem_json_summary(&summ, out, OUT_BYTES);
    HOST_CHECK("S1-U02", 0u == telem_json_summary(&summ, out, need - 1u),
               "summary one byte short returns 0");
    HOST_CHECK("S1-U02", need == telem_json_summary(&summ, out, need),
               "summary exact size fits");
    need = telem_json_state(&state, out, OUT_BYTES);
    HOST_CHECK("S1-U02", 0u == telem_json_state(&state, out, need - 1u),
               "state one byte short returns 0");
    HOST_CHECK("S1-U02", 0u == telem_json_event(1u, 0u, out, 0u),
               "zero-size buffer returns 0");

    return (0 == HOST_SUMMARY("telem_json")) ? 0 : 1;
}

/* ---- private functions ------------------------------------------------ */

static bool text_is(char const * p_out, uint32_t len, char const * p_expect)
{
    return (len == (uint32_t) strlen(p_expect))
           && (0 == memcmp(p_out, p_expect, len));
}

/*** end of file ***/
