/**
 * @file    test_telem_bin_host.c
 * @brief   Laptop tests of the binary payload builders: exact bytes
 *          (S1-U01, S1-U02, packed) and the golden vectors for the laptop
 *          decoder (S1-U10: writes golden/<name>.bin and golden/<name>.csv;
 *          the Makefile then runs tools/telem_decode.py --golden golden).
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    FR10, FR12, NFR8, minimum-bit items 7-8
 */

#include "host_test.h"
#include "telem_bin.h"
#include "app_types.h"
#include "app_layout.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ---- private constants and types ------------------------------------ */

#define OUT_BYTES     (32u)
#define GOLDEN_DIR    "golden"
#define FIELD_MAX     (16u)

/* One golden vector: payload bytes plus the stored (offset-removed)
   value of every non-PAD field, in CSV order.                       */
typedef struct
{
    char const * p_name[FIELD_MAX];
    int32_t      value[FIELD_MAX];
    uint32_t     count;
} golden_t;

/* ---- private function prototypes ------------------------------------- */

static bool bytes_are(uint8_t const * p_out, uint32_t len,
                      uint8_t const * p_expect, uint32_t expect_len);
static bool golden_write(char const * p_name, uint8_t const * p_buf,
                         uint32_t len, golden_t const * p_gold);
static void golden_add(golden_t * p_gold, char const * p_name,
                       int32_t value);
static bool golden_state(char const * p_name, telem_bin_state_t const * p_st,
                         int32_t odo_mm);

/* ---- public functions ------------------------------------------------- */

int main(void)
{
    /* Expected bytes: fields LSB-first after the MQ_HDR byte (D.4). The
       state below is the same sample as the baseline JSON test.        */
    static uint8_t const exp_state[18] =
    {
        0x01u, 0x44u, 0x28u, 0xD9u, 0xD0u, 0x07u, 0xC4u, 0xE0u, 0x0Fu,
        0x83u, 0xA1u, 0x0Fu, 0x5Au, 0x64u, 0x98u, 0x66u, 0x08u, 0x00u
    };
    static uint8_t const exp_event[3]   = {0x09u, 0x02u, 0x18u};
    static uint8_t const exp_prof[6]    =
    {
        0x11u, 0x46u, 0x86u, 0xECu, 0x1Fu, 0x20u
    };
    static uint8_t const exp_summ[7]    =
    {
        0x19u, 0x65u, 0x10u, 0x27u, 0x00u, 0xECu, 0x01u
    };
    static uint8_t const exp_hb[6]      =
    {
        0x21u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x0Fu
    };
    uint8_t             out[OUT_BYTES];
    uint32_t            len   = 0u;
    bool                b_clp = true;
    bool                b_ok  = true;
    telem_bin_state_t   st    = {0};
    telem_bin_profile_t prof  = {0};
    telem_bin_summary_t summ  = {0};
    golden_t            gold  = {{NULL}, {0}, 0u};

    HOST_INIT();

    /* S1-U01 (packed): 18-byte state from a fixed struct */
    st.sys_state     = 4u;
    st.run_state     = 0u;
    st.motion_state  = 1u;
    st.speed_l_cpw   = 20;
    st.speed_r_cpw   = -20;
    st.pos_l_counts  = 1000;
    st.pos_r_counts  = -1000;
    st.line_status   = 0u;
    st.steering      = -250;
    st.path_counts   = 3086u;   /* 1000 mm (324 um per count) */
    st.pitch_ddeg    = 45;
    st.hump_peak_mm  = 12u;
    st.turn_rate_dps = -90;
    st.motion_class  = 6u;
    st.faults        = 0x10u;
    len = telem_bin_state(&st, out, OUT_BYTES, &b_clp);
    HOST_CHECK("S1-U01", bytes_are(out, len, exp_state, 18u),
               "state payload equals the expected 18 bytes");
    HOST_CHECK("S1-U01", (!b_clp) && (18u == (MQ_STATE_BYTES + 1u)),
               "no clamp reported; 1 + MQ_STATE_BYTES = 18");
    HOST_CHECK("S1-U01", 0u == telem_bin_state(&st, out, 17u, NULL),
               "17-byte buffer returns 0");

    st.speed_l_cpw = 500;
    (void) telem_bin_state(&st, out, OUT_BYTES, &b_clp);
    HOST_CHECK("S1-U01", b_clp, "RNG field out of range reports a clamp");
    st.speed_l_cpw = 20;
    st.motion_class = 9u;     /* ENUM out of range: clamped silently */
    (void) telem_bin_state(&st, out, OUT_BYTES, &b_clp);
    HOST_CHECK("S1-U01", !b_clp, "ENUM clamp is silent");
    st.motion_class = 6u;

    /* S1-U02 (packed): exact bytes of the other payloads */
    len = telem_bin_event(2u | (3u << 11u), out, OUT_BYTES);
    HOST_CHECK("S1-U02", bytes_are(out, len, exp_event, 3u),
               "event payload is 3 exact bytes");

    prof.angle_step     = 6u;
    prof.width_mm       = 200u;
    prof.nearest_mm     = 400u;
    prof.clear_left_mm  = 1020u;
    prof.clear_right_mm = 0u;
    prof.side           = 1u;
    len = telem_bin_profile(&prof, out, OUT_BYTES);
    HOST_CHECK("S1-U02", bytes_are(out, len, exp_prof, 6u),
               "profile payload is 6 exact bytes (4 mm steps)");

    summ.end_reason   = 1u;
    summ.hump_peak_mm = 25u;
    summ.odometry_mm  = 5000u;
    summ.faults       = 0u;
    summ.run_time_s   = 123u;
    len = telem_bin_summary(&summ, out, OUT_BYTES);
    HOST_CHECK("S1-U02", bytes_are(out, len, exp_summ, 7u),
               "summary payload is 7 exact bytes");

    len = telem_bin_heartbeat(70000u, 300u, 5000u, out, OUT_BYTES);
    HOST_CHECK("S1-U02", bytes_are(out, len, exp_hb, 6u),
               "heartbeat payload is 6 exact bytes (saturated)");

    HOST_CHECK("S1-U02", (0x29u == telem_bin_header(APP_MQ_TYPE_ONLINE))
                         && (0x31u == telem_bin_header(APP_MQ_TYPE_OFFLINE)),
               "availability bytes 0x29 online, 0x31 offline");
    HOST_CHECK("S1-U02", 0u == telem_bin_summary(&summ, out, 6u),
               "too-small buffer returns 0");

    /* S1-U10: golden vectors (every field at min and max; state at mid) */
    (void) memset(&st, 0, sizeof(st));
    st.speed_l_cpw   = -127;
    st.speed_r_cpw   = -127;
    st.pos_l_counts  = -131071;
    st.pos_r_counts  = -131071;
    st.steering      = -1000;
    st.pitch_ddeg    = -450;
    st.turn_rate_dps = -360;
    b_ok = golden_state("state_min", &st, 0);

    st.sys_state     = 7u;
    st.run_state     = 4u;
    st.motion_state  = 7u;
    st.speed_l_cpw   = 127;
    st.speed_r_cpw   = 127;
    st.pos_l_counts  = 131071;
    st.pos_r_counts  = 131071;
    st.line_status   = 3u;
    st.steering      = 1000;
    st.path_counts   = 101132u;  /* 32767 mm (324 um per count) */
    st.pitch_ddeg    = 450;
    st.hump_peak_mm  = 127u;
    st.turn_rate_dps = 360;
    st.motion_class  = 6u;
    st.faults        = 1023u;
    b_ok = golden_state("state_max", &st, 32767) && b_ok;

    st.sys_state     = 4u;
    st.run_state     = 2u;
    st.motion_state  = 2u;
    st.speed_l_cpw   = 36;
    st.speed_r_cpw   = -5;
    st.pos_l_counts  = 12345;
    st.pos_r_counts  = -6789;
    st.line_status   = 1u;
    st.steering      = -321;
    st.path_counts   = 3086u;    /* 1000 mm (324 um per count) */
    st.pitch_ddeg    = 87;
    st.hump_peak_mm  = 33u;
    st.turn_rate_dps = -45;
    st.motion_class  = 3u;
    st.faults        = 0x050u;
    b_ok = golden_state("state_mid", &st, 1000) && b_ok;

    len  = telem_bin_event(0u, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "WORD", 0);
    b_ok = golden_write("event_min", out, len, &gold) && b_ok;
    len  = telem_bin_event(0xFFFFu, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "WORD", 65535);
    b_ok = golden_write("event_max", out, len, &gold) && b_ok;

    (void) memset(&prof, 0, sizeof(prof));
    len  = telem_bin_profile(&prof, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "ANGLE_STEP", 0);
    golden_add(&gold, "WIDTH", 0);
    golden_add(&gold, "NEAREST", 0);
    golden_add(&gold, "CLEAR_L", 0);
    golden_add(&gold, "CLEAR_R", 0);
    golden_add(&gold, "SIDE", 0);
    b_ok = golden_write("profile_min", out, len, &gold) && b_ok;
    prof.angle_step     = 24u;
    prof.width_mm       = 1020u;
    prof.nearest_mm     = 1020u;
    prof.clear_left_mm  = 1020u;
    prof.clear_right_mm = 1020u;
    prof.side           = 1u;
    len  = telem_bin_profile(&prof, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "ANGLE_STEP", 24);
    golden_add(&gold, "WIDTH", 255);
    golden_add(&gold, "NEAREST", 255);
    golden_add(&gold, "CLEAR_L", 255);
    golden_add(&gold, "CLEAR_R", 255);
    golden_add(&gold, "SIDE", 1);
    b_ok = golden_write("profile_max", out, len, &gold) && b_ok;

    (void) memset(&summ, 0, sizeof(summ));
    len  = telem_bin_summary(&summ, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "END_REASON", 0);
    golden_add(&gold, "HUMP_PEAK", 0);
    golden_add(&gold, "ODO_MM", 0);
    golden_add(&gold, "FAULTS", 0);
    golden_add(&gold, "RUN_S", 0);
    b_ok = golden_write("summary_min", out, len, &gold) && b_ok;
    summ.end_reason   = 3u;
    summ.hump_peak_mm = 127u;
    summ.odometry_mm  = 32767u;
    summ.faults       = 1023u;
    summ.run_time_s   = 1023u;
    len  = telem_bin_summary(&summ, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "END_REASON", 3);
    golden_add(&gold, "HUMP_PEAK", 127);
    golden_add(&gold, "ODO_MM", 32767);
    golden_add(&gold, "FAULTS", 1023);
    golden_add(&gold, "RUN_S", 1023);
    b_ok = golden_write("summary_max", out, len, &gold) && b_ok;

    len  = telem_bin_heartbeat(0u, 0u, 0u, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "UPTIME_S", 0);
    golden_add(&gold, "DROPS", 0);
    golden_add(&gold, "LAT_MAX_US", 0);
    b_ok = golden_write("heartbeat_min", out, len, &gold) && b_ok;
    len  = telem_bin_heartbeat(65535u, 255u, 4095u, out, OUT_BYTES);
    gold.count = 0u;
    golden_add(&gold, "UPTIME_S", 65535);
    golden_add(&gold, "DROPS", 255);
    golden_add(&gold, "LAT_MAX_US", 4095);
    b_ok = golden_write("heartbeat_max", out, len, &gold) && b_ok;

    HOST_CHECK("S1-U10", b_ok, "golden vectors written to " GOLDEN_DIR "/");

    return (0 == HOST_SUMMARY("telem_bin")) ? 0 : 1;
}

/* ---- private functions ------------------------------------------------ */

static bool bytes_are(uint8_t const * p_out, uint32_t len,
                      uint8_t const * p_expect, uint32_t expect_len)
{
    return (len == expect_len) && (0 == memcmp(p_out, p_expect, len));
}

static void golden_add(golden_t * p_gold, char const * p_name,
                       int32_t value)
{
    if (p_gold->count < FIELD_MAX)
    {
        p_gold->p_name[p_gold->count] = p_name;
        p_gold->value[p_gold->count]  = value;
        p_gold->count++;
    }
}

/* State vector: CSV values are the inputs as stored (ODO in mm). */
static bool golden_state(char const * p_name, telem_bin_state_t const * p_st,
                         int32_t odo_mm)
{
    uint8_t  out[OUT_BYTES];
    uint32_t len  = telem_bin_state(p_st, out, OUT_BYTES, NULL);
    golden_t gold = {{NULL}, {0}, 0u};

    golden_add(&gold, "SYS", (int32_t) p_st->sys_state);
    golden_add(&gold, "RUN", (int32_t) p_st->run_state);
    golden_add(&gold, "MOT", (int32_t) p_st->motion_state);
    golden_add(&gold, "SPEED_L", p_st->speed_l_cpw);
    golden_add(&gold, "SPEED_R", p_st->speed_r_cpw);
    golden_add(&gold, "POS_L", p_st->pos_l_counts);
    golden_add(&gold, "POS_R", p_st->pos_r_counts);
    golden_add(&gold, "LINE", (int32_t) p_st->line_status);
    golden_add(&gold, "STEER", p_st->steering);
    golden_add(&gold, "ODO_MM", odo_mm);
    golden_add(&gold, "PITCH", p_st->pitch_ddeg);
    golden_add(&gold, "HUMP_PEAK", (int32_t) p_st->hump_peak_mm);
    golden_add(&gold, "TURN", p_st->turn_rate_dps);
    golden_add(&gold, "MCLASS", (int32_t) p_st->motion_class);
    golden_add(&gold, "FAULTS", (int32_t) p_st->faults);

    return golden_write(p_name, out, len, &gold);
}

static bool golden_write(char const * p_name, uint8_t const * p_buf,
                         uint32_t len, golden_t const * p_gold)
{
    char     path[64];
    FILE *   p_file = NULL;
    uint32_t idx    = 0u;
    bool     b_ok   = (len > 0u);

    (void) snprintf(path, sizeof(path), GOLDEN_DIR "/%s.bin", p_name);
    p_file = fopen(path, "wb");
    if ((NULL == p_file) || (len != fwrite(p_buf, 1u, len, p_file)))
    {
        b_ok = false;
    }
    if (NULL != p_file)
    {
        (void) fclose(p_file);
    }

    (void) snprintf(path, sizeof(path), GOLDEN_DIR "/%s.csv", p_name);
    p_file = fopen(path, "w");
    if (NULL == p_file)
    {
        b_ok = false;
    }
    else
    {
        (void) fprintf(p_file, "field,value\n");
        for (idx = 0u; idx < p_gold->count; idx++)
        {
            (void) fprintf(p_file, "%s,%ld\n", p_gold->p_name[idx],
                           (long) p_gold->value[idx]);
        }
        (void) fclose(p_file);
    }

    return b_ok;
}

/*** end of file ***/
