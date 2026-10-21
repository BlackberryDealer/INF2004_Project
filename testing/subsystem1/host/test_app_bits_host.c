/**
 * @file    test_app_bits_host.c
 * @brief   Laptop tests of app_bits.c and the generated app_layout.h
 *          (S1-U20 .. S1-U23; S1-U24 is the shell test in the Makefile).
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    Minimum-bit requirement items 7-8
 */

#include "host_test.h"
#include "app_bits.h"
#include "app_layout.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- private constants and types ------------------------------------ */

/* Number of SB_/MSG_ fields in app_layout.h, counted by the Makefile so a
   new layout row without a table entry below fails S1-U20.             */
#ifndef LAYOUT_SB_MSG_FIELDS
#error "Build with the Makefile: it passes -DLAYOUT_SB_MSG_FIELDS"
#endif

typedef struct
{
    char const * p_name;
    uint32_t     pos;
    uint32_t     width;
    int32_t      min;
    int32_t      max;
    int32_t      offset;
} layout_field_t;

#define LAYOUT_FIELD(g, f)                                               \
    { #g "_" #f, g##_##f##_POS, g##_##f##_BITS, g##_##f##_MIN,           \
      g##_##f##_MAX, g##_##f##_OFFSET }

/* Every field of every SB_ (status board) and MSG_ (kernel message) group. */
static layout_field_t const g_fields[] =
{
    LAYOUT_FIELD(SB_LINE0, STEERING),
    LAYOUT_FIELD(SB_LINE0, STATUS),
    LAYOUT_FIELD(SB_LINE0, JUNCTION),
    LAYOUT_FIELD(SB_LINE0, CALIBRATED),
    LAYOUT_FIELD(SB_MOT0, SPEED_L),
    LAYOUT_FIELD(SB_MOT0, SPEED_R),
    LAYOUT_FIELD(SB_MOT0, STATE),
    LAYOUT_FIELD(SB_MOT1, POS_L),
    LAYOUT_FIELD(SB_MOT2, POS_R),
    LAYOUT_FIELD(SB_MOT3, PATH),
    LAYOUT_FIELD(SB_IMU0, PITCH),
    LAYOUT_FIELD(SB_IMU0, HUMP_NOW),
    LAYOUT_FIELD(SB_IMU0, HUMP_PEAK),
    LAYOUT_FIELD(SB_IMU0, MCLASS),
    LAYOUT_FIELD(SB_IMU0, TURN_SRC),
    LAYOUT_FIELD(SB_IMU1, TURN_RATE),
    LAYOUT_FIELD(SB_OBS0, DIST),
    LAYOUT_FIELD(SB_OBS0, SCAN_STATE),
    LAYOUT_FIELD(SB_OBS0, ANGLE_STEP),
    LAYOUT_FIELD(SB_OBS0, PLAN_SIDE),
    LAYOUT_FIELD(SB_OBS0, PROFILE_VALID),
    LAYOUT_FIELD(SB_OBS1, WIDTH),
    LAYOUT_FIELD(SB_OBS1, NEAREST),
    LAYOUT_FIELD(SB_OBS1, CLEAR_L),
    LAYOUT_FIELD(SB_OBS1, CLEAR_R),
    LAYOUT_FIELD(SB_OBS2, LEG_LAT),
    LAYOUT_FIELD(SB_OBS2, LEG_FWD),
    LAYOUT_FIELD(SB_MIS0, SYS),
    LAYOUT_FIELD(SB_MIS0, RUN),
    LAYOUT_FIELD(SB_MIS0, AVOID),
    LAYOUT_FIELD(SB_MIS0, END_REASON),
    LAYOUT_FIELD(SB_MIS0, ARMED),
    LAYOUT_FIELD(SB_MIS0, ARMED_CMD),
    LAYOUT_FIELD(MSG_CMD, PARAM),
    LAYOUT_FIELD(MSG_CMD, TYPE),
    LAYOUT_FIELD(MSG_CMD, TAG),
    LAYOUT_FIELD(MSG_EVT, VALUE),
    LAYOUT_FIELD(MSG_EVT, TYPE),
    LAYOUT_FIELD(MSG_ECHO, WIDTH),
    LAYOUT_FIELD(MSG_ECHO, SEQ),
    LAYOUT_FIELD(MSG_ECHO, VALID),
    LAYOUT_FIELD(MSG_ECHO, PAD),
    LAYOUT_FIELD(MSG_EDGE, GAP),
    LAYOUT_FIELD(MSG_EDGE, LEVEL),
};

#define FIELD_COUNT  (sizeof(g_fields) / sizeof(g_fields[0]))

/* Backgrounds the neighbouring fields hold while one field is written. */
static uint32_t const g_backgrounds[] = { 0x00000000u, 0xFFFFFFFFu,
                                          0xA5A5A5A5u, 0x5A5A5A5Au };

#define BACKGROUND_COUNT  (sizeof(g_backgrounds) / sizeof(g_backgrounds[0]))

/* ---- private functions ------------------------------------------------ */

/* Puts value into every background word; true if it reads back, decodes to
   value, and no other bit of the word changed.                          */
static bool field_round_trips(layout_field_t const * p_field, int32_t value)
{
    bool     b_ok      = true;
    bool     b_clamped = true;
    uint32_t raw       = 0u;
    uint32_t mask      = ((1u << p_field->width) - 1u) << p_field->pos;
    uint32_t word      = 0u;
    uint32_t idx       = 0u;

    raw = app_bits_encode(value, p_field->min, p_field->max,
                          p_field->offset, &b_clamped);
    b_ok = b_ok && (!b_clamped);

    for (idx = 0u; idx < BACKGROUND_COUNT; idx++)
    {
        word = app_bits_put(g_backgrounds[idx], p_field->pos, p_field->width,
                            raw);
        b_ok = b_ok &&
               (raw == app_bits_get(word, p_field->pos, p_field->width));
        b_ok = b_ok && (value == app_bits_decode(app_bits_get(word,
                                                 p_field->pos,
                                                 p_field->width),
                                                 p_field->offset));
        b_ok = b_ok && ((word & ~mask) == (g_backgrounds[idx] & ~mask));
    }

    return b_ok;
}

static void test_u20_layout_round_trip(void);
static void test_u21_encode_clamp(void);
static void test_u22_stream_round_trip(void);
static void test_u23_stream_overflow(void);

/* One pass/fail count for the whole program (HOST_INIT at file scope). */
HOST_INIT();

/* ---- tests ------------------------------------------------------------ */

static void test_u20_layout_round_trip(void)
{
    bool     b_all = true;
    uint32_t idx   = 0u;

    for (idx = 0u; idx < FIELD_COUNT; idx++)
    {
        layout_field_t const * p_field = &g_fields[idx];
        bool b_ok = field_round_trips(p_field, p_field->min) &&
                    field_round_trips(p_field, p_field->max);

        if ((p_field->min <= 0) && (0 <= p_field->max))
        {
            b_ok = b_ok && field_round_trips(p_field, 0);
        }
        if (!b_ok)
        {
            (void) printf("       field %s failed\n", p_field->p_name);
        }
        b_all = b_all && b_ok;
    }

    HOST_CHECK("S1-U20", FIELD_COUNT == (uint32_t) LAYOUT_SB_MSG_FIELDS,
               "table lists every SB_/MSG_ field of app_layout.h");
    HOST_CHECK("S1-U20", b_all,
               "put/get round-trips min, max and 0 of every SB_/MSG_ "
               "field; neighbours unchanged");
}

static void test_u21_encode_clamp(void)
{
    bool     b_clamped = false;
    uint32_t raw       = 0u;

    raw = app_bits_encode(-1001, SB_LINE0_STEERING_MIN, SB_LINE0_STEERING_MAX,
                          SB_LINE0_STEERING_OFFSET, &b_clamped);
    HOST_CHECK("S1-U21", (24u == raw) && b_clamped,
               "steering -1001 clamps to code 24 and sets the flag");

    b_clamped = true;
    raw = app_bits_encode(1000, SB_LINE0_STEERING_MIN, SB_LINE0_STEERING_MAX,
                          SB_LINE0_STEERING_OFFSET, &b_clamped);
    HOST_CHECK("S1-U21", (2024u == raw) && (!b_clamped),
               "steering 1000 encodes to 2024 with no clamp flag");

    raw = app_bits_encode(5000, SB_LINE0_STEERING_MIN, SB_LINE0_STEERING_MAX,
                          SB_LINE0_STEERING_OFFSET, NULL);
    HOST_CHECK("S1-U21", 2024u == raw,
               "NULL clamp-flag pointer accepted (5000 clamps to 2024)");

    b_clamped = false;
    HOST_CHECK("S1-U21", (100u == app_bits_clamp_u(300u, 100u, &b_clamped))
               && b_clamped, "unsigned clamp 300 -> 100 sets the flag");
    HOST_CHECK("S1-U21", 7u == app_bits_clamp_u(7u, 100u, NULL),
               "unsigned clamp in range is unchanged, NULL flag accepted");
}

static void test_u22_stream_round_trip(void)
{
    uint8_t           buf[8];
    app_bits_writer_t writer;
    app_bits_reader_t reader;
    uint32_t          v3  = 0u;
    uint32_t          v5  = 0u;
    uint32_t          v18 = 0u;
    uint32_t          v11 = 0u;

    app_bits_writer_init(&writer, buf, (uint32_t) sizeof(buf));
    app_bits_write(&writer, 0x7u, 3u);
    app_bits_write(&writer, 0x1Fu, 5u);
    app_bits_write(&writer, 0x3FFFFu, 18u);
    app_bits_write(&writer, 0x7FFu, 11u);
    HOST_CHECK("S1-U22", (!writer.b_is_overflow) &&
               (5u == app_bits_writer_bytes(&writer)),
               "3+5+18+11 bits at maximum fill 5 bytes, no overflow");

    app_bits_reader_init(&reader, buf, app_bits_writer_bytes(&writer));
    v3  = app_bits_read(&reader, 3u);
    v5  = app_bits_read(&reader, 5u);
    v18 = app_bits_read(&reader, 18u);
    v11 = app_bits_read(&reader, 11u);
    HOST_CHECK("S1-U22", (0x7u == v3) && (0x1Fu == v5) &&
               (0x3FFFFu == v18) && (0x7FFu == v11) &&
               (!reader.b_is_overflow),
               "reader returns the same four values");
    HOST_CHECK("S1-U22", (0xFFu == buf[0]) && (0xFFu == buf[3]) &&
               (0x1Fu == buf[4]) && (0u == buf[5]),
               "stream bit n is bit n%8 of byte n/8; unused bits zero");

    app_bits_writer_init(&writer, buf, (uint32_t) sizeof(buf));
    app_bits_write(&writer, APP_MQ_VERSION, MQ_HDR_VER_BITS);
    app_bits_write(&writer, APP_MQ_TYPE_STATE, MQ_HDR_TYPE_BITS);
    HOST_CHECK("S1-U22", (1u | (0u << 3)) == buf[0],
               "state header byte 0 equals 1 | (0 << 3)");
}

static void test_u23_stream_overflow(void)
{
    uint8_t           buf[3] = { 0u, 0u, 0xEEu };
    app_bits_writer_t writer;
    app_bits_reader_t reader;
    uint32_t          value = 0u;

    /* Writer sees only the first 2 bytes; byte 2 is a guard. */
    app_bits_writer_init(&writer, buf, 2u);
    app_bits_write(&writer, 0x3FFu, 10u);
    app_bits_write(&writer, 0x7Fu, 7u);           /* 17 bits > 16 */
    HOST_CHECK("S1-U23", writer.b_is_overflow && (10u == writer.bit_pos) &&
               (0xFFu == buf[0]) && (0x03u == buf[1]) && (0xEEu == buf[2]),
               "write past the end sets overflow and writes nothing");

    app_bits_write(&writer, 0x1u, 1u);            /* would fit, but sticky */
    HOST_CHECK("S1-U23", (10u == writer.bit_pos) && (0x03u == buf[1]),
               "after an overflow later writes are ignored");

    app_bits_writer_init(&writer, buf, 2u);
    app_bits_write(&writer, 0x1u, 0u);
    app_bits_write(&writer, 0x1u, 32u);
    HOST_CHECK("S1-U23", writer.b_is_overflow && (0u == writer.bit_pos),
               "width 0 or 32 is rejected as overflow");

    app_bits_reader_init(&reader, buf, 2u);
    value = app_bits_read(&reader, 10u);
    value = app_bits_read(&reader, 7u);
    HOST_CHECK("S1-U23", reader.b_is_overflow && (0u == value),
               "reading past the end sets overflow and returns 0");
}

/* ---- main ------------------------------------------------------------- */

int main(void)
{
    test_u20_layout_round_trip();
    test_u21_encode_clamp();
    test_u22_stream_round_trip();
    test_u23_stream_overflow();

    return (0 == HOST_SUMMARY("app_bits")) ? 0 : 1;
}
