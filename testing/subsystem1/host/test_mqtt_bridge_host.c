/**
 * @file    test_mqtt_bridge_host.c
 * @brief   Laptop tests of mqtt_bridge.c against a mock kernel: binary
 *          command validation (S1-U11), rate limit (S1-U12), MAP
 *          (S1-U35), K11 take, routing (debug topics S1-U45), console
 *          line formatting (S1-U46),
 *          availability, link changes and drop counters. The real source
 *          is included so its private counters can be checked.
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    FR11, FR20, NFR9c
 */

#include "host_test.h"
#include "mqtt_bridge.c"      /* the unit under test, from build/ */

#include <string.h>

/* ---- mock kernel and neighbours -------------------------------------- */

#define MOCK_K11_ID     (7)
#define MOCK_Q_MAX      (8u)
#define MOCK_MSG_MAX    (APP_MQ_PAYLOAD_MAX)   /* = K11 maxmsz */

static uint8_t  g_q_msg[MOCK_Q_MAX][MOCK_MSG_MAX];
static uint32_t g_q_len[MOCK_Q_MAX];
static uint32_t g_q_count;
static uint32_t g_now_us;
static uint32_t g_faults;
static uint32_t g_posts;
static uint32_t g_post_type[16];
static uint32_t g_post_value[16];

#define MOCK_K12_ID     (9)
#define MOCK_LOG_MAX    (4u)
static telem_log_rec_t g_log_q[MOCK_LOG_MAX];
static uint32_t        g_log_n;
static uint32_t        g_log_drops;
static ID              g_k12_id = MOCK_K12_ID;

INT tk_rcv_mbf(ID id, void * p_msg, TMO tmout)
{
    INT      len = E_TMOUT;
    uint32_t idx = 0u;

    (void) tmout;
    if ((MOCK_K12_ID == id) && (g_log_n > 0u))
    {
        (void) memcpy(p_msg, &g_log_q[0], sizeof(telem_log_rec_t));
        for (idx = 1u; idx < g_log_n; idx++)
        {
            g_log_q[idx - 1u] = g_log_q[idx];
        }
        g_log_n--;
        return (INT) sizeof(telem_log_rec_t);
    }
    if ((MOCK_K11_ID == id) && (g_q_count > 0u))
    {
        (void) memcpy(p_msg, g_q_msg[0], g_q_len[0]);
        len = (INT) g_q_len[0];
        for (idx = 1u; idx < g_q_count; idx++)
        {
            (void) memcpy(g_q_msg[idx - 1u], g_q_msg[idx], MOCK_MSG_MAX);
            g_q_len[idx - 1u] = g_q_len[idx];
        }
        g_q_count--;
    }
    return len;
}

ER tk_snd_mbf(ID id, void const * p_msg, INT size, TMO tmout)
{
    (void) id;
    (void) p_msg;
    (void) size;
    (void) tmout;
    return E_OK;
}

void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    (void) p_format;
    (void) arg_a;
    (void) arg_b;
    (void) arg_c;
}

ID telem_k11_id(void)
{
    return MOCK_K11_ID;
}

ID telem_k12_id(void)
{
    return g_k12_id;
}

uint32_t telem_log_drops(void)
{
    return g_log_drops;
}

static void log_push(char const * p_fmt, int32_t a, int32_t b, int32_t c)
{
    if (g_log_n < MOCK_LOG_MAX)
    {
        g_log_q[g_log_n].p_format = p_fmt;
        g_log_q[g_log_n].arg_a    = a;
        g_log_q[g_log_n].arg_b    = b;
        g_log_q[g_log_n].arg_c    = c;
        g_log_n++;
    }
}

/* LOG payload as text (after the header byte), for comparisons */
static char g_text[200];
static uint32_t take_log_text(uint32_t size)
{
    uint8_t  out[200];
    uint32_t len = mqtt_bridge_take_log(out, size);

    (void) memset(g_text, 0, sizeof(g_text));
    if (len > 1u)
    {
        (void) memcpy(g_text, &out[1], len - 1u);
    }
    return len;
}

uint32_t sys_tick_now_us(void)
{
    return g_now_us;
}

void fault_set(uint32_t bits)
{
    g_faults |= bits;
}

void fault_clear(uint32_t bits)
{
    g_faults &= ~bits;
}

bool mission_post_event(uint32_t type, uint32_t value)
{
    if (g_posts < 16u)
    {
        g_post_type[g_posts]  = type;
        g_post_value[g_posts] = value;
    }
    g_posts++;
    return true;
}

uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    return ((char_index & 0x3Fu) | ((cmd & 0x3u) << 6u));
}

/* ---- helpers ---------------------------------------------------------- */

/* Like tk_snd_mbf, refuses a message longer than K11's maxmsz. */
static void q_push(uint8_t const * p_msg, uint32_t len)
{
    if ((len > MOCK_MSG_MAX) || (g_q_count >= MOCK_Q_MAX))
    {
        return;
    }
    (void) memcpy(g_q_msg[g_q_count], p_msg, len);
    g_q_len[g_q_count] = len;
    g_q_count++;
}

static uint32_t g_drops_seen;

/* Command drops counted since the last call. */
static uint32_t new_cmd_drops(void)
{
    uint32_t const now = app_bits_get(g_bridge_word,
                                      PRV_BRIDGE_CMD_DROPS_POS,
                                      PRV_BRIDGE_CMD_DROPS_BITS);
    uint32_t const delta = now - g_drops_seen;

    g_drops_seen = now;
    return delta;
}

static void post_cmd(uint32_t mq_type)
{
    uint8_t const byte0 = (uint8_t) telem_bin_header(mq_type);

    mqtt_bridge_post_command(&byte0, 1u);
}

/* ---- tests ------------------------------------------------------------ */

int main(void)
{
    uint8_t      buf[MQTT_BRIDGE_TX_MAX];
    uint8_t      msg[MOCK_MSG_MAX];
    uint8_t      cmd[3]  = {0u, 0u, 0u};
    uint8_t      type    = 0xFFu;
    uint32_t     len     = 0u;
    uint32_t     idx     = 0u;
    char const * p_top   = NULL;
    uint8_t      qos     = 9u;
    bool         b_ret   = true;

    HOST_INIT();

    /* S1-U12 (packed): repeated command within 100 ms dropped, counted */
    g_now_us = 5000000u;
    post_cmd(APP_MQ_TYPE_CMD_STOP);
    g_now_us += 50000u;
    post_cmd(APP_MQ_TYPE_CMD_STOP);
    HOST_CHECK("S1-U12", (1u == g_posts) && (APP_EVT_STOP == g_post_type[0]),
               "two STOPs 50 ms apart: one STOP event posted");
    HOST_CHECK("S1-U12", 1u == new_cmd_drops(),
               "two STOPs 50 ms apart: one command drop");
    g_now_us += 50000u;   /* 100 ms after the accepted one */
    post_cmd(APP_MQ_TYPE_CMD_STOP);
    HOST_CHECK("S1-U12", 2u == g_posts, "STOP 100 ms later is accepted");
    post_cmd(APP_MQ_TYPE_CMD_START);
    post_cmd(APP_MQ_TYPE_CMD_RESET);
    HOST_CHECK("S1-U12", (4u == g_posts)
                         && (APP_EVT_START == g_post_type[2])
                         && (APP_EVT_RESET == g_post_type[3]),
               "rate limit is per command type");

    /* the 16-bit 10 ms clock wraps every 655.36 s: still per-gap */
    g_now_us = 5100000u + 655360000u;  /* START was accepted at 5.1 s */
    post_cmd(APP_MQ_TYPE_CMD_START);
    HOST_CHECK("S1-U12", (4u == g_posts) && (1u == new_cmd_drops()),
               "same 16-bit stamp after a wrap is rate-limited");
    g_now_us += 100000u;
    post_cmd(APP_MQ_TYPE_CMD_START);
    HOST_CHECK("S1-U12", 5u == g_posts, "accepted 100 ms later");

    /* MAP: 2 bytes, byte 1 = MQ_MAP (char 0..42, cmd 0..3) */
    g_now_us += 1000000u;
    cmd[0] = (uint8_t) telem_bin_header(APP_MQ_TYPE_CMD_MAP);
    cmd[1] = (uint8_t) app_bits_put(app_bits_put(0u, MQ_MAP_CHAR_POS,
                                                 MQ_MAP_CHAR_BITS, 10u),
                                    MQ_MAP_CMD_POS, MQ_MAP_CMD_BITS, 3u);
    mqtt_bridge_post_command(cmd, 2u);
    HOST_CHECK("S1-U35", (6u == g_posts) && (APP_EVT_MAP == g_post_type[5])
                         && (mission_evt_barcode_value(3u, 10u)
                             == g_post_value[5]),
               "MAP command posts APP_EVT_MAP with char and command");

    /* S1-U40: SNAP ("summary now") is 1 byte 0xA1 and posts
       APP_EVT_SNAPSHOT through the mission queue, rate-limited per type */
    g_now_us += 1000000u;
    HOST_CHECK("S1-U40", 0xA1u == telem_bin_header(APP_MQ_TYPE_CMD_SNAP),
               "SNAP command byte is 0xA1");
    post_cmd(APP_MQ_TYPE_CMD_SNAP);
    HOST_CHECK("S1-U40", (7u == g_posts)
                         && (APP_EVT_SNAPSHOT == g_post_type[6])
                         && (0u == g_post_value[6]),
               "SNAP posts APP_EVT_SNAPSHOT (value 0) to mission");
    g_now_us += 50000u;
    post_cmd(APP_MQ_TYPE_CMD_SNAP);
    HOST_CHECK("S1-U40", (7u == g_posts) && (1u == new_cmd_drops()),
               "second SNAP 50 ms later is rate-limited and counted");
    g_now_us += 50000u;
    post_cmd(APP_MQ_TYPE_CMD_SNAP);
    cmd[0] = (uint8_t) telem_bin_header(APP_MQ_TYPE_CMD_SNAP);
    g_now_us += 1000000u;
    mqtt_bridge_post_command(cmd, 2u);              /* SNAP, 2 bytes */
    HOST_CHECK("S1-U40", (8u == g_posts) && (1u == new_cmd_drops()),
               "SNAP 100 ms later accepted; a 2-byte SNAP is rejected");

    /* S1-U11: wrong version or length rejected and counted */
    g_now_us += 1000000u;
    cmd[0] = (uint8_t) (telem_bin_header(APP_MQ_TYPE_CMD_STOP) + 1u);
    mqtt_bridge_post_command(cmd, 1u);              /* version 2 */
    cmd[0] = (uint8_t) telem_bin_header(APP_MQ_TYPE_CMD_STOP);
    mqtt_bridge_post_command(cmd, 2u);              /* STOP, 2 bytes */
    cmd[0] = (uint8_t) telem_bin_header(APP_MQ_TYPE_CMD_MAP);
    mqtt_bridge_post_command(cmd, 1u);              /* MAP, 1 byte */
    mqtt_bridge_post_command(cmd, 3u);              /* MAP, 3 bytes */
    cmd[1] = 43u;                                   /* char 43: invalid */
    mqtt_bridge_post_command(cmd, 2u);
    cmd[0] = (uint8_t) telem_bin_header(APP_MQ_TYPE_STATE);
    mqtt_bridge_post_command(cmd, 1u);              /* not a command */
    mqtt_bridge_post_command(NULL, 1u);
    mqtt_bridge_post_command(cmd, 0u);
    HOST_CHECK("S1-U11", (8u == g_posts) && (8u == new_cmd_drops()),
               "wrong version, length, type or char: rejected, counted");

    for (idx = 0u; idx < 300u; idx++)
    {
        bridge_count_cmd_drop();
    }
    HOST_CHECK("S1-U11", 255u == app_bits_get(g_bridge_word,
                                              PRV_BRIDGE_CMD_DROPS_POS,
                                              PRV_BRIDGE_CMD_DROPS_BITS),
               "command drops saturate at 255");
    HOST_CHECK("S1-U11", 0u == mqtt_bridge_publish_drops(),
               "command drops never touch the publish-drop field");

    /* take: the binary payload itself; type from MQ_HDR */
    HOST_CHECK("S1-U30", 0u == mqtt_bridge_take(buf, sizeof(buf), &type),
               "empty K11 returns 0");
    len = telem_bin_heartbeat(7u, 1u, 99u, msg, sizeof(msg));
    q_push(msg, len);
    idx = mqtt_bridge_take(buf, sizeof(buf), &type);
    HOST_CHECK("S1-U30", (6u == idx) && (APP_MQ_TYPE_HEARTBEAT == type)
                         && (0 == memcmp(buf, msg, 6u)),
               "take returns the type and all payload bytes");
    q_push(msg, len);
    HOST_CHECK("S1-U30", (0u == mqtt_bridge_take(buf, 4u, &type))
                         && (1u == mqtt_bridge_publish_drops()),
               "payload larger than the caller's buffer is dropped");

    /* route */
    mqtt_bridge_route(APP_MQ_TYPE_STATE, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U31", (0 == strcmp(p_top, TELEM_TOPIC_STATE))
                         && (0u == qos) && (!b_ret), "state route");
    mqtt_bridge_route(APP_MQ_TYPE_SUMMARY, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U31", (0 == strcmp(p_top, TELEM_TOPIC_EVENT))
                         && (1u == qos) && (!b_ret), "summary route");
    mqtt_bridge_route(APP_MQ_TYPE_HEARTBEAT, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U31", (0 == strcmp(p_top, TELEM_TOPIC_HB))
                         && (0u == qos) && (!b_ret), "heartbeat route");
    mqtt_bridge_route(APP_MQ_TYPE_OFFLINE, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U31", (0 == strcmp(p_top, TELEM_TOPIC_AVAIL))
                         && (1u == qos) && b_ret, "availability route");
    mqtt_bridge_route(APP_MQ_TYPE_SNAPSHOT, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U40", (0 == strcmp(p_top, TELEM_TOPIC_EVENT))
                         && (1u == qos) && (!b_ret),
               "snapshot reply goes to the event topic, QoS 1");
    mqtt_bridge_route(30u, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U31", (0 == strcmp(p_top, TELEM_TOPIC_EVENT))
                         && (0u == qos) && (!b_ret), "unknown route");

    /* availability: one header byte */
    len = mqtt_bridge_availability(true, buf, sizeof(buf));
    HOST_CHECK("S1-U32", (1u == len) && (0x29u == buf[0]),
               "online payload is the byte 0x29");
    len = mqtt_bridge_availability(false, buf, sizeof(buf));
    HOST_CHECK("S1-U32", (1u == len) && (0x31u == buf[0]),
               "offline payload is the byte 0x31");
    HOST_CHECK("S1-U32", 0u == mqtt_bridge_availability(false, buf, 0u),
               "availability that does not fit returns 0");

    /* link changes: fault bit only, never a mission event */
    idx = g_posts;
    mqtt_bridge_link_changed(false);
    HOST_CHECK("S1-U33", 0u != (g_faults & APP_FAULT_WIFI_LOSS),
               "link down sets WIFI_LOSS");
    mqtt_bridge_link_changed(true);
    HOST_CHECK("S1-U33", (0u == (g_faults & APP_FAULT_WIFI_LOSS))
                         && (idx == g_posts),
               "link up clears WIFI_LOSS; no mission event (NFR9c)");

    for (idx = 0u; idx < 300u; idx++)
    {
        mqtt_bridge_count_publish_drop();
    }
    HOST_CHECK("S1-U34", 255u == mqtt_bridge_publish_drops(),
               "publish drops saturate at 255");
    HOST_CHECK("S1-U34", 0 == strcmp(mqtt_bridge_cmd_topic(),
                                     TELEM_TOPIC_CMD), "command topic");

    /* S1-U45: the debug payloads go to their own QoS 0 topics */
    mqtt_bridge_route(APP_MQ_TYPE_SENSORS, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U45", (0 == strcmp(p_top, TELEM_TOPIC_SENS))
                         && (0u == qos) && (!b_ret),
               "SENSORS -> c28/d, QoS 0, not retained");
    mqtt_bridge_route(APP_MQ_TYPE_LOG, &p_top, &qos, &b_ret);
    HOST_CHECK("S1-U45", (0 == strcmp(p_top, TELEM_TOPIC_LOG))
                         && (0u == qos) && (!b_ret),
               "LOG -> c28/l, QoS 0, not retained");
    HOST_CHECK("S1-U45", (0 == strcmp(TELEM_TOPIC_SENS, "c28/d"))
                         && (0 == strcmp(TELEM_TOPIC_LOG, "c28/l")),
               "topic names as documented for the dashboard");

    /* S1-U46: console lines formatted like app_log(), safely */
    HOST_CHECK("S1-U46", 0u == take_log_text(sizeof(buf)),
               "nothing queued: no payload");
    log_push("US dist %d mm, lost %d\n", 536, 0, 0);
    len = take_log_text(sizeof(buf));
    HOST_CHECK("S1-U46", (0 == strcmp(g_text, "US dist 536 mm, lost 0"))
                         && ((1u + strlen(g_text)) == len),
               "%d arguments, trailing newline removed, header + text");
    {
        uint8_t hdr[4];
        log_push("x", 0, 0, 0);
        (void) mqtt_bridge_take_log(hdr, sizeof(hdr));
        HOST_CHECK("S1-U46", (0x49u == hdr[0]) && ('x' == (char) hdr[1]),
                   "byte 0 is the LOG header (version 1, type 9)");
    }
    log_push("CAL_IR heading %d cdeg, inferred 0x%x, 100%% %d\n",
             -207, 0xdb6c06, INT32_MIN);
    (void) take_log_text(sizeof(buf));
    HOST_CHECK("S1-U46", 0 == strcmp(g_text, "CAL_IR heading -207 cdeg, "
                                     "inferred 0xdb6c06, 100% -2147483648"),
               "negative, hexadecimal, %%, INT32_MIN");
    log_push("a\nb %d %d %d %d\n", 1, 2, 3);
    (void) take_log_text(sizeof(buf));
    HOST_CHECK("S1-U46", 0 == strcmp(g_text, "a b 1 2 3 0"),
               "inner newline -> space; a 4th %d prints 0");
    log_push("ENC pos L %d R %d\n", 123456, -7, 0);
    len = take_log_text(12u);  /* header + 11 text bytes */
    HOST_CHECK("S1-U46", (12u == len) && (0 == strcmp(g_text, "ENC pos L 1")),
               "never writes past the buffer (cut at the end)");
    HOST_CHECK("S1-U46", 0u == mqtt_bridge_take_log(buf, 1u),
               "a 1-byte buffer takes nothing");
    g_log_drops = 5u;
    log_push("next\n", 0, 0, 0);
    (void) take_log_text(sizeof(buf));
    HOST_CHECK("S1-U46", 0 == strcmp(g_text,
                                     "(5 console lines dropped: queue full)"),
               "K12 overflow reported first");
    (void) take_log_text(sizeof(buf));
    HOST_CHECK("S1-U46", 0 == strcmp(g_text, "next"),
               "then the queued line; the drop count is told once");
    g_k12_id = 0;
    log_push("hidden\n", 0, 0, 0);
    HOST_CHECK("S1-U46", 0u == take_log_text(sizeof(buf)),
               "no K12 (image without the mirror): nothing");

    return (0 == HOST_SUMMARY("mqtt_bridge")) ? 0 : 1;
}

/*** end of file ***/
