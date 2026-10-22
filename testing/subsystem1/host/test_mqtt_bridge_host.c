/**
 * @file    test_mqtt_bridge_host.c
 * @brief   Laptop tests of mqtt_bridge.c against a mock kernel: binary
 *          command validation (S1-U11), rate limit (S1-U12), MAP
 *          (S1-U35), K11 take, routing,
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
#define MOCK_MSG_MAX    (MQTT_BRIDGE_TX_MAX + 1u)

static uint8_t  g_q_msg[MOCK_Q_MAX][MOCK_MSG_MAX];
static uint32_t g_q_len[MOCK_Q_MAX];
static uint32_t g_q_count;
static uint32_t g_now_us;
static uint32_t g_faults;
static uint32_t g_posts;
static uint32_t g_post_type[16];
static uint32_t g_post_value[16];

INT tk_rcv_mbf(ID id, void * p_msg, TMO tmout)
{
    INT      len = E_TMOUT;
    uint32_t idx = 0u;

    (void) tmout;
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

static void q_push(uint8_t const * p_msg, uint32_t len)
{
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
    HOST_CHECK("S1-U11", (6u == g_posts) && (8u == new_cmd_drops()),
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

    return (0 == HOST_SUMMARY("mqtt_bridge")) ? 0 : 1;
}

/*** end of file ***/
