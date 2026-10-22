/**
 * @file    test_mqtt_utk_host.c
 * @brief   Laptop tests of the radio-task MQTT code (mqtt_utk.c, SHARED
 *          B1.2.5) against mock lwIP and mock bridge functions: pending
 *          payload and publish refusals (S1-U13, S1-U15), reconnect
 *          back-off (S1-U36), last will, availability and command
 *          forwarding (S1-U37). The real source is included so its
 *          private state can be checked.
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    FR10, FR11, FR20, NFR9
 */

#include "host_test.h"
#include "../../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"

#include <stdio.h>
#include <string.h>

/* ---- mock lwIP -------------------------------------------------------- */

#define MOCK_PUB_MAX  (32u)

static mqtt_client_t      g_client_obj;
static u8_t               g_is_connected;
static mqtt_connection_cb_t g_conn_cb;
static struct mqtt_connect_client_info_t g_info;
static err_t              g_connect_ret = ERR_OK;
static uint32_t           g_connects;
static err_t              g_pub_script[MOCK_PUB_MAX];
static uint32_t           g_pub_calls;
static uint32_t           g_pub_ok;
static uint8_t            g_pub_first[MOCK_PUB_MAX];
static char const *       g_pub_topic[MOCK_PUB_MAX];
static u8_t               g_pub_retain[MOCK_PUB_MAX];
static char const *       g_sub_topic;
static u8_t               g_sub_qos;
static mqtt_incoming_publish_cb_t g_pub_cb;
static mqtt_incoming_data_cb_t    g_data_cb;
static uint32_t           g_retry_ms;
static uint32_t           g_retry_armed;
static uint32_t           g_drain_armed;

mqtt_client_t * mqtt_client_new(void)
{
    return &g_client_obj;
}

u8_t mqtt_client_is_connected(mqtt_client_t * p_client)
{
    (void) p_client;
    return g_is_connected;
}

void mqtt_set_inpub_callback(mqtt_client_t * p_client,
                             mqtt_incoming_publish_cb_t pub_cb,
                             mqtt_incoming_data_cb_t data_cb, void * arg)
{
    (void) p_client;
    (void) arg;
    g_pub_cb  = pub_cb;
    g_data_cb = data_cb;
}

err_t mqtt_subscribe(mqtt_client_t * p_client, const char * topic, u8_t qos,
                     mqtt_request_cb_t cb, void * arg)
{
    (void) p_client;
    (void) cb;
    (void) arg;
    g_sub_topic = topic;
    g_sub_qos   = qos;
    return ERR_OK;
}

/* Returns the scripted result for each call (ERR_OK when unscripted). */
err_t mqtt_publish(mqtt_client_t * p_client, const char * topic,
                   const void * payload, u16_t payload_length, u8_t qos,
                   u8_t retain, mqtt_request_cb_t cb, void * arg)
{
    err_t err = ERR_OK;

    (void) p_client;
    (void) payload_length;
    (void) qos;
    (void) cb;
    (void) arg;
    if (g_pub_calls < MOCK_PUB_MAX)
    {
        err = g_pub_script[g_pub_calls];
        g_pub_first[g_pub_calls]  = ((const uint8_t *) payload)[0];
        g_pub_topic[g_pub_calls]  = topic;
        g_pub_retain[g_pub_calls] = retain;
    }
    g_pub_calls++;
    if (ERR_OK == err)
    {
        g_pub_ok++;
    }
    return err;
}

err_t mqtt_client_connect(mqtt_client_t * p_client, const void * ipaddr,
                          u16_t port, mqtt_connection_cb_t cb, void * arg,
                          const struct mqtt_connect_client_info_t * info)
{
    (void) p_client;
    (void) ipaddr;
    (void) port;
    (void) arg;
    g_conn_cb = cb;
    g_info    = *info;
    g_connects++;
    return g_connect_ret;
}

int ip4addr_aton(const char * cp, ip_addr_t * addr)
{
    (void) cp;
    addr->addr = 1u;
    return 1;
}

void sys_timeout(uint32_t msecs, sys_timeout_handler handler, void * arg)
{
    (void) arg;
    if (retry_timeout == handler)
    {
        g_retry_ms = msecs;
        g_retry_armed++;
    }
    else if (drain_timeout == handler)
    {
        g_drain_armed++;
    }
    else
    {
        /* not tracked */
    }
}

void sys_untimeout(sys_timeout_handler handler, void * arg)
{
    (void) handler;
    (void) arg;
}

int tm_printf(UB * format, ...)
{
    (void) format;
    return 0;
}

/* ---- mock bridge (the real one is tested in test_mqtt_bridge_host) ---- */

#define MOCK_TAKE_MAX (8u)

static uint8_t  g_take_first[MOCK_TAKE_MAX];
static uint32_t g_take_count;
static uint32_t g_take_next;
static uint32_t g_pub_drops;
static int      g_link_up = -1;
static uint32_t g_cmd_len;
static uint8_t  g_cmd_byte0;
static uint32_t g_cmds;

uint32_t mqtt_bridge_take(uint8_t * p_buf, uint32_t buf_size,
                          uint8_t * p_mq_type)
{
    uint32_t len = 0u;

    if ((g_take_next < g_take_count) && (buf_size >= 3u))
    {
        p_buf[0]   = g_take_first[g_take_next];
        p_buf[1]   = 0u;
        p_buf[2]   = 0u;
        *p_mq_type = APP_MQ_TYPE_EVENT;
        g_take_next++;
        len = 3u;
    }
    return len;
}

void mqtt_bridge_route(uint8_t mq_type, char const ** pp_topic,
                       uint8_t * p_qos, bool * pb_is_retained)
{
    *pp_topic       = (APP_MQ_TYPE_OFFLINE == mq_type) ? "avail"
                      : (APP_MQ_TYPE_ONLINE == mq_type) ? "avail" : "event";
    *p_qos          = 1u;
    *pb_is_retained = ((APP_MQ_TYPE_OFFLINE == mq_type)
                       || (APP_MQ_TYPE_ONLINE == mq_type));
}

uint32_t mqtt_bridge_availability(bool b_is_online, uint8_t * p_buf,
                                  uint32_t size)
{
    (void) size;
    p_buf[0] = b_is_online ? 0x29u : 0x31u;
    return 1u;
}

void mqtt_bridge_post_command(uint8_t const * p_payload, uint32_t length)
{
    g_cmds++;
    g_cmd_len   = length;
    g_cmd_byte0 = (length > 0u) ? p_payload[0] : 0u;
}

void mqtt_bridge_link_changed(bool b_is_up)
{
    g_link_up = b_is_up ? 1 : 0;
}

void mqtt_bridge_count_publish_drop(void)
{
    g_pub_drops++;
}

uint32_t mqtt_bridge_publish_drops(void)
{
    return g_pub_drops;
}

char const * mqtt_bridge_cmd_topic(void)
{
    return "c28/c";
}

/* ---- helpers ---------------------------------------------------------- */

#define PEND_LEN()   mq_get(PRV_MQTT_PEND_LEN_POS, PRV_MQTT_PEND_LEN_BITS)
#define PEND_TRIES() mq_get(PRV_MQTT_PEND_TRIES_POS, PRV_MQTT_PEND_TRIES_BITS)

static void queue_payloads(uint8_t first_a, uint8_t first_b)
{
    g_take_first[0] = first_a;
    g_take_first[1] = first_b;
    g_take_count    = 2u;
    g_take_next     = 0u;
}

static void reset_publish(void)
{
    (void) memset(g_pub_script, 0, sizeof(g_pub_script));
    g_pub_calls = 0u;
    g_pub_ok    = 0u;
    g_pub_drops = 0u;
    pend_clear();
}

/* ---- tests ------------------------------------------------------------ */

int main(void)
{
    uint8_t const cmd[2] = {0x89u, 0x2Au};
    uint32_t      idx    = 0u;
    bool          b_ok   = true;

    HOST_INIT();

    /* S1-U36: connect, refuse, back-off 1 2 4 8 8 s, reset on accept */
    mqtt_utk_start();
    HOST_CHECK("S1-U37", (1u == g_connects) && (4u == g_info.keep_alive),
               "connect uses the 4 s keepalive");
    HOST_CHECK("S1-U37", (NULL != g_info.will_topic)
                         && (0 == strcmp(g_info.will_topic, "avail"))
                         && (1u == g_info.will_msg_len)
                         && (0x31u == (uint8_t) g_info.will_msg[0])
                         && (1u == g_info.will_qos)
                         && (1u == g_info.will_retain),
               "retained QoS 1 last will carries the offline payload");
    for (idx = 0u; idx < 5u; idx++)
    {
        uint32_t const expect[5] = {1000u, 2000u, 4000u, 8000u, 8000u};
        g_conn_cb(&g_client_obj, NULL, MQTT_CONNECT_TIMEOUT);
        b_ok = b_ok && (expect[idx] == g_retry_ms) && (1u == connecting);
        mqtt_utk_start();   /* the poll loop must not skip the wait */
        b_ok = b_ok && ((idx + 1u) == g_connects);
        retry_timeout(NULL);
        b_ok = b_ok && ((idx + 2u) == g_connects);
    }
    HOST_CHECK("S1-U36", b_ok,
               "retries at 1, 2, 4, 8, 8 s; poll loop cannot cut the wait");
    HOST_CHECK("S1-U36", 0 == g_link_up, "refused link reported down");

    g_connect_ret = ERR_RTE;
    g_conn_cb(&g_client_obj, NULL, MQTT_CONNECT_DISCONNECTED);
    retry_timeout(NULL);
    HOST_CHECK("S1-U36", (8000u == g_retry_ms) && (1u == connecting),
               "an immediate connect failure takes the back-off path");
    g_connect_ret = ERR_OK;
    retry_timeout(NULL);

    g_is_connected = 1u;
    g_pub_calls    = 0u;
    g_conn_cb(&g_client_obj, NULL, MQTT_CONNECT_ACCEPTED);
    HOST_CHECK("S1-U37", (1 == g_link_up) && (0u == connecting)
                         && (0u == mq_get(PRV_MQTT_BACKOFF_POS, PRV_MQTT_BACKOFF_BITS)),
               "accepted: link up, back-off reset");
    HOST_CHECK("S1-U37", (1u == g_pub_calls) && (0x29u == g_pub_first[0])
                         && (0 == strcmp(g_pub_topic[0], "avail"))
                         && (1u == g_pub_retain[0]),
               "accepted: retained online availability published");
    HOST_CHECK("S1-U37", (NULL != g_sub_topic)
                         && (0 == strcmp(g_sub_topic, "c28/c"))
                         && (1u == g_sub_qos),
               "subscribed to the bridge's command topic at QoS 1");
    HOST_CHECK("S1-U37", g_drain_armed > 0u, "drain chain started");

    /* commands: raw bytes on the command topic reach the bridge */
    g_pub_cb(NULL, "c28/c", 2u);
    g_data_cb(NULL, cmd, 2u, MQTT_DATA_FLAG_LAST);
    HOST_CHECK("S1-U37", (1u == g_cmds) && (2u == g_cmd_len)
                         && (0x89u == g_cmd_byte0),
               "binary command bytes forwarded unfiltered");
    g_pub_cb(NULL, "other/topic", 2u);
    g_data_cb(NULL, cmd, 2u, MQTT_DATA_FLAG_LAST);
    g_pub_cb(NULL, "c28/cx", 2u);
    g_data_cb(NULL, cmd, 2u, MQTT_DATA_FLAG_LAST);
    HOST_CHECK("S1-U37", 1u == g_cmds, "other topics are ignored");

    /* S1-U13: refused twice -> dropped and counted; next one sent */
    reset_publish();
    queue_payloads(0xA1u, 0xB2u);
    g_pub_script[0] = ERR_MEM;
    g_pub_script[1] = ERR_MEM;
    drain_timeout(NULL);   /* try 1 of A, stop for this call */
    HOST_CHECK("S1-U13", (1u == g_pub_calls) && (3u == PEND_LEN())
                         && (1u == PEND_TRIES()),
               "first refusal keeps A pending");
    drain_timeout(NULL);   /* try 2 of A: dropped */
    HOST_CHECK("S1-U13", (1u == g_pub_drops) && (0u == PEND_LEN()),
               "second refusal drops A and counts it");
    drain_timeout(NULL);   /* B is taken and sent */
    HOST_CHECK("S1-U13", (3u == g_pub_calls) && (1u == g_pub_ok)
                         && (0xB2u == g_pub_first[2])
                         && (1u == g_pub_drops),
               "next payload is still sent");

    /* S1-U15: A refused once then sent; B refused once then sent */
    reset_publish();
    queue_payloads(0xA1u, 0xB2u);
    g_pub_script[0] = ERR_MEM;   /* A try 1 */
    g_pub_script[1] = ERR_OK;    /* A try 2 */
    g_pub_script[2] = ERR_MEM;   /* B try 1 */
    g_pub_script[3] = ERR_OK;    /* B try 2 */
    drain_timeout(NULL);
    drain_timeout(NULL);   /* A sent, B taken, tries 0 -> refused once */
    HOST_CHECK("S1-U15", (3u == g_pub_calls) && (3u == PEND_LEN())
                         && (1u == PEND_TRIES()) && (0u == g_pub_drops),
               "B starts at 0 tries: one refusal keeps it pending");
    drain_timeout(NULL);
    HOST_CHECK("S1-U15", (4u == g_pub_calls) && (2u == g_pub_ok)
                         && (0xB2u == g_pub_first[3])
                         && (0u == g_pub_drops),
               "B is retried and sent, not dropped");

    /* two publishes per call, nothing when disconnected */
    reset_publish();
    queue_payloads(0xA1u, 0xB2u);
    g_take_first[2] = 0xC3u;
    g_take_count    = 3u;
    drain_timeout(NULL);
    HOST_CHECK("S1-U13", 2u == g_pub_calls, "at most 2 publishes per call");
    g_is_connected = 0u;
    drain_timeout(NULL);
    HOST_CHECK("S1-U13", 2u == g_pub_calls, "no publish while disconnected");

    /* a late QoS 1 failure is counted too */
    reset_publish();
    publish_done_cb(NULL, ERR_TIMEOUT);
    publish_done_cb(NULL, ERR_OK);
    HOST_CHECK("S1-U13", 1u == g_pub_drops,
               "late publish failure (publish_done_cb) counted");

    return (0 == HOST_SUMMARY("mqtt_utk")) ? 0 : 1;
}

/*** end of file ***/
