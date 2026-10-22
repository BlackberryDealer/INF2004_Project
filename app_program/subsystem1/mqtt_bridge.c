/**
 * @file    mqtt_bridge.c
 * @brief   Kernel-side functions the radio task (mqtt_utk.c) calls: take the
 *          next payload from K11, route it, build the availability payload,
 *          validate and rate-limit remote commands, report link changes.
 *          Packed representation (B1.4): binary payloads and commands,
 *          counters in the PRV_BRIDGE word.
 * @owner   Buddy 1 (communications)
 * @kind    task
 * @reqs    FR10, FR11, FR12, FR20, NFR9
 */

#include "mqtt_bridge.h"       /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "../app_layout.h"
#include "../app_bits.h"
#include "cfg_telem.h"
#include "telem.h"
#include "telem_bin.h"
#include "../subsystem2/fault.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/sys_tick.h"

/* ---- private constants and types ------------------------------------ */

/* Index of each command in the rate-limit table. */
#define BRIDGE_CMD_START      (0u)
#define BRIDGE_CMD_STOP       (1u)
#define BRIDGE_CMD_RESET      (2u)
#define BRIDGE_CMD_MAP        (3u)
#define BRIDGE_CMD_COUNT      (4u)
#define BRIDGE_CMD_NONE       (BRIDGE_CMD_COUNT)

#define BRIDGE_US_PER_10MS    (10000u)
#define BRIDGE_CMD_GAP_10MS   (TELEM_CMD_MIN_GAP_MS / 10u)
#define BRIDGE_CMD_LEN        (1u)     /* START, STOP, RESET: header only */
#define BRIDGE_MAP_LEN        (2u)     /* MAP: header + MQ_MAP byte       */

#if (BRIDGE_CMD_GAP_10MS < 1u) || (BRIDGE_CMD_GAP_10MS > PRV_CMD_T_LAST_MAX)
#error "TELEM_CMD_MIN_GAP_MS no longer fits the 16-bit 10 ms time stamps"
#endif
#if (BRIDGE_CMD_COUNT > PRV_BRIDGE_CMD_SEEN_BITS)
#error "PRV_BRIDGE.CMD_SEEN needs one bit per command type: widen it"
#endif

/* ---- private data ---------------------------------------------------- */

/* Both written only by the radio task (rule 1). PRV_CMD_T: one 16-bit
   time stamp per command type. PRV_BRIDGE (command drops, publish drops,
   seen mask) is read by telem_task as one aligned word (atomic).       */
static uint16_t          g_cmd_last_10ms[BRIDGE_CMD_COUNT] =
{
    0u, 0u, 0u, 0u
};
static volatile uint32_t g_bridge_word = 0u;

/* ---- private function prototypes ------------------------------------- */

static uint32_t bridge_parse_command(uint8_t const * p_payload,
                                     uint32_t length, uint32_t * p_map);
static bool     bridge_rate_ok(uint32_t cmd_idx);
static void     bridge_count_cmd_drop(void);
static void     bridge_count(uint32_t pos, uint32_t bits, int32_t max);

/* ---- public functions ------------------------------------------------- */

/**
 * mqtt_bridge_take
 * @brief   Takes the next queued payload from K11 without waiting. The
 *          K11 message is the binary payload; its type is MQ_HDR.TYPE.
 * @param   p_buf      output buffer
 * @param   buf_size   size of p_buf
 * @param   p_mq_type  receives APP_MQ_TYPE_*
 * @return  payload length copied, 0 if nothing is queued (or it does not
 *          fit in p_buf, then it is dropped)
 * @context radio task
 */
uint32_t mqtt_bridge_take(uint8_t * p_buf, uint32_t buf_size,
                          uint8_t * p_mq_type)
{
    uint8_t  local[MQTT_BRIDGE_TX_MAX + 1u] = {0u};
    uint32_t copied = 0u;
    uint32_t idx    = 0u;
    INT      rcv    = 0;

    if ((NULL != p_buf) && (NULL != p_mq_type) && (0 < telem_k11_id()))
    {
        rcv = tk_rcv_mbf(telem_k11_id(), local, TMO_POL);
    }
    if (rcv > 0)
    {
        copied = (uint32_t) rcv;   /* cast: rcv > 0 */
        if (copied <= buf_size)
        {
            /* cast: a 5-bit field, < 32 */
            *p_mq_type = (uint8_t) app_bits_get(local[0], MQ_HDR_TYPE_POS,
                                                MQ_HDR_TYPE_BITS);
            for (idx = 0u; idx < copied; idx++)
            {
                p_buf[idx] = local[idx];
            }
        }
        else
        {
            copied = 0u;
            mqtt_bridge_count_publish_drop();
        }
    }

    return copied;
}

/**
 * mqtt_bridge_route
 * @brief   Topic, QoS and retain flag for a payload type: STATE -> state
 *          topic QoS 0; EVENT, PROFILE, SUMMARY -> event topic QoS 1;
 *          HEARTBEAT -> hb topic QoS 0; ONLINE/OFFLINE -> availability
 *          topic QoS 1 retained; anything else -> event topic QoS 0.
 * @param   mq_type         APP_MQ_TYPE_*
 * @param   pp_topic        receives the topic
 * @param   p_qos           receives the QoS
 * @param   pb_is_retained  receives the retain flag
 * @return  none
 * @context radio task
 */
void mqtt_bridge_route(uint8_t mq_type, char const ** pp_topic, uint8_t * p_qos,
                       bool * pb_is_retained)
{
    char const * p_topic       = TELEM_TOPIC_EVENT;
    uint8_t      qos           = 0u;
    bool         b_is_retained = false;

    switch (mq_type)
    {
        case APP_MQ_TYPE_STATE:
            p_topic = TELEM_TOPIC_STATE;
            break;

        case APP_MQ_TYPE_EVENT:
        case APP_MQ_TYPE_PROFILE:
        case APP_MQ_TYPE_SUMMARY:
            qos = 1u;
            break;

        case APP_MQ_TYPE_HEARTBEAT:
            p_topic = TELEM_TOPIC_HB;
            break;

        case APP_MQ_TYPE_ONLINE:
        case APP_MQ_TYPE_OFFLINE:
            p_topic       = TELEM_TOPIC_AVAIL;
            qos           = 1u;
            b_is_retained = true;
            break;

        default:
            /* unknown: event topic, QoS 0 */
            break;
    }

    if (NULL != pp_topic)
    {
        *pp_topic = p_topic;
    }
    if (NULL != p_qos)
    {
        *p_qos = qos;
    }
    if (NULL != pb_is_retained)
    {
        *pb_is_retained = b_is_retained;
    }
}

/**
 * mqtt_bridge_availability
 * @brief   Availability payload: one header byte of type ONLINE (0x29) or
 *          OFFLINE (0x31).
 * @param   b_is_online  true = online
 * @param   p_buf        output buffer
 * @param   size         size of p_buf
 * @return  1, or 0 if it does not fit
 * @context radio task
 */
uint32_t mqtt_bridge_availability(bool b_is_online, uint8_t * p_buf,
                                  uint32_t size)
{
    uint32_t len  = 0u;
    uint32_t type = APP_MQ_TYPE_OFFLINE;

    if (b_is_online)
    {
        type = APP_MQ_TYPE_ONLINE;
    }
    if ((NULL != p_buf) && (size >= 1u))
    {
        p_buf[0] = (uint8_t) telem_bin_header(type); /* cast: one byte */
        len      = 1u;
    }

    return len;
}

/**
 * mqtt_bridge_post_command
 * @brief   Validates a binary command (version 1; START / STOP / RESET
 *          are 1 byte, MAP is 2 bytes with the MQ_MAP fields in byte 1),
 *          applies the 100 ms per-type rate limit and posts it to the
 *          mission queue. Anything else counts as a command drop.
 * @param   p_payload  received bytes
 * @param   length     payload length
 * @return  none
 * @context radio task
 */
void mqtt_bridge_post_command(uint8_t const * p_payload, uint32_t length)
{
    uint32_t       map_val = 0u;
    uint32_t const cmd_idx = bridge_parse_command(p_payload, length,
                                                  &map_val);
    uint32_t       evt     = APP_EVT_START;
    uint32_t       value   = 0u;

    if (BRIDGE_CMD_NONE == cmd_idx)
    {
        bridge_count_cmd_drop();
    }
    else if (!bridge_rate_ok(cmd_idx))
    {
        bridge_count_cmd_drop();
    }
    else
    {
        switch (cmd_idx)
        {
            case BRIDGE_CMD_STOP:
                evt = APP_EVT_STOP;
                break;

            case BRIDGE_CMD_RESET:
                evt = APP_EVT_RESET;
                break;

            case BRIDGE_CMD_MAP:
                evt   = APP_EVT_MAP;
                value = map_val;
                break;

            default:   /* BRIDGE_CMD_START */
                evt = APP_EVT_START;
                break;
        }
        /* a full K4 sets APP_FAULT_QUEUE_OVERFLOW inside mission */
        (void) mission_post_event(evt, value);
    }
}

/**
 * mqtt_bridge_link_changed
 * @brief   Broker link went up or down: clears or sets APP_FAULT_WIFI_LOSS.
 *          Never posts a mission event (NFR9c: autonomy unaffected).
 * @param   b_is_up  true = broker connected
 * @return  none
 * @context radio task
 */
void mqtt_bridge_link_changed(bool b_is_up)
{
    if (b_is_up)
    {
        fault_clear(APP_FAULT_WIFI_LOSS);
    }
    else
    {
        fault_set(APP_FAULT_WIFI_LOSS);
    }
}

/**
 * mqtt_bridge_count_publish_drop
 * @brief   Counts one payload the radio code gave up on (saturates at 255).
 * @return  none
 * @context radio task
 */
void mqtt_bridge_count_publish_drop(void)
{
    bridge_count(PRV_BRIDGE_PUB_DROPS_POS, PRV_BRIDGE_PUB_DROPS_BITS,
                 PRV_BRIDGE_PUB_DROPS_MAX);
}

/**
 * mqtt_bridge_publish_drops
 * @brief   Publish drops so far (one aligned word read, atomic).
 * @return  0..255
 * @context any task
 */
uint32_t mqtt_bridge_publish_drops(void)
{
    return app_bits_get(g_bridge_word, PRV_BRIDGE_PUB_DROPS_POS,
                        PRV_BRIDGE_PUB_DROPS_BITS);
}

/**
 * mqtt_bridge_cmd_topic
 * @brief   The command topic to subscribe to.
 * @return  NUL-terminated topic string
 * @context radio task
 */
char const * mqtt_bridge_cmd_topic(void)
{
    return TELEM_TOPIC_CMD;
}

/* ---- private functions ------------------------------------------------ */

/**
 * bridge_parse_command
 * @brief   Checks version and length and returns the command. For MAP,
 *          builds the mission event value from the MQ_MAP byte.
 * @param   p_payload  received bytes (may be NULL)
 * @param   length     payload length
 * @param   p_map      receives the MAP event value (MAP only)
 * @return  BRIDGE_CMD_* index, or BRIDGE_CMD_NONE
 * @context radio task
 */
static uint32_t bridge_parse_command(uint8_t const * p_payload,
                                     uint32_t length, uint32_t * p_map)
{
    uint32_t cmd_idx = BRIDGE_CMD_NONE;
    uint32_t version = 0u;
    uint32_t type    = 0u;
    uint32_t ch_idx  = 0u;

    if ((NULL != p_payload) && (length >= BRIDGE_CMD_LEN))
    {
        version = app_bits_get(p_payload[0], MQ_HDR_VER_POS, MQ_HDR_VER_BITS);
        type    = app_bits_get(p_payload[0], MQ_HDR_TYPE_POS,
                               MQ_HDR_TYPE_BITS);
    }
    if (APP_MQ_VERSION != version)
    {
        cmd_idx = BRIDGE_CMD_NONE;   /* also: no payload at all */
    }
    else if (BRIDGE_CMD_LEN == length)
    {
        if (APP_MQ_TYPE_CMD_START == type)
        {
            cmd_idx = BRIDGE_CMD_START;
        }
        else if (APP_MQ_TYPE_CMD_STOP == type)
        {
            cmd_idx = BRIDGE_CMD_STOP;
        }
        else if (APP_MQ_TYPE_CMD_RESET == type)
        {
            cmd_idx = BRIDGE_CMD_RESET;
        }
        else
        {
            cmd_idx = BRIDGE_CMD_NONE;
        }
    }
    else if ((BRIDGE_MAP_LEN == length) && (APP_MQ_TYPE_CMD_MAP == type))
    {
        ch_idx = app_bits_get(p_payload[1], MQ_MAP_CHAR_POS,
                              MQ_MAP_CHAR_BITS);
        if (ch_idx <= (uint32_t) MQ_MAP_CHAR_MAX)  /* cast: 42; ENUM */
        {
            cmd_idx = BRIDGE_CMD_MAP;
            *p_map  = mission_evt_barcode_value(
                          app_bits_get(p_payload[1], MQ_MAP_CMD_POS,
                                       MQ_MAP_CMD_BITS), ch_idx);
        }
    }
    else
    {
        cmd_idx = BRIDGE_CMD_NONE;   /* wrong length for the type */
    }

    return cmd_idx;
}

/**
 * bridge_rate_ok
 * @brief   One command per type per TELEM_CMD_MIN_GAP_MS. Times are kept
 *          as 16-bit counts of 10 ms; only differences of 100 ms matter,
 *          so the wrap every 655 s is harmless. Records the time when the
 *          command is accepted.
 * @param   cmd_idx  BRIDGE_CMD_* (< BRIDGE_CMD_COUNT)
 * @return  true if the command may be posted
 * @context radio task (division allowed: event time)
 */
static bool bridge_rate_ok(uint32_t cmd_idx)
{
    /* cast: keep the low 16 bits of the 10 ms count; wraps by design */
    uint16_t const now_10ms = (uint16_t) (sys_tick_now_us()
                                          / BRIDGE_US_PER_10MS);
    uint32_t const bit      = 1u << cmd_idx;
    uint32_t const seen     = app_bits_get(g_bridge_word,
                                           PRV_BRIDGE_CMD_SEEN_POS,
                                           PRV_BRIDGE_CMD_SEEN_BITS);
    bool           b_is_ok  = false;
    uint16_t       gap      = 0u;

    if (0u == (seen & bit))
    {
        b_is_ok = true;
    }
    else
    {
        /* cast: unsigned 16-bit difference, wraps by design */
        gap     = (uint16_t) (now_10ms - g_cmd_last_10ms[cmd_idx]);
        b_is_ok = (gap >= BRIDGE_CMD_GAP_10MS);
    }

    if (b_is_ok)
    {
        g_cmd_last_10ms[cmd_idx] = now_10ms;
        g_bridge_word = app_bits_put(g_bridge_word, PRV_BRIDGE_CMD_SEEN_POS,
                                     PRV_BRIDGE_CMD_SEEN_BITS, seen | bit);
    }

    return b_is_ok;
}

/**
 * bridge_count_cmd_drop
 * @brief   Counts one rejected or rate-limited command (saturates at 255).
 * @context radio task
 */
static void bridge_count_cmd_drop(void)
{
    bridge_count(PRV_BRIDGE_CMD_DROPS_POS, PRV_BRIDGE_CMD_DROPS_BITS,
                 PRV_BRIDGE_CMD_DROPS_MAX);
}

/**
 * bridge_count
 * @brief   Saturating increment of one PRV_BRIDGE counter (the radio task
 *          is the word's only writer, so no lock is needed).
 * @param   pos   layout _POS
 * @param   bits  layout _BITS
 * @param   max   layout _MAX (saturation value)
 * @context radio task
 */
static void bridge_count(uint32_t pos, uint32_t bits, int32_t max)
{
    uint32_t const count = app_bits_get(g_bridge_word, pos, bits);

    if (count < (uint32_t) max)   /* cast: positive layout constant */
    {
        g_bridge_word = app_bits_put(g_bridge_word, pos, bits, count + 1u);
    }
}

/*** end of file ***/
