/**
 * @file    mqtt_bridge.c
 * @brief   Phase 0 STUB of the radio-task bridge: nothing to publish, commands
 *          ignored.
 * @owner   Buddy 1 (communications)
 * @kind    task
 * @reqs    FR10, FR11, FR12, FR20
 */

#include "mqtt_bridge.h"       /* own header first (BARR-C 4.3.c) */
#include <stddef.h>              /* NULL */

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * mqtt_bridge_take
 * @brief   STUB: copy the next queued payload. Never blocks.
 * @param   p_buf      output buffer
 * @param   buf_size   size of p_buf
 * @param   p_mq_type  receives APP_MQ_TYPE_*
 * @return  0 (stub: nothing queued)
 * @context radio task
 */
uint32_t mqtt_bridge_take(uint8_t * p_buf, uint32_t buf_size,
                          uint8_t * p_mq_type)
{
    (void) p_buf;   /* stub: unused until the owner's version */
    (void) buf_size;
    (void) p_mq_type;

    return 0u;
}

/**
 * mqtt_bridge_route
 * @brief   STUB: topic, QoS and retain flag for a payload type. Returns an
 *          empty topic, QoS 0, not retained.
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
    (void) mq_type;   /* stub: one route for every type */

    if (NULL != pp_topic)
    {
        *pp_topic = "";
    }
    if (NULL != p_qos)
    {
        *p_qos = 0u;
    }
    if (NULL != pb_is_retained)
    {
        *pb_is_retained = false;
    }
}

/**
 * mqtt_bridge_availability
 * @brief   STUB: availability payload.
 * @param   b_is_online  true = online
 * @param   p_buf        output buffer
 * @param   size         size of p_buf
 * @return  0 (stub: empty payload)
 * @context radio task
 */
uint32_t mqtt_bridge_availability(bool b_is_online, uint8_t * p_buf,
                                  uint32_t size)
{
    (void) b_is_online;   /* stub: unused until the owner's version */
    (void) p_buf;
    (void) size;

    return 0u;
}

/**
 * mqtt_bridge_post_command
 * @brief   STUB: validate a remote command and post it to mission.
 * @param   p_payload  received bytes
 * @param   length     payload length
 * @return  none
 * @context radio task
 */
void mqtt_bridge_post_command(uint8_t const * p_payload, uint32_t length)
{
    (void) p_payload;   /* stub: unused until the owner's version */
    (void) length;
}

/**
 * mqtt_bridge_link_changed
 * @brief   STUB: broker link went up or down.
 * @param   b_is_up  true = broker connected
 * @return  none
 * @context radio task
 */
void mqtt_bridge_link_changed(bool b_is_up)
{
    (void) b_is_up;   /* stub: unused until the owner's version */
}

/**
 * mqtt_bridge_count_publish_drop
 * @brief   STUB: count one payload the radio code gave up on.
 * @return  none
 * @context radio task
 */
void mqtt_bridge_count_publish_drop(void)
{
}

/**
 * mqtt_bridge_publish_drops
 * @brief   STUB: publish drops so far.
 * @return  0 (stub)
 * @context any task
 */
uint32_t mqtt_bridge_publish_drops(void)
{
    return 0u;
}

/**
 * mqtt_bridge_cmd_topic
 * @brief   STUB: the command topic to subscribe to.
 * @return  empty string (stub)
 * @context radio task
 */
char const * mqtt_bridge_cmd_topic(void)
{
    return "";
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
