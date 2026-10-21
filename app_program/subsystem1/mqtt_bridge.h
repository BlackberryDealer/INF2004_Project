/**
 * @file    mqtt_bridge.h
 * @brief   The ONLY functions mqtt_utk.c (radio task) may call. Must stay free
 *          of kernel headers: mqtt_utk.c cannot include them.
 * @owner   Buddy 1 (communications)
 * @kind    task
 */

#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#define MQTT_BRIDGE_TX_MAX   (160u)   /* largest payload (baseline JSON) */

/* Copy the next queued payload into p_buf and its APP_MQ_TYPE_* into
   *p_mq_type. Returns the payload length, 0 if nothing is queued.
   Never blocks.                                                      */
extern uint32_t mqtt_bridge_take(uint8_t * p_buf, uint32_t buf_size,
                                 uint8_t * p_mq_type);

/* Topic, QoS and retain flag for an APP_MQ_TYPE_* value. */
extern void     mqtt_bridge_route(uint8_t mq_type, char const ** pp_topic,
                                  uint8_t * p_qos, bool * pb_is_retained);

/* Availability payload ("online"/"offline" text in the baseline build,
   one byte in the packed build). Returns its length.                 */
extern uint32_t mqtt_bridge_availability(bool b_is_online,
                                         uint8_t * p_buf, uint32_t size);

/* A complete payload received on the command topic. Validates,
   rate-limits, posts START / STOP / RESET / MAP to the mission queue. */
extern void     mqtt_bridge_post_command(uint8_t const * p_payload,
                                         uint32_t length);

/* Broker connection went up (true) or down (false). */
extern void     mqtt_bridge_link_changed(bool b_is_up);

/* The radio code gave up on a payload (publish refused twice). Counted
   in the bridge's own word; reported in the heartbeat drops.          */
extern void     mqtt_bridge_count_publish_drop(void);
extern uint32_t mqtt_bridge_publish_drops(void);   /* 0..255, for telem */

/* The command topic string the radio task subscribes to. */
extern char const * mqtt_bridge_cmd_topic(void);

#endif /* MQTT_BRIDGE_H */

/*** end of file ***/
