/**
 * @file    test_lwip_sizing_host.c
 * @brief   S1-U38: the lwIP MQTT request slots and output ring (real
 *          lwipopts.h) hold 4 s of telemetry, so a Wi-Fi stall shorter
 *          than the keepalive delays messages instead of dropping them.
 * @owner   Buddy 1 (communications)
 * @kind    test
 * @reqs    NFR8, NFR9
 */

#include "host_test.h"
#include "cfg_telem.h"
#include "app_types.h"
#include "../../../lib/liblwip/sysdepend/pico_rp2040/lwipopts.h"

#include <stdint.h>
#include <string.h>

/* lwIP's own defaults when lwipopts.h leaves them out (mqtt_opts.h) */
#ifndef MQTT_REQ_MAX_IN_FLIGHT
#define MQTT_REQ_MAX_IN_FLIGHT    4
#endif
#ifndef MQTT_OUTPUT_RINGBUF_SIZE
#define MQTT_OUTPUT_RINGBUF_SIZE  256
#endif

#define STALL_S          (4u)   /* MQTT_UTK_KEEPALIVE_S in mqtt_utk.c */
#define PKT_FIXED_B      (1u + 1u + 2u + 2u)  /* type, length, topic len, id */

int main(void)
{
    HOST_INIT();

    uint32_t const per_s  = (1000u / TELEM_STATE_PERIOD_MS)
                            + (1000u / (TELEM_STATE_PERIOD_MS
                                        * TELEM_HB_EVERY_N_STATE));
    uint32_t const msgs   = per_s * STALL_S;
    uint32_t const pkt_b  = PKT_FIXED_B
                            + (uint32_t) strlen(TELEM_TOPIC_STATE)
                            + APP_MQ_PAYLOAD_MAX;

    (void) printf("[INFO] %u msg/s, %u msgs in %u s, %u B per packet, "
                  "%u B needed; slots %d, ring %d B\n",
                  (unsigned) per_s, (unsigned) msgs, (unsigned) STALL_S,
                  (unsigned) pkt_b, (unsigned) (msgs * pkt_b),
                  (int) MQTT_REQ_MAX_IN_FLIGHT,
                  (int) MQTT_OUTPUT_RINGBUF_SIZE);

    HOST_CHECK("S1-U38", (uint32_t) MQTT_REQ_MAX_IN_FLIGHT >= msgs,
               "request slots hold 4 s of state + heartbeat");
    HOST_CHECK("S1-U38", (uint32_t) MQTT_OUTPUT_RINGBUF_SIZE
                         >= (msgs * pkt_b),
               "output ring holds 4 s of state + heartbeat packets");

    return (0 == HOST_SUMMARY("lwip_sizing")) ? 0 : 1;
}
