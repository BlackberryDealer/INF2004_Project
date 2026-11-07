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

    /* state, the debug SENSORS payload with every state (2026-11-06),
       heartbeat; console lines wait while the ring is > 1/4 full */
    uint32_t const per_s  = (1000u / TELEM_STATE_PERIOD_MS)
                            + ((0u != TELEM_DEBUG_MQTT)
                               ? (1000u / TELEM_STATE_PERIOD_MS) : 0u)
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
               "request slots hold 4 s of state + sensors + heartbeat");
    HOST_CHECK("S1-U38", (uint32_t) MQTT_OUTPUT_RINGBUF_SIZE
                         >= (msgs * pkt_b),
               "output ring holds 4 s of state + sensors + heartbeat");

    {
        /* the client is mem_calloc'd from the MEM_SIZE heap: ring, the
           16 B request slots (RP2040 pointers) and ~256 B of other members,
           next to a full TCP send buffer in a stall, plus 1 KB for DHCP,
           ARP and the received frames */
        uint32_t const client_b = (uint32_t) MQTT_OUTPUT_RINGBUF_SIZE
                                  + ((uint32_t) MQTT_REQ_MAX_IN_FLIGHT * 16u)
                                  + 256u;
        uint32_t const need_b   = client_b + (uint32_t) TCP_SND_BUF + 1024u;

        (void) printf("[INFO] lwIP heap: client ~%u B + TCP_SND_BUF %u B + "
                      "1024 B = %u B of MEM_SIZE %u B\n",
                      (unsigned) client_b, (unsigned) TCP_SND_BUF,
                      (unsigned) need_b, (unsigned) MEM_SIZE);
        HOST_CHECK("S1-U38", (uint32_t) MEM_SIZE >= need_b,
                   "lwIP heap holds the MQTT client and a full send buffer");
    }
    return (0 == HOST_SUMMARY("lwip_sizing")) ? 0 : 1;
}
