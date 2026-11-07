/* Host stand-in for lwIP's lwip/pbuf.h: a single-segment pbuf. D7. */
#ifndef MOCK_LWIP_PBUF_H
#define MOCK_LWIP_PBUF_H
#include "lwip/apps/mqtt.h"
struct pbuf { const void * payload; u16_t len; };
u16_t pbuf_copy_partial(const struct pbuf * p, void * dataptr, u16_t len,
                        u16_t offset);
u8_t  pbuf_free(struct pbuf * p);
#endif
