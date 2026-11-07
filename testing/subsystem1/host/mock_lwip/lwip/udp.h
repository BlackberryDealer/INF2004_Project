/* Host stand-in for lwIP's lwip/udp.h: only what the broker beacon in
   mqtt_utk.c uses. The test keeps the receive callback and calls it.
   Host-only, outside BARR-C scope (D7).                                */
#ifndef MOCK_LWIP_UDP_H
#define MOCK_LWIP_UDP_H
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
struct udp_pcb { int dummy; };
typedef void (*udp_recv_fn)(void * arg, struct udp_pcb * pcb, struct pbuf * p,
                            const ip_addr_t * addr, u16_t port);
struct udp_pcb * udp_new_ip_type(u8_t type);
err_t udp_bind(struct udp_pcb * pcb, const ip_addr_t * ipaddr, u16_t port);
void  udp_recv(struct udp_pcb * pcb, udp_recv_fn recv, void * recv_arg);
void  udp_remove(struct udp_pcb * pcb);
#endif
