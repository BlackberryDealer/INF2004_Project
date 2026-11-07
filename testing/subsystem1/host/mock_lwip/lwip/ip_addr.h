/* Host stand-in for lwip/ip_addr.h (see lwip/apps/mqtt.h). */
#ifndef MOCK_LWIP_IP_ADDR_H
#define MOCK_LWIP_IP_ADDR_H
typedef struct { unsigned addr; int type; } ip_addr_t;
#define IPADDR_TYPE_V4 0
#define ip_2_ip4(p) (p)
#define IP_SET_TYPE(p, t) ((p)->type = (t))
int ip4addr_aton(const char * cp, ip_addr_t * addr);
/* 2026-11-06 broker beacon: address compare/copy/print, as in lwIP */
#define IP_IS_V4(p) ((p)->type == IPADDR_TYPE_V4)
#define ip_addr_cmp(a, b) ((a)->addr == (b)->addr)
#define ip_addr_copy(dst, src) ((dst) = (src))
extern const ip_addr_t mock_ip_any;
#define IP4_ADDR_ANY (&mock_ip_any)
char * ipaddr_ntoa(const ip_addr_t * addr);
#endif
