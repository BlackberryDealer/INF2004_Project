/* Host stand-in for lwip/ip_addr.h (see lwip/apps/mqtt.h). */
#ifndef MOCK_LWIP_IP_ADDR_H
#define MOCK_LWIP_IP_ADDR_H
typedef struct { unsigned addr; int type; } ip_addr_t;
#define IPADDR_TYPE_V4 0
#define ip_2_ip4(p) (p)
#define IP_SET_TYPE(p, t) ((p)->type = (t))
int ip4addr_aton(const char * cp, ip_addr_t * addr);
#endif
