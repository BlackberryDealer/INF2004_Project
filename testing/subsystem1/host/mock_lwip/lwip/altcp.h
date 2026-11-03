/* Host stand-in for lwIP's lwip/altcp.h: only the Nagle switch that
   mqtt_utk.c uses. The test records the call. Host-only, D7.           */
#ifndef MOCK_LWIP_ALTCP_H
#define MOCK_LWIP_ALTCP_H
struct altcp_pcb;
void altcp_nagle_disable(struct altcp_pcb * conn);
#endif
