/* Host stand-in for lwip/timeouts.h (see lwip/apps/mqtt.h). */
#ifndef MOCK_LWIP_TIMEOUTS_H
#define MOCK_LWIP_TIMEOUTS_H
#include <stdint.h>
typedef void (*sys_timeout_handler)(void * arg);
void sys_timeout(uint32_t msecs, sys_timeout_handler handler, void * arg);
void sys_untimeout(sys_timeout_handler handler, void * arg);
#endif
