/* Host stand-in for tm/tmonitor.h (see lwip/apps/mqtt.h). */
#ifndef MOCK_TMONITOR_H
#define MOCK_TMONITOR_H
typedef unsigned char UB;
int tm_printf(UB * format, ...);
#endif
