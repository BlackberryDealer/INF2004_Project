/*
 *  mock/tm/tmonitor.h
 *  Minimal T-Monitor console stub for host execution of on-target test_*.c.
 */

#ifndef MOCK_TMONITOR_H
#define MOCK_TMONITOR_H

#include <tk/tkernel.h>

typedef uint8_t UB;

extern INT tm_printf(UB const * format, ...);

#endif /* MOCK_TMONITOR_H */
