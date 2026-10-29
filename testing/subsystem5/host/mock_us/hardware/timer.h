/* Host stand-in for the SDK's hardware/timer.h (time_us_32 only). */
#ifndef MOCK_US_HARDWARE_TIMER_H
#define MOCK_US_HARDWARE_TIMER_H
#include <stdint.h>
uint32_t time_us_32(void);
#endif
