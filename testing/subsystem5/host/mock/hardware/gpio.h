/* Host-simulation stand-in for the SDK's hardware/gpio.h (gpio_get only). */
#ifndef MOCK_HARDWARE_GPIO_H
#define MOCK_HARDWARE_GPIO_H
#include <stdbool.h>
#include <stdint.h>
bool gpio_get(uint32_t pin);
#endif
