/*
 *  mock/hardware/gpio.h
 *  Minimal GPIO stub for host simulation of barcode.c.
 */

#ifndef MOCK_HARDWARE_GPIO_H
#define MOCK_HARDWARE_GPIO_H

#include <stdint.h>
#include <stdbool.h>

extern bool gpio_get(uint32_t gpio);

#endif /* MOCK_HARDWARE_GPIO_H */
