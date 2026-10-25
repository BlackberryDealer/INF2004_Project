/* Host stand-in for the SDK's hardware/gpio.h, for test_drv_ultrasonic_host
   only: records TRIG edges against the simulated 1 MHz timer.           */
#ifndef MOCK_US_HARDWARE_GPIO_H
#define MOCK_US_HARDWARE_GPIO_H
#include <stdbool.h>
#include <stdint.h>
void gpio_init(uint32_t pin);
void gpio_set_dir(uint32_t pin, bool b_out);
void gpio_put(uint32_t pin, bool b_value);
#endif
