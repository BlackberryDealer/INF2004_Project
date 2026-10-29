/**
 * @file    test_drv_ultrasonic_host.c
 * @brief   S5-U09 (review 2026-10-26): the HC-SR04P trigger pulse is at
 *          least 10 us long even when the task is interrupted or
 *          pre-empted at any point, and across the 32-bit timer wrap.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    test
 * @reqs    FR6, A8
 *
 * drv_ultrasonic.c is compiled against mock_us/: gpio_put() records the
 * simulated time of each TRIG edge, and every time_us_32() call advances
 * the simulated 1 MHz timer by 1 us. An "interruption" adds a further
 * delay right after a chosen timer read, as an ISR or a higher-priority
 * task on the same core would. The pulse is the time from TRIG high to
 * TRIG low.
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include "../../../app_program/board_pins.h"
#include "../../../app_program/subsystem5/drv_ultrasonic.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define SIM_NO_STALL    (0xFFFFFFFFu)
#define SIM_STALL_US    (40u)          /* a long ISR or pre-emption */
#define SIM_MIN_US      (10u)          /* HC-SR04P datasheet minimum */
#define SIM_MAX_US      (20u)          /* 20 us busy-wait rule, no stall */

static uint32_t g_sim_us        = 0u;
static uint32_t g_sim_reads     = 0u;
static uint32_t g_sim_stall_at  = SIM_NO_STALL;  /* after this read */
static uint32_t g_sim_high_us   = 0u;
static uint32_t g_sim_low_us    = 0u;
static uint32_t g_sim_highs     = 0u;
static uint32_t g_sim_lows      = 0u;

void gpio_init(uint32_t pin) { (void) pin; }
void gpio_set_dir(uint32_t pin, bool b_out) { (void) pin; (void) b_out; }

void gpio_put(uint32_t pin, bool b_value)
{
    if (PIN_US_TRIG == pin)
    {
        if (b_value)
        {
            g_sim_high_us = g_sim_us;
            g_sim_highs++;
        }
        else
        {
            g_sim_low_us = g_sim_us;
            g_sim_lows++;
        }
    }
}

uint32_t time_us_32(void)
{
    uint32_t const now = g_sim_us;

    g_sim_us += 1u;                    /* each read takes time */
    if (g_sim_reads == g_sim_stall_at)
    {
        g_sim_us += SIM_STALL_US;      /* interrupted after this read */
    }
    g_sim_reads++;
    return now;
}

/* One trigger from start_us with a stall after read stall_at; returns
   the pulse length in us (wrap-safe) and checks there was one pulse. */
static uint32_t sim_pulse(uint32_t start_us, uint32_t stall_at,
                          bool * pb_one_pulse)
{
    g_sim_us       = start_us;
    g_sim_reads    = 0u;
    g_sim_stall_at = stall_at;
    g_sim_highs    = 0u;
    g_sim_lows     = 0u;
    drv_ultrasonic_trigger();
    *pb_one_pulse = (1u == g_sim_highs) && (1u == g_sim_lows);
    return g_sim_low_us - g_sim_high_us;
}

int main(void)
{
    uint32_t pulse   = 0u;
    uint32_t k       = 0u;
    uint32_t min_us  = 0xFFFFFFFFu;
    bool     b_one   = false;
    bool     b_all   = true;

    HOST_INIT();

    drv_ultrasonic_init();
    pulse = sim_pulse(1000u, SIM_NO_STALL, &b_one);
    HOST_CHECK("S5-U09", b_one && (pulse >= SIM_MIN_US)
               && (pulse <= SIM_MAX_US),
               "uninterrupted pulse is 10..20 us");

    /* An interruption after every possible timer read, including the
       first one (the window between the start read and TRIG high in the
       original order).                                                 */
    for (k = 0u; k < 16u; k++)
    {
        pulse = sim_pulse(5000u, k, &b_one);
        b_all = b_all && b_one;
        min_us = (pulse < min_us) ? pulse : min_us;
    }
    HOST_CHECK("S5-U09", b_all && (min_us >= SIM_MIN_US),
               "pulse stays >= 10 us with a 40 us interruption anywhere");

    pulse = sim_pulse(0u, 0u, &b_one);
    HOST_CHECK("S5-U09", b_one && (pulse >= SIM_MIN_US),
               "interruption right after the first timer read: >= 10 us");

    /* Timer wrap: start a few us before 2^32. */
    pulse = sim_pulse(0xFFFFFFFAu, SIM_NO_STALL, &b_one);
    HOST_CHECK("S5-U09", b_one && (pulse >= SIM_MIN_US)
               && (pulse <= SIM_MAX_US),
               "pulse across the 32-bit timer wrap is 10..20 us");
    pulse = sim_pulse(0xFFFFFFFAu, 0u, &b_one);
    HOST_CHECK("S5-U09", b_one && (pulse >= SIM_MIN_US),
               "interrupted pulse across the timer wrap: >= 10 us");

    return (0 == HOST_SUMMARY("drv_ultrasonic")) ? 0 : 1;
}

/*** end of file ***/
