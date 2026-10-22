/**
 * @file    test_speed_pid_host.c
 * @brief   Laptop test of speed_pid.c (S2-U02): a set-point step on a
 *          first-order wheel model.
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    NFR2
 *
 * Model (spec B2.2.5): speed += (duty / 14 - speed) / 4 each 10 ms tick,
 * i.e. about 14 permille of duty per cpw of steady speed. Kept in 1/16 cpw
 * so integer truncation does not freeze the model short of the target.
 * Pass when: duty always within +-1000, the integrator never beyond
 * +-MOTION_I_CLAMP, every duty change <= the ramp, and the speed is within
 * 5 % of the set-point from tick 50 to the end (settled within 50 ticks).
 */

#include "host_test.h"
#include "speed_pid.h"
#include "cfg_motion.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ---- private constants and types ------------------------------------ */

#define TEST_TICKS        (150)
#define TEST_SETTLE_TICK  (50)
#define TEST_MODEL_SCALE  (16)    /* model speed kept in 1/16 cpw */
#define TEST_MODEL_GAIN   (14)    /* permille per cpw             */
#define TEST_MODEL_TAU    (4)

/* ---- private function prototypes ------------------------------------- */

static bool test_step(int32_t setpoint_cpw);
static int32_t test_abs(int32_t value);

/* ---- public functions ------------------------------------------------- */

/**
 * main
 * @brief   Runs S2-U02 for cruise, slow, reverse and stop set-points.
 * @return  0 if every check passed, 1 otherwise
 * @context host
 */
int main(void)
{
    HOST_INIT();

    HOST_CHECK("S2-U02", test_step(36), "step to 36 cpw (250 mm/s)");
    HOST_CHECK("S2-U02", test_step(21), "step to 21 cpw (150 mm/s)");
    HOST_CHECK("S2-U02", test_step(-14), "step to -14 cpw (reverse)");
    HOST_CHECK("S2-U02", test_step(43), "step to 43 cpw (300 mm/s)");

    return (0 == HOST_SUMMARY("speed_pid")) ? 0 : 1;
}

/* ---- private functions ------------------------------------------------ */

/**
 * test_step
 * @brief   Simulates TEST_TICKS ticks from rest, then a step back to 0.
 * @param   setpoint_cpw  set-point, counts per 50 ms
 * @return  true if every property holds
 * @context host
 */
static bool test_step(int32_t setpoint_cpw)
{
    speed_pid_t pid      = {0};
    int32_t     duty     = 0;
    int32_t     speed16  = 0;    /* model speed x 16 */
    int32_t     tick     = 0;
    bool        b_is_ok  = true;
    int32_t     settled_band = test_abs(setpoint_cpw) * TEST_MODEL_SCALE;

    settled_band = (settled_band * 5) / 100;   /* 5 % of the set-point */

    for (tick = 0; tick < (TEST_TICKS * 2); tick++)
    {
        int32_t const sp   = (tick < TEST_TICKS) ? setpoint_cpw : 0;
        int32_t const prev = duty;

        duty = speed_pid_step(&pid, sp, speed16 / TEST_MODEL_SCALE, duty);

        b_is_ok = b_is_ok && (test_abs(duty) <= MOTION_DUTY_MAX_PERMILLE);
        b_is_ok = b_is_ok && (test_abs(pid.integ) <= MOTION_I_CLAMP);
        b_is_ok = b_is_ok && (test_abs(duty - prev)
                              <= MOTION_RAMP_PERMILLE_PER_TICK);

        speed16 += (((duty * TEST_MODEL_SCALE) / TEST_MODEL_GAIN) - speed16)
                   / TEST_MODEL_TAU;

        if ((tick >= TEST_SETTLE_TICK) && (tick < TEST_TICKS))
        {
            b_is_ok = b_is_ok
                      && (test_abs(speed16 - (setpoint_cpw * TEST_MODEL_SCALE))
                          <= settled_band);
        }
    }

    b_is_ok = b_is_ok && (0 == duty) && (0 == pid.integ);
    (void) printf("  sp %ld: final speed x16 %ld after the stop step\n",
                  (long) setpoint_cpw, (long) speed16);

    return b_is_ok;
}

/**
 * test_abs
 * @brief   Absolute value (inputs are far from INT32_MIN).
 * @param   value  any value with |value| < 2^31
 * @return  |value|
 * @context host
 */
static int32_t test_abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*** end of file ***/
