/**
 * @file    test_flags_host.c
 * @brief   Isolated tests of safety.c (K3) and fault.c (K12): S2-U13 and
 *          S2-U14, run on the laptop against mock_kernel.c event flags.
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    FR13, minimum-bit item 5
 *
 * Why host and not target: on the robot these flags are live system
 * state. Injecting ENCODER/STALL there could erase a real fault and, once
 * mission runs, drive it into FAULT, which restoring the bits cannot undo
 * (review finding). Here the real safety.c and fault.c run on their own
 * flag objects; the mock reproduces the kernel's OR/AND semantics.
 */

#include "host_test.h"
#include "app_types.h"
#include "safety.h"
#include "fault.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * main
 * @brief   Runs S2-U13 (fault mask) and S2-U14 (safety flag).
 * @return  0 if every check passed, 1 otherwise
 * @context host
 */
int main(void)
{
    uint32_t const both = APP_FAULT_ENCODER | APP_FAULT_MOTOR_STALL;

    HOST_INIT();

    /* ---- S2-U13: shared fault bitmask (K12) -------------------------- */
    HOST_CHECK("S2-U13", E_OK == fault_init(), "fault_init creates K12");
    HOST_CHECK("S2-U13", 0u == fault_get(), "no fault after init");
    fault_set(both);
    HOST_CHECK("S2-U13", both == fault_get(), "ENCODER and STALL set");
    fault_clear(APP_FAULT_MOTOR_STALL);
    HOST_CHECK("S2-U13", APP_FAULT_ENCODER == fault_get(),
               "clear STALL -> only ENCODER left (inverted-mask clear)");
    fault_set(0xFFFFFFFFu);
    HOST_CHECK("S2-U13", APP_FAULT_ALL == fault_get(),
               "bits outside the 10-bit mask are ignored");
    fault_clear(APP_FAULT_ALL);
    HOST_CHECK("S2-U13", 0u == fault_get(), "clear all -> 0");

    /* ---- S2-U14: safety interlock flag (K3) -------------------------- */
    HOST_CHECK("S2-U14", E_OK == safety_init(), "safety_init creates K3");
    HOST_CHECK("S2-U14", !safety_is_active(), "not active after init");
    safety_set(SAFETY_ESTOP);
    HOST_CHECK("S2-U14", safety_is_active(), "set from a task -> active");
    safety_set_from_isr(SAFETY_IMPACT);
    HOST_CHECK("S2-U14", (SAFETY_ESTOP | SAFETY_IMPACT) == safety_get(),
               "ISR set adds IMPACT, ESTOP kept");
    safety_clear(SAFETY_ESTOP);
    HOST_CHECK("S2-U14", SAFETY_IMPACT == safety_get(),
               "clear ESTOP leaves IMPACT: still active");
    safety_clear(SAFETY_ALL);
    HOST_CHECK("S2-U14", !safety_is_active(),
               "clear -> safety_is_active() false");
    safety_set(0xF8u);
    HOST_CHECK("S2-U14", !safety_is_active(),
               "bits outside SAFETY_ALL are ignored");
    HOST_CHECK("S2-U13", 0u == fault_get(),
               "K3 and K12 are separate objects");

    return (0 == HOST_SUMMARY("flags")) ? 0 : 1;
}

/*** end of file ***/
