/**
 * @file    test_motion.c
 * @brief   On-target tests of the motion subsystem (TEST=1 builds only).
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    NFR16
 *
 * Runs in usermain at APP_PRI_TEST (21) while the whole system runs, so a
 * test here must never change live system state. Only checks that cannot
 * do so run here:
 *
 *   S2-I07  motion_move_backward(151) is refused (NFR16 reverse cap), and
 *           every other out-of-range argument is refused too: refused
 *           calls send nothing to K5, so the robot cannot move.
 *
 * S2-U13 and S2-U14 (fault and safety flags) are NOT run here: setting
 * ENCODER/STALL or ESTOP on the live flags could erase a real fault or
 * drive mission into FAULT, which restoring the bits cannot undo. They run
 * on isolated flag objects in testing/subsystem2/host/test_flags_host.c.
 * The cycle tests (S2-U03, U12, U15..U19) run in the host simulation
 * test_motion_task_host.c and again on the robot in Phase 3 integration.
 */

#include "test_motion.h"       /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#include "cfg_motion.h"
#include "motion.h"
#endif

/* ---- private function prototypes ------------------------------------- */

#if APP_TEST
static bool test_motion_refused(void);
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_motion_run
 * @brief   Runs the on-target motion tests and prints the [TEST] lines.
 *          Empty unless built with TEST=1.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_motion_run(void)
{
#if APP_TEST
    TEST_INIT();

    /* S2-I07: argument checks; nothing is sent */
    TEST_CHECK("S2-I07", !motion_move_backward(MOTION_REVERSE_MAX_MM + 1),
               "motion_move_backward(151) returns false (NFR16)");
    TEST_CHECK("S2-I07", test_motion_refused(),
               "every out-of-range argument is refused");

    TEST_SUMMARY("motion");
#endif
}

/* ---- private functions ------------------------------------------------ */

#if APP_TEST
/**
 * test_motion_refused
 * @brief   Every command refuses the values just outside its range, so
 *          nothing reaches K5 and the robot never moves during the test.
 * @return  true if all were refused
 * @context usermain (TEST=1)
 */
static bool test_motion_refused(void)
{
    bool b_is_ok = true;

    b_is_ok = b_is_ok && (!motion_move_forward(0));
    b_is_ok = b_is_ok && (!motion_move_forward(MOTION_MOVE_MAX_MM + 1));
    b_is_ok = b_is_ok && (!motion_move_backward(0));
    b_is_ok = b_is_ok && (!motion_move_backward(-1));
    b_is_ok = b_is_ok && (!motion_turn_left(0));
    b_is_ok = b_is_ok && (!motion_turn_left(MOTION_TURN_MAX_DEG + 1));
    b_is_ok = b_is_ok && (!motion_turn_right(0));
    b_is_ok = b_is_ok && (!motion_turn_right(MOTION_TURN_MAX_DEG + 1));
    b_is_ok = b_is_ok && (!motion_follow(-1));
    b_is_ok = b_is_ok && (!motion_follow(MOTION_FOLLOW_MAX_MM_S + 1));

    return b_is_ok;
}
#endif

/*** end of file ***/
