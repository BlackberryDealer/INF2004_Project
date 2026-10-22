/**
 * @file    test_obstacle.c
 * @brief   On-target tests for the obstacle subsystem (none in Phase 0).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    test
 * @reqs    -
 */

#include "test_obstacle.h"     /* own header first (BARR-C 4.3.c) */
#include "../test_report.h"

/* ---- public functions ------------------------------------------------- */

/**
 * test_obstacle_run
 * @brief   Runs the obstacle on-target tests and prints a summary.
 * @context usermain (TEST=1 builds)
 */
void test_obstacle_run(void)
{
#if APP_TEST
    TEST_INIT();
    TEST_SUMMARY("obstacle");
#endif
}

/*** end of file ***/
