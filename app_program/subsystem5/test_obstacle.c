/**
 * @file    test_obstacle.c
 * @brief   On-target tests of obstacle (TEST=1 builds only). Phase 0 skeleton:
 *          no checks yet.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    test
 * @reqs    Test IDs as listed in the owner's section
 */

#include "test_obstacle.h"     /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_obstacle_run
 * @brief   Runs every on-target test of obstacle and prints the
 *          [TEST] summary line. Empty unless built with TEST=1.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_obstacle_run(void)
{
#if APP_TEST
    TEST_INIT();
    TEST_SUMMARY("obstacle");
#endif
}

/*** end of file ***/
