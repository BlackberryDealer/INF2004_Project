/**
 * @file    test_line.c
 * @brief   On-target tests of line (TEST=1 builds only). Phase 0 skeleton: no
 *          checks yet.
 * @owner   Buddy 3 (line and barcode)
 * @kind    test
 * @reqs    Test IDs as listed in the owner's section
 */

#include "test_line.h"         /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_line_run
 * @brief   Runs every on-target test of line and prints the
 *          [TEST] summary line. Empty unless built with TEST=1.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_line_run(void)
{
#if APP_TEST
    TEST_INIT();
    TEST_SUMMARY("line");
#endif
}

/*** end of file ***/
