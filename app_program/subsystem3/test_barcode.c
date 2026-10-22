/**
 * @file    test_barcode.c
 * @brief   On-target tests of barcode (TEST=1 builds only). Phase 0 skeleton:
 *          no checks yet.
 * @owner   Buddy 3 (line and barcode)
 * @kind    test
 * @reqs    Test IDs as listed in the owner's section
 */

#include "test_barcode.h"      /* own header first (BARR-C 4.3.c) */

#if APP_TEST
#include "../test_report.h"
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * test_barcode_run
 * @brief   Runs every on-target test of barcode and prints the
 *          [TEST] summary line. Empty unless built with TEST=1.
 * @return  none
 * @context usermain (at APP_PRI_TEST)
 */
void test_barcode_run(void)
{
#if APP_TEST
    TEST_INIT();
    TEST_SUMMARY("barcode");
#endif
}

/*** end of file ***/
