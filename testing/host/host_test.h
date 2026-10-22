/*
 *  host_test.h
 *  Laptop-side test helpers for the pure (kind "algo") modules.
 *  Host-only and outside BARR-C scope (docs/BARR_C_DEVIATIONS.md, D7).
 *
 *  Prints exactly the same line format as app_program/test_report.h, so a
 *  host log and a target log can be searched the same way:
 *
 *      [TEST] S2-U01 PASS  counts to mm rounds to nearest
 *      [TEST] S2-U02 FAIL  heading sign   (test_odometry_host.c:41)
 *      [TEST] odometry SUMMARY 1 passed, 1 failed
 *
 *  Usage (one test program per pure module):
 *
 *      #include "host_test.h"
 *      int main(void)
 *      {
 *          HOST_INIT();
 *          HOST_CHECK("S2-U01", 0u == odometry_counts_to_mm(0u),
 *                     "zero counts is zero mm");
 *          return (0 == HOST_SUMMARY("odometry")) ? 0 : 1;
 *      }
 */

#ifndef HOST_TEST_H
#define HOST_TEST_H

#include <stdio.h>

#define HOST_INIT()  int host_pass_ = 0; int host_fail_ = 0

#define HOST_CHECK(id, cond, desc)                                      \
    do                                                                  \
    {                                                                   \
        if (cond)                                                       \
        {                                                               \
            host_pass_++;                                               \
            (void) printf("[TEST] %s PASS  %s\n", (id), (desc));        \
        }                                                               \
        else                                                            \
        {                                                               \
            host_fail_++;                                               \
            (void) printf("[TEST] %s FAIL  %s   (%s:%d)\n",             \
                          (id), (desc), __FILE__, __LINE__);            \
        }                                                               \
    } while (0)

/* Prints the summary line; the expression's value is the fail count. */
#define HOST_SUMMARY(name)                                              \
    ((void) printf("[TEST] %s SUMMARY %d passed, %d failed\n",          \
                   (name), host_pass_, host_fail_),                     \
     host_fail_)

#endif /* HOST_TEST_H */
