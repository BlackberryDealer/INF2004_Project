/**
 * @file    odometry.c
 * @brief   Phase 0 STUB of the odometry conversions: every function returns 0.
 *          Pure, host-safe.
 * @owner   Buddy 2 (motion)
 * @kind    algo
 * @reqs    NFR3 (implemented in Phase 2)
 */

#include "odometry.h"          /* own header first (BARR-C 4.3.c) */

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * odometry_counts_to_mm
 * @brief   STUB: encoder counts to millimetres.
 * @param   counts  encoder counts
 * @return  0 (stub)
 * @context any
 */
uint32_t odometry_counts_to_mm(uint32_t counts)
{
    (void) counts;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * odometry_mm_to_counts
 * @brief   STUB: millimetres to encoder counts.
 * @param   distance_mm  distance in mm
 * @return  0 (stub)
 * @context any
 */
uint32_t odometry_mm_to_counts(uint32_t distance_mm)
{
    (void) distance_mm;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * odometry_cpw_to_mm_s
 * @brief   STUB: window speed to mm/s.
 * @param   speed_cpw  counts per 50 ms window
 * @return  0 (stub)
 * @context any
 */
int32_t odometry_cpw_to_mm_s(int32_t speed_cpw)
{
    (void) speed_cpw;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * odometry_mm_s_to_cpw
 * @brief   STUB: mm/s to counts per 50 ms window.
 * @param   speed_mm_s  mm/s
 * @return  0 (stub)
 * @context any
 */
int32_t odometry_mm_s_to_cpw(int32_t speed_mm_s)
{
    (void) speed_mm_s;   /* stub: unused until the owner's version */

    return 0;
}

/**
 * odometry_turn_counts
 * @brief   STUB: per-wheel counts for an on-the-spot turn.
 * @param   angle_deg  degrees
 * @return  0 (stub)
 * @context any
 */
uint32_t odometry_turn_counts(uint32_t angle_deg)
{
    (void) angle_deg;   /* stub: unused until the owner's version */

    return 0u;
}

/**
 * odometry_heading_cdeg
 * @brief   STUB: heading from wheel positions (+ = right).
 * @param   pos_l_counts  left wheel position
 * @param   pos_r_counts  right wheel position
 * @return  0 (stub)
 * @context any
 */
int32_t odometry_heading_cdeg(int32_t pos_l_counts, int32_t pos_r_counts)
{
    (void) pos_l_counts;   /* stub: unused until the owner's version */
    (void) pos_r_counts;

    return 0;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
