/**
 * @file    odometry.h
 * @brief   Pure conversions; any module may include it.
 * @owner   Buddy 2 (motion)
 * @kind    algo
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

extern uint32_t odometry_counts_to_mm(uint32_t counts);
extern uint32_t odometry_mm_to_counts(uint32_t distance_mm);
extern int32_t  odometry_cpw_to_mm_s(int32_t speed_cpw);
extern int32_t  odometry_mm_s_to_cpw(int32_t speed_mm_s);
extern uint32_t odometry_turn_counts(uint32_t angle_deg); /* per wheel  */
extern int32_t  odometry_heading_cdeg(int32_t pos_l_counts,
                                      int32_t pos_r_counts); /* + = right */

#endif /* ODOMETRY_H */

/*** end of file ***/
