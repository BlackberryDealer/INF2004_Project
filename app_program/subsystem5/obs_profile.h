/**
 * @file    obs_profile.h
 * @brief   Scan table -> obstacle profile -> bypass plan (pure C).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    algo
 */

#ifndef OBS_PROFILE_H
#define OBS_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#define OBS_OUT_CONTINUE  (0u)
#define OBS_OUT_PLAN      (1u)
#define OBS_OUT_NO_PATH   (2u)

typedef struct
{
    bool     b_is_valid;
    uint32_t angle_step;     /* step of the nearest point */
    uint32_t nearest_mm;     /* forward distance of the nearest point */
    int32_t  x_left_mm;      /* lateral extent, + = right of centre */
    int32_t  x_right_mm;
    uint32_t width_mm;
    uint32_t clear_left_mm;  /* free lateral gap left of the obstacle,
                                1020 = clear */
    uint32_t clear_right_mm;
} obs_profile_t;

typedef struct
{
    uint32_t outcome;        /* OBS_OUT_* */
    uint32_t side;           /* APP_SIDE_* (valid for PLAN) */
    uint32_t leg_lateral_mm;
    uint32_t leg_forward_mm;
    bool     b_was_clamped;  /* a leg exceeded its NFR16 limit */
} obs_plan_t;

/* p_scan_mm: 25 distances, step n = 30 + 5n deg; APP_DIST_NONE_MM or
   >= APP_PROFILE_CLEAR_MM means clear.                                */
extern void obs_profile_build(uint16_t const * p_scan_mm,
                              obs_profile_t * p_prof);
extern void obs_profile_plan(obs_profile_t const * p_prof,
                             obs_plan_t * p_plan);

#endif /* OBS_PROFILE_H */

/*** end of file ***/
