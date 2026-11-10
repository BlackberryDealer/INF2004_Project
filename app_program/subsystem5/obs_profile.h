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
    uint32_t angle_step;     /* step of the shortest echo (S5-U45) */
    uint32_t nearest_mm;     /* forward distance of that echo */
    int32_t  x_left_mm;      /* lateral extent, + = right of centre */
    int32_t  x_right_mm;
    uint32_t width_mm;       /* beam-corrected estimate, kept within
                                OBS_WIDTH_MIN_MM..OBS_WIDTH_MAX_MM */
    uint32_t raw_width_mm;   /* measured span of the whole run (before
                                beam correction; S5-U45 uncertainty) */
    uint32_t clear_left_mm;  /* free lateral gap left of the obstacle,
                                1020 = clear */
    uint32_t clear_right_mm;
    uint32_t block_step_left;   /* step that sets clear_left_mm, or
                                   APP_SCAN_STEPS if clear (S5-U49) */
    uint32_t block_step_right;
    int32_t  plan_left_mm;      /* S5-U53: what the bypass must clear:
                                   estimate - OBS_EST_MARGIN_MM, widened
                                   to every return that may be the box */
    int32_t  plan_right_mm;
    uint32_t ext_step_left;     /* return that widened plan_left_mm, or
                                   APP_SCAN_STEPS (estimate margin)  */
    uint32_t ext_step_right;
    bool     b_extended_left;   /* outer echo projects ahead of the face */
    bool     b_extended_right;
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
   >= APP_PROFILE_CLEAR_MM means clear. inferred: bit n set = step n was
   not measured (filled from a neighbour).                              */
extern void obs_profile_build(uint16_t const * p_scan_mm, uint32_t inferred,
                              obs_profile_t * p_prof);
extern void obs_profile_plan(obs_profile_t const * p_prof,
                             obs_plan_t * p_plan);

#endif /* OBS_PROFILE_H */

/*** end of file ***/
