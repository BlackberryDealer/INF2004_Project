/**
 * @file    obs_edge.h
 * @brief   1-degree edge pass of the obstacle scan (S5-U83): after the
 *          5-degree points, finds each box corner one degree at a time
 *          and turns the two corners into the reported width. Pure C, no
 *          kernel or servo calls: obstacle.c moves the servo to the angle
 *          each step asks for and stores the state packed (PRV_OBS_E,
 *          PRV_OBS_C).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    algo
 * @reqs    FR7 (obstacle width within +-10 mm, 24 Nov 2026)
 */

#ifndef OBS_EDGE_H
#define OBS_EDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "obs_profile.h"

/* Which side the pass is on (PRV_OBS_E.EDGE_PHASE) */
#define OBS_EDGE_OFF      (0u)
#define OBS_EDGE_LO_SIDE  (1u)     /* low servo angles (robot's right) */
#define OBS_EDGE_HI_SIDE  (2u)

/* Working copy of the pass state; obstacle.c packs it between cycles. */
typedef struct
{
    uint32_t phase;       /* OBS_EDGE_* */
    uint32_t sub;         /* probes taken on this side, 1..SUB_MAX */
    uint32_t lo_deg;      /* outermost in-band angle, low side */
    uint32_t hi_deg;      /* outermost in-band angle, high side */
    uint32_t limit_code;  /* band limit, 4 mm code */
    uint32_t lo_code;     /* corner echo, low side, 4 mm code */
    uint32_t hi_code;     /* corner echo, high side, 4 mm code */
    bool     b_ok_lo;     /* low edge confirmed (background beyond) */
    bool     b_ok_hi;
    bool     b_grace;     /* the probe now taken is the extra degree */
} obs_edge_t;

/* Where the servo must point next. b_is_far: a long swing (the first
   probe of a side), which needs the longer settle time.             */
typedef struct
{
    uint32_t deg;
    bool     b_is_far;
} obs_edge_probe_t;

/* p_scan_codes: the 25 scan points as 4 mm codes (255 = clear).       */
extern bool     obs_edge_begin(obs_edge_t * p_edge,
                               uint8_t const * p_scan_codes,
                               obs_profile_t const * p_prof,
                               obs_edge_probe_t * p_next);
extern bool     obs_edge_sample(obs_edge_t * p_edge, uint32_t code,
                                uint8_t const * p_scan_codes,
                                obs_edge_probe_t * p_next);
extern uint32_t obs_edge_width(obs_edge_t const * p_edge,
                               uint32_t * p_pm_mm);

#endif /* OBS_EDGE_H */

/*** end of file ***/
