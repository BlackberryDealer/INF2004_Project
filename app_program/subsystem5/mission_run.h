/**
 * @file    mission_run.h
 * @brief   RUN superstate (Figure 5b) and the shared mission context.
 *          Internal to mission.c, mission_run.c and test_mission.c.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef MISSION_RUN_H
#define MISSION_RUN_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "status_board.h"

#define MISSION_REQ_NONE   (0xFFu)   /* sys_request: stay in RUN */

/* One event taken from K4 (or made from a button press) */
typedef struct
{
    bool     b_has_event;
    uint32_t type;           /* APP_EVT_*            */
    uint32_t value;          /* event value (D.13)   */
} mission_evt_in_t;

/* Whole mission state (baseline: natural types; Phase 4 packs it into
   the PRV_MIS_* words). Only mission_task touches it.                */
typedef struct
{
    uint32_t sys_state;          /* APP_SYS_*  */
    uint32_t run_state;          /* APP_RUN_*  */
    uint32_t avoid_step;         /* APP_AVOID_* */
    uint32_t substep;            /* step inside a sweep, turn or rejoin */
    uint32_t end_reason;         /* APP_END_*  */
    bool     b_is_armed;
    uint32_t armed_cmd;          /* APP_BAR_CMD_* */
    uint32_t no_path_cnt;        /* NO_PATH answers for this obstacle */
    uint32_t last_faults;        /* fault mask seen last pass */
    uint32_t sys_request;        /* set by RUN handlers: APP_SYS_* */

    uint32_t arm_start;          /* path counts when armed */
    uint32_t path_at_stop;       /* path counts when the bypass started */
    int32_t  diff_at_stop;       /* pos_l - pos_r when the bypass started */
    uint32_t run_start_path;     /* path counts at START */
    uint32_t rejoin_start;       /* path counts at the rejoin move */
    uint32_t inhibit_mark;       /* path counts to clear the inhibits */

    uint32_t side;               /* planned bypass side, APP_SIDE_* */
    uint32_t leg_lat_mm;
    uint32_t leg_fwd_mm;
    uint32_t rejoin_e_mm;        /* expected rejoin distance */

    uint32_t cal_start_us;       /* E5 timers (sys_tick_now_us) */
    uint32_t sweep_start_us;
    uint32_t warn_start_us;
    uint32_t search_start_us;
    uint32_t fr9_start_us;
    uint32_t resume_start_us;
    uint32_t run_start_us;

    bool     b_is_warning;       /* 1 s start warning running */
    bool     b_resume_pending;   /* RESUME set, clear it 150 ms later */
    bool     b_inhibit_pending;  /* clear inhibits at inhibit_mark */
    bool     b_fr9_running;
    bool     b_wait_standstill;  /* AVOID entry: wait for 0 speed */
    bool     b_reset_pending;    /* FAULT: RESET seen, check next pass */
    bool     b_has_run;          /* run start values are valid */
    bool     b_post_avoid_slow;  /* FOLLOW at SLOW until inhibits clear */
    bool     b_is_quiet;         /* test matrix: no logging */
} mission_ctx_t;

/* Enters RUN at FOLLOW. */
extern void mission_run_enter(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap,
                              uint32_t now_us);

/* One RUN pass (with or without an event). May set p_ctx->sys_request
   (FINISHED, LINE_LOST_STOP or FAULT) with end_reason.               */
extern void mission_run_handle(mission_ctx_t * p_ctx,
                               mission_evt_in_t const * p_evt,
                               status_board_t const * p_snap,
                               uint32_t now_us);

/* Starts a RESUME pulse (FLG_MODE RESUME set now, cleared by mission.c
   MISSION_RESUME_PULSE_MS later).                                    */
extern void mission_run_resume_pulse(mission_ctx_t * p_ctx, uint32_t now_us);

#if APP_TEST
/* S5-U05: every state x every event type on a scratch context.
   Returns the number of combinations with an undefined outcome.     */
extern uint32_t mission_test_matrix(uint32_t * p_cases);
#endif

#endif /* MISSION_RUN_H */

/*** end of file ***/
