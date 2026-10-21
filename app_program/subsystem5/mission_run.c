/**
 * @file    mission_run.c
 * @brief   RUN superstate (Figure 5b): FOLLOW, ARMED, EXECUTE_TURN,
 *          SEARCH_LINE and the AVOID bypass sequence R1 (FR7-FR9).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR1, FR3, FR7, FR8, FR9, FR14, FR17, NFR15, NFR16
 */

#include "mission_run.h"       /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../subsystem1/telem.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "cfg_mission.h"
#include "mission.h"

/* ---- private constants and types ------------------------------------ */

#define RUN_TURN_RIGHT_ANGLE_DEG  (90)
#define RUN_UTURN_DEG             (180)
#define RUN_SQRT2_X1000           (1414u)   /* 45 deg leg = lateral x 1.414 */
#define RUN_PCT                   (100u)
#define RUN_CDEG_PER_DEG          (100)
#define RUN_US_PER_MS             (1000u)
#define RUN_TURN_MAX_DEG          (360)
#define RUN_LEG_MIN_MM            (1)
#define RUN_INHIBITS              (APP_MODE_BARCODE_INHIBIT          \
                                   | APP_MODE_JUNCTION_INHIBIT)
#define RUN_BAR_CMD_SHIFT         (6u)      /* event value bits 6-7 */
#define RUN_BAR_CMD_MASK          (0x3u)

/* substeps */
#define RUN_SUB_FIRST     (0u)
#define RUN_SUB_SECOND    (1u)
#define RUN_SUB_THIRD     (2u)
#define RUN_SUB_FOURTH    (3u)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static void run_goto(mission_ctx_t * p_ctx, uint32_t run_state,
                     status_board_t const * p_snap, uint32_t now_us);
static bool run_common(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us);
static void run_follow(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us);
static void run_armed(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us);
static void run_execute(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                        status_board_t const * p_snap, uint32_t now_us);
static void run_search(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us);
static void run_avoid(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us);
static void run_avoid_wait(mission_ctx_t * p_ctx, uint32_t type,
                           status_board_t const * p_snap, uint32_t now_us);
static void run_avoid_legs(mission_ctx_t * p_ctx, uint32_t type);
static void run_avoid_rejoin(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us);
static void run_avoid_heading(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap, uint32_t now_us);
static void run_finish_avoid(mission_ctx_t * p_ctx,
                             status_board_t const * p_snap, uint32_t now_us);
static void run_background(mission_ctx_t * p_ctx,
                           status_board_t const * p_snap, uint32_t now_us);
static void run_follow_or_armed(mission_ctx_t * p_ctx,
                                status_board_t const * p_snap,
                                uint32_t now_us);
static void run_arm(mission_ctx_t * p_ctx, uint32_t value,
                    status_board_t const * p_snap);
static void run_request(mission_ctx_t * p_ctx, uint32_t sys_state,
                        uint32_t end_reason);
static void run_turn(uint32_t side, int32_t angle_deg);
static void run_post_profile(status_board_t const * p_snap);
static void run_back_to_wait(mission_ctx_t * p_ctx);

/* ---- public functions ------------------------------------------------- */

/**
 * mission_run_enter
 * @brief   Enters RUN: disarmed, no bypass in progress, FOLLOW at cruise.
 * @param   p_ctx  mission context
 * @param   p_snap status board snapshot of this pass
 * @param   now_us this pass
 * @context mission_task
 */
void mission_run_enter(mission_ctx_t * p_ctx, status_board_t const * p_snap,
                       uint32_t now_us)
{
    p_ctx->b_is_armed        = false;
    p_ctx->armed_cmd         = APP_BAR_CMD_LEFT;
    p_ctx->avoid_step        = APP_AVOID_WAIT_PLAN;
    p_ctx->no_path_cnt       = 0u;
    p_ctx->b_post_avoid_slow = false;
    p_ctx->b_inhibit_pending = false;
    p_ctx->b_fr9_running     = false;
    p_ctx->b_wait_standstill = false;
    p_ctx->sys_request       = MISSION_REQ_NONE;
    run_goto(p_ctx, APP_RUN_FOLLOW, p_snap, now_us);
}

/**
 * mission_run_handle
 * @brief   One RUN pass: common events first (STOP, OBSTACLE, STALL),
 *          then the current substate, then path- and time-based work.
 * @param   p_ctx  mission context
 * @param   p_evt  this pass's event (b_has_event false if none)
 * @param   p_snap status board snapshot of this pass
 * @param   now_us this pass
 * @context mission_task
 */
void mission_run_handle(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                        status_board_t const * p_snap, uint32_t now_us)
{
    bool b_is_done = run_common(p_ctx, p_evt, p_snap, now_us);

    if (!b_is_done)
    {
        switch (p_ctx->run_state)
        {
            case APP_RUN_FOLLOW:
                run_follow(p_ctx, p_evt, p_snap, now_us);
                break;
            case APP_RUN_ARMED:
                run_armed(p_ctx, p_evt, p_snap, now_us);
                break;
            case APP_RUN_EXECUTE_TURN:
                run_execute(p_ctx, p_evt, p_snap, now_us);
                break;
            case APP_RUN_SEARCH_LINE:
                run_search(p_ctx, p_evt, p_snap, now_us);
                break;
            case APP_RUN_AVOID:
                run_avoid(p_ctx, p_evt, p_snap, now_us);
                break;
            default:
                run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
                break;
        }
    }

    if (MISSION_REQ_NONE == p_ctx->sys_request)
    {
        run_background(p_ctx, p_snap, now_us);
    }
}

/**
 * mission_run_resume_pulse
 * @brief   Sets FLG_MODE RESUME; mission.c clears it 150 ms later so the
 *          60 ms obstacle_task is sure to see the edge.
 * @param   p_ctx  mission context
 * @param   now_us this pass
 * @context mission_task
 */
void mission_run_resume_pulse(mission_ctx_t * p_ctx, uint32_t now_us)
{
    if (!p_ctx->b_resume_pending)
    {
        mission_mode_set(APP_MODE_RESUME);
        p_ctx->b_resume_pending = true;
        p_ctx->resume_start_us  = now_us;
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * run_goto
 * @brief   Changes RUN substate and does its entry action.
 * @param   p_ctx     mission context
 * @param   run_state APP_RUN_*
 * @param   p_snap    snapshot of this pass
 * @param   now_us    this pass
 * @context mission_task
 */
static void run_goto(mission_ctx_t * p_ctx, uint32_t run_state,
                     status_board_t const * p_snap, uint32_t now_us)
{
    p_ctx->run_state = run_state;
    p_ctx->substep   = RUN_SUB_FIRST;

    switch (run_state)
    {
        case APP_RUN_FOLLOW:
            if (p_ctx->b_is_armed || p_ctx->b_post_avoid_slow)
            {
                (void) motion_follow(MOTION_SLOW_MM_S);   /* in range */
            }
            else
            {
                (void) motion_follow(MOTION_CRUISE_MM_S); /* in range */
            }
            break;
        case APP_RUN_ARMED:
            (void) motion_follow(MOTION_SLOW_MM_S);       /* review pt 4 */
            break;
        case APP_RUN_EXECUTE_TURN:
            (void) motion_stop();                         /* always acts */
            (void) motion_move_forward(
                (int32_t) MISSION_JCT_ADVANCE_MM);  /* cast: 60, in range */
            break;
        case APP_RUN_SEARCH_LINE:
            p_ctx->search_start_us = now_us;
            (void) motion_turn_left(MISSION_SEARCH_SWEEP_DEG); /* in range */
            break;
        case APP_RUN_AVOID:
            (void) motion_stop();          /* ESTOP is already set */
            mission_mode_set(RUN_INHIBITS);
            p_ctx->diff_at_stop      = status_board_pos_l(p_snap)
                                       - status_board_pos_r(p_snap);
            p_ctx->path_at_stop      = status_board_path(p_snap);
            p_ctx->avoid_step        = APP_AVOID_WAIT_PLAN;
            p_ctx->no_path_cnt       = 0u;
            p_ctx->b_wait_standstill = true;
            p_ctx->b_inhibit_pending = false;
            break;
        default:
            run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
            break;
    }
}

/**
 * run_common
 * @brief   Events every RUN substate handles the same way.
 * @return  true if the event was consumed here
 * @context mission_task
 */
static bool run_common(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us)
{
    bool b_is_done = false;

    if (p_evt->b_has_event)
    {
        if (APP_EVT_STOP == p_evt->type)
        {
            run_request(p_ctx, APP_SYS_FINISHED, APP_END_STOP);
            b_is_done = true;
        }
        else if (APP_EVT_STALL == p_evt->type)
        {
            run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
            b_is_done = true;
        }
        else if ((APP_EVT_OBSTACLE == p_evt->type)
                 && (APP_RUN_AVOID != p_ctx->run_state))
        {
            run_goto(p_ctx, APP_RUN_AVOID, p_snap, now_us);
            b_is_done = true;
        }
        else
        {
            /* substate-specific */
        }
    }

    return b_is_done;
}

/**
 * run_follow
 * @brief   FOLLOW: line following (never armed here: a barcode moves
 *          to ARMED, see run_follow_or_armed()).
 * @context mission_task
 */
static void run_follow(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us)
{
    if (p_evt->b_has_event)
    {
        switch (p_evt->type)
        {
            case APP_EVT_BARCODE:
                run_arm(p_ctx, p_evt->value, p_snap);
                run_goto(p_ctx, APP_RUN_ARMED, p_snap, now_us);
                break;
            case APP_EVT_LINE_LOST:
                run_goto(p_ctx, APP_RUN_SEARCH_LINE, p_snap, now_us);
                break;
            case APP_EVT_OBSTACLE_AHEAD:
                (void) motion_follow(MOTION_SLOW_MM_S);   /* in range */
                break;
            case APP_EVT_CONTINUE:
                if (!p_ctx->b_post_avoid_slow)
                {
                    (void) motion_follow(MOTION_CRUISE_MM_S); /* in range */
                }
                break;
            default:
                break;                     /* JUNCTION while not armed */
        }
    }
}

/**
 * run_armed
 * @brief   ARMED: a barcode command waits for the next junction (FR3).
 * @context mission_task
 */
static void run_armed(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const expire = odometry_mm_to_counts(MISSION_ARM_EXPIRE_MM);

    if (p_evt->b_has_event)
    {
        switch (p_evt->type)
        {
            case APP_EVT_JUNCTION:
                run_goto(p_ctx, APP_RUN_EXECUTE_TURN, p_snap, now_us);
                break;
            case APP_EVT_BARCODE:
                run_arm(p_ctx, p_evt->value, p_snap);  /* newer replaces */
                break;
            case APP_EVT_LINE_LOST:
                run_goto(p_ctx, APP_RUN_SEARCH_LINE, p_snap, now_us);
                break;
            default:
                break;           /* OBSTACLE_AHEAD/CONTINUE: already slow */
        }
    }

    if ((APP_RUN_ARMED == p_ctx->run_state)
        && ((status_board_path(p_snap) - p_ctx->arm_start) >= expire))
    {
        p_ctx->b_is_armed = false;                 /* command expired */
        run_goto(p_ctx, APP_RUN_FOLLOW, p_snap, now_us);
    }
}

/**
 * run_execute
 * @brief   EXECUTE_TURN: advance the wheels over the junction, then turn
 *          as commanded (STRAIGHT: no turn), then disarm.
 * @context mission_task
 */
static void run_execute(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                        status_board_t const * p_snap, uint32_t now_us)
{
    if (p_evt->b_has_event && (APP_EVT_MOVE_DONE == p_evt->type))
    {
        if (RUN_SUB_FIRST == p_ctx->substep)
        {
            p_ctx->substep = RUN_SUB_SECOND;
            switch (p_ctx->armed_cmd)
            {
                case APP_BAR_CMD_LEFT:
                    (void) motion_turn_left(RUN_TURN_RIGHT_ANGLE_DEG);
                    break;
                case APP_BAR_CMD_RIGHT:
                    (void) motion_turn_right(RUN_TURN_RIGHT_ANGLE_DEG);
                    break;
                case APP_BAR_CMD_UTURN:
                    (void) motion_turn_left(RUN_UTURN_DEG);
                    break;
                default:                   /* STRAIGHT: nothing to turn */
                    p_ctx->b_is_armed = false;
                    run_goto(p_ctx, APP_RUN_FOLLOW, p_snap, now_us);
                    break;
            }
        }
        else
        {
            p_ctx->b_is_armed = false;
            run_goto(p_ctx, APP_RUN_FOLLOW, p_snap, now_us);
        }
    }
}

/**
 * run_search
 * @brief   SEARCH_LINE: sweep left 60, right 120, left 60 until the line
 *          is found; 5 s without it -> LINE_LOST_STOP (FR14).
 * @context mission_task
 */
static void run_search(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const limit_us = MISSION_SEARCH_TIMEOUT_MS * RUN_US_PER_MS;

    if (p_evt->b_has_event && (APP_EVT_LINE_FOUND == p_evt->type))
    {
        (void) motion_stop();                         /* always acts */
        run_follow_or_armed(p_ctx, p_snap, now_us);
    }
    else if (p_evt->b_has_event && (APP_EVT_MOVE_DONE == p_evt->type))
    {
        if (RUN_SUB_FIRST == p_ctx->substep)
        {
            (void) motion_turn_right(2 * MISSION_SEARCH_SWEEP_DEG); /* 120 */
        }
        else if (RUN_SUB_SECOND == p_ctx->substep)
        {
            (void) motion_turn_left(MISSION_SEARCH_SWEEP_DEG);
        }
        else
        {
            /* sweep finished: wait for LINE_FOUND or the timeout */
        }
        if (p_ctx->substep < RUN_SUB_FOURTH)
        {
            p_ctx->substep++;
        }
    }
    else
    {
        /* other events are ignored while searching */
    }

    if ((APP_RUN_SEARCH_LINE == p_ctx->run_state)
        && ((now_us - p_ctx->search_start_us) >= limit_us))
    {
        (void) motion_stop();                         /* always acts */
        run_request(p_ctx, APP_SYS_LINE_LOST_STOP, APP_END_LINE_LOST);
    }
}

/**
 * run_avoid
 * @brief   AVOID: waits for standstill, asks for a scan, then runs the
 *          bypass steps (sequence R1).
 * @context mission_task
 */
static void run_avoid(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const mot   = status_board_motion_state(p_snap);
    uint32_t const fr9us = MISSION_FR9_LIMIT_MS * RUN_US_PER_MS;
    uint32_t const type  = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;

    if (p_ctx->b_wait_standstill
        && ((APP_MOT_ESTOP == mot) || (APP_MOT_BRAKE == mot))
        && (0 == status_board_speed_l(p_snap))
        && (0 == status_board_speed_r(p_snap)))
    {
        p_ctx->b_wait_standstill = false;
        mission_mode_set(APP_MODE_STOPPED | APP_MODE_RESCAN);
    }

    switch (p_ctx->avoid_step)
    {
        case APP_AVOID_WAIT_PLAN:
            run_avoid_wait(p_ctx, type, p_snap, now_us);
            break;
        case APP_AVOID_REVERSE:
            if (APP_EVT_MOVE_DONE == type)
            {
                mission_mode_set(APP_MODE_RESCAN);  /* clear since NO_PATH */
                p_ctx->avoid_step = APP_AVOID_WAIT_PLAN;
            }
            break;
        case APP_AVOID_TURN_OUT:
        case APP_AVOID_LEG_OUT:
        case APP_AVOID_TURN_PARALLEL:
        case APP_AVOID_LEG_PASS:
            run_avoid_legs(p_ctx, type);
            if (APP_AVOID_REJOIN == p_ctx->avoid_step)
            {
                p_ctx->fr9_start_us  = now_us;    /* end of the pass leg */
                p_ctx->b_fr9_running = true;
            }
            break;
        case APP_AVOID_REJOIN:
            run_avoid_rejoin(p_ctx, p_evt, p_snap, now_us);
            break;
        default:
            run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
            break;
    }

    if ((APP_RUN_AVOID == p_ctx->run_state) && p_ctx->b_fr9_running
        && ((now_us - p_ctx->fr9_start_us) > fr9us)
        && (MISSION_REQ_NONE == p_ctx->sys_request))
    {
        fault_set(APP_FAULT_REACQUIRE_FAIL);       /* FR9: > 5 s */
        run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
    }
}

/**
 * run_avoid_wait
 * @brief   WAIT_PLAN: act on the scan result (PLAN, CONTINUE, NO_PATH).
 * @param   type APP_EVT_* of this pass (APP_EVT_COUNT = none)
 * @context mission_task
 */
static void run_avoid_wait(mission_ctx_t * p_ctx, uint32_t type,
                           status_board_t const * p_snap, uint32_t now_us)
{
    switch (type)
    {
        case APP_EVT_PLAN:
            mission_mode_clear(APP_MODE_RESCAN | APP_MODE_STOPPED);
            safety_clear(SAFETY_ESTOP);
            p_ctx->side       = status_board_plan_side(p_snap);
            p_ctx->leg_lat_mm = status_board_leg_lateral(p_snap);
            p_ctx->leg_fwd_mm = status_board_leg_forward(p_snap);
            run_post_profile(p_snap);
            run_turn(p_ctx->side, RUN_TURN_RIGHT_ANGLE_DEG);
            p_ctx->avoid_step = APP_AVOID_TURN_OUT;
            break;
        case APP_EVT_CONTINUE:
            safety_clear(SAFETY_ESTOP);
            mission_mode_clear(APP_MODE_RESCAN | APP_MODE_STOPPED);
            run_post_profile(p_snap);
            mission_mode_clear(RUN_INHIBITS);
            run_follow_or_armed(p_ctx, p_snap, now_us);
            break;
        case APP_EVT_NO_PATH:
            p_ctx->no_path_cnt++;
            if (p_ctx->no_path_cnt >= 2u)
            {
                run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
            }
            else
            {
                mission_mode_clear(APP_MODE_RESCAN);   /* keep STOPPED */
                safety_clear(SAFETY_ESTOP);
                (void) motion_move_backward(MOTION_REVERSE_MAX_MM);
                p_ctx->avoid_step = APP_AVOID_REVERSE;
            }
            break;
        default:
            break;                       /* still scanning */
    }
}

/**
 * run_avoid_legs
 * @brief   TURN_OUT -> LEG_OUT -> TURN_PARALLEL -> LEG_PASS -> REJOIN.
 *          SIDE_TOO_CLOSE on a leg stops and asks for a new scan.
 * @param   type APP_EVT_* of this pass (APP_EVT_COUNT = none)
 * @context mission_task
 */
static void run_avoid_legs(mission_ctx_t * p_ctx, uint32_t type)
{
    uint32_t const other = (APP_SIDE_LEFT == p_ctx->side)
                           ? APP_SIDE_RIGHT : APP_SIDE_LEFT;
    bool const     b_leg = (APP_AVOID_LEG_OUT == p_ctx->avoid_step)
                           || (APP_AVOID_LEG_PASS == p_ctx->avoid_step);

    if ((APP_EVT_SIDE_TOO_CLOSE == type) && b_leg)
    {
        run_back_to_wait(p_ctx);
    }
    else if (APP_EVT_MOVE_DONE == type)
    {
        switch (p_ctx->avoid_step)
        {
            case APP_AVOID_TURN_OUT:
                /* cast: leg <= 1020 mm */
                (void) motion_move_forward((p_ctx->leg_lat_mm > 0u)
                                           ? (int32_t) p_ctx->leg_lat_mm
                                           : RUN_LEG_MIN_MM);
                p_ctx->avoid_step = APP_AVOID_LEG_OUT;
                break;
            case APP_AVOID_LEG_OUT:
                run_turn(other, RUN_TURN_RIGHT_ANGLE_DEG);
                p_ctx->avoid_step = APP_AVOID_TURN_PARALLEL;
                break;
            case APP_AVOID_TURN_PARALLEL:
                /* cast: <= 1020 */
                (void) motion_move_forward((p_ctx->leg_fwd_mm > 0u)
                                           ? (int32_t) p_ctx->leg_fwd_mm
                                           : RUN_LEG_MIN_MM);
                p_ctx->avoid_step = APP_AVOID_LEG_PASS;
                break;
            default:                    /* LEG_PASS done: turn to rejoin */
                run_turn(other, MISSION_REJOIN_DEG);
                p_ctx->avoid_step = APP_AVOID_REJOIN;
                p_ctx->substep    = RUN_SUB_FIRST;
                break;
        }
    }
    else
    {
        /* other events are ignored on the legs */
    }
}

/**
 * run_avoid_rejoin
 * @brief   REJOIN: drive at 45 deg toward the line; accept the first
 *          LINE_FOUND inside the distance window with the expected width,
 *          advance over it, then turn back to the original heading.
 * @context mission_task
 */
static void run_avoid_rejoin(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const lo   = (p_ctx->rejoin_e_mm
                           * (RUN_PCT - MISSION_REJOIN_WIN_PCT)) / RUN_PCT;
    uint32_t const hi   = (p_ctx->rejoin_e_mm
                           * (RUN_PCT + MISSION_REJOIN_WIN_PCT)) / RUN_PCT;
    uint32_t       done = 0u;

    if ((RUN_SUB_FIRST == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        p_ctx->rejoin_e_mm  = (p_ctx->leg_lat_mm * RUN_SQRT2_X1000) / 1000u;
        p_ctx->rejoin_start = status_board_path(p_snap);
        p_ctx->substep      = RUN_SUB_SECOND;
        /* cast: <= 1020 x 1.414 x 1.3 < 2047 */
        (void) motion_move_forward((int32_t) ((p_ctx->rejoin_e_mm
                                               * (RUN_PCT
                                                  + MISSION_REJOIN_WIN_PCT))
                                              / RUN_PCT));
    }
    else if ((RUN_SUB_SECOND == p_ctx->substep)
             && (APP_EVT_LINE_FOUND == type))
    {
        done = odometry_counts_to_mm(status_board_path(p_snap)
                                     - p_ctx->rejoin_start);
        if ((done >= lo) && (done <= hi)
            && (p_evt->value + MISSION_REJOIN_WIDTH_TOL_MM
                >= MISSION_REJOIN_WIDTH_MM)
            && (p_evt->value
                <= (MISSION_REJOIN_WIDTH_MM + MISSION_REJOIN_WIDTH_TOL_MM)))
        {
            (void) motion_stop();                       /* always acts */
            (void) motion_move_forward(
                (int32_t) MISSION_JCT_ADVANCE_MM);  /* cast: 60, in range */
            p_ctx->substep = RUN_SUB_THIRD;
        }
    }
    else if ((RUN_SUB_SECOND == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        fault_set(APP_FAULT_REACQUIRE_FAIL);    /* no line in the window */
        run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
    }
    else if ((RUN_SUB_THIRD == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        run_avoid_heading(p_ctx, p_snap, now_us);
    }
    else if ((RUN_SUB_FOURTH == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        run_finish_avoid(p_ctx, p_snap, now_us);
    }
    else
    {
        /* waiting for the current move */
    }
}

/**
 * run_avoid_heading
 * @brief   Turns back to the heading the robot had when it stopped
 *          (+ error = turned right, so turn left). Skips small errors.
 * @context mission_task
 */
static void run_avoid_heading(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap, uint32_t now_us)
{
    int32_t const diff_now = status_board_pos_l(p_snap)
                             - status_board_pos_r(p_snap);
    int32_t const err_cdeg = odometry_heading_cdeg(diff_now, 0)
                             - odometry_heading_cdeg(p_ctx->diff_at_stop, 0);
    int32_t const mag_cdeg = (err_cdeg < 0) ? -err_cdeg : err_cdeg;
    int32_t       turn_deg = 0;

    if (mag_cdeg <= MISSION_HEADING_TOL_CDEG)
    {
        run_finish_avoid(p_ctx, p_snap, now_us);
    }
    else
    {
        /* event time: division allowed */
        turn_deg = (mag_cdeg + (RUN_CDEG_PER_DEG / 2)) / RUN_CDEG_PER_DEG;
        if (turn_deg > RUN_TURN_MAX_DEG)
        {
            turn_deg = RUN_TURN_MAX_DEG;
        }
        if (err_cdeg > 0)
        {
            (void) motion_turn_left(turn_deg);
        }
        else
        {
            (void) motion_turn_right(turn_deg);
        }
        p_ctx->substep = RUN_SUB_FOURTH;
    }
}

/**
 * run_finish_avoid
 * @brief   Bypass done: FOLLOW at slow speed (or ARMED), RESUME pulse for
 *          obstacle_task, inhibits cleared 150 mm later, armed command kept.
 * @context mission_task
 */
static void run_finish_avoid(mission_ctx_t * p_ctx,
                             status_board_t const * p_snap, uint32_t now_us)
{
    p_ctx->b_post_avoid_slow = true;
    p_ctx->b_inhibit_pending = true;
    p_ctx->inhibit_mark      = status_board_path(p_snap)
                               + odometry_mm_to_counts(
                                   MISSION_INHIBIT_CLEAR_MM);
    p_ctx->avoid_step        = APP_AVOID_WAIT_PLAN;
    mission_run_resume_pulse(p_ctx, now_us);
    run_follow_or_armed(p_ctx, p_snap, now_us);
}

/**
 * run_background
 * @brief   Work every RUN pass outside the substates: stop the FR9
 *          timer at the first following cycle after a bypass (and log
 *          it), clear the inhibits 150 mm later and go back to cruise.
 * @context mission_task
 */
static void run_background(mission_ctx_t * p_ctx,
                           status_board_t const * p_snap, uint32_t now_us)
{
    if (p_ctx->b_fr9_running && (APP_RUN_AVOID != p_ctx->run_state))
    {
        p_ctx->b_fr9_running = false;     /* first FOLLOW cycle (FR9) */
        if (!p_ctx->b_is_quiet)
        {
            /* cast: elapsed ms < 2^31 */
            app_log("FR9 %d ms\n",
                    (int32_t) ((now_us - p_ctx->fr9_start_us)
                               / RUN_US_PER_MS), 0, 0);
        }
    }

    if (p_ctx->b_inhibit_pending
        && (status_board_path(p_snap) >= p_ctx->inhibit_mark))
    {
        p_ctx->b_inhibit_pending = false;
        p_ctx->b_post_avoid_slow = false;
        mission_mode_clear(RUN_INHIBITS);
        if ((APP_RUN_FOLLOW == p_ctx->run_state) && (!p_ctx->b_is_armed))
        {
            (void) motion_follow(MOTION_CRUISE_MM_S);   /* in range */
        }
    }
}

/**
 * run_follow_or_armed
 * @brief   Back to line following: ARMED if a command is stored.
 * @context mission_task
 */
static void run_follow_or_armed(mission_ctx_t * p_ctx,
                                status_board_t const * p_snap,
                                uint32_t now_us)
{
    if (p_ctx->b_is_armed)
    {
        run_goto(p_ctx, APP_RUN_ARMED, p_snap, now_us);
    }
    else
    {
        run_goto(p_ctx, APP_RUN_FOLLOW, p_snap, now_us);
    }
}

/**
 * run_arm
 * @brief   Stores a barcode command and where it was read.
 * @param   value BARCODE event value (char bits 0-5, cmd bits 6-7)
 * @context mission_task
 */
static void run_arm(mission_ctx_t * p_ctx, uint32_t value,
                    status_board_t const * p_snap)
{
    p_ctx->armed_cmd  = (value >> RUN_BAR_CMD_SHIFT) & RUN_BAR_CMD_MASK;
    p_ctx->b_is_armed = true;
    p_ctx->arm_start  = status_board_path(p_snap);
}

/**
 * run_request
 * @brief   Asks the top level to leave RUN.
 * @param   sys_state  APP_SYS_FINISHED, _LINE_LOST_STOP or _FAULT
 * @param   end_reason APP_END_*
 * @context mission_task
 */
static void run_request(mission_ctx_t * p_ctx, uint32_t sys_state,
                        uint32_t end_reason)
{
    p_ctx->sys_request = sys_state;
    p_ctx->end_reason  = end_reason;
}

/**
 * run_turn
 * @brief   Turns toward a side.
 * @param   side      APP_SIDE_LEFT or APP_SIDE_RIGHT
 * @param   angle_deg 1..360
 * @context mission_task
 */
static void run_turn(uint32_t side, int32_t angle_deg)
{
    if (APP_SIDE_LEFT == side)
    {
        (void) motion_turn_left(angle_deg);    /* in range */
    }
    else
    {
        (void) motion_turn_right(angle_deg);   /* in range */
    }
}

/**
 * run_post_profile
 * @brief   Sends the obstacle profile from the board to telemetry.
 * @context mission_task
 */
static void run_post_profile(status_board_t const * p_snap)
{
    telem_bin_profile_t prof = {0u, 0u, 0u, 0u, 0u, 0u};

    prof.angle_step     = status_board_angle_step(p_snap);
    prof.width_mm       = status_board_width(p_snap);
    prof.nearest_mm     = status_board_nearest(p_snap);
    prof.clear_left_mm  = status_board_clear_left(p_snap);
    prof.clear_right_mm = status_board_clear_right(p_snap);
    prof.side           = status_board_plan_side(p_snap);
    (void) telem_post_profile(&prof);   /* a full K7 is counted there */
}

/**
 * run_back_to_wait
 * @brief   SIDE_TOO_CLOSE on a leg: stop and ask for a new scan.
 * @context mission_task
 */
static void run_back_to_wait(mission_ctx_t * p_ctx)
{
    (void) motion_stop();                             /* always acts */
    mission_mode_set(APP_MODE_STOPPED | APP_MODE_RESCAN);
    p_ctx->avoid_step = APP_AVOID_WAIT_PLAN;
}

/*** end of file ***/
