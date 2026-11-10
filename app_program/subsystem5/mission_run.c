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
#include "../subsystem3/barcode.h"
#include "cfg_mission.h"
#include "cfg_obstacle.h"          /* OBS_SCAN_MM, OBS_CREEP_TOL_MM */
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
#define RUN_BAR_CHAR_MASK         (0x3Fu)

#define RUN_BAR_IDLE              (0u)
#define RUN_BAR_BRAKING           (1u)
#define RUN_BAR_REVERSING         (2u)
#define RUN_BAR_SETTLING          (3u)
#define RUN_BAR_READING           (4u)

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
static void run_creep_or_scan(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap,
                              uint32_t now_us);
static bool run_creep_end(mission_ctx_t * p_ctx, uint32_t type,
                          status_board_t const * p_snap);
static void run_avoid_step(mission_ctx_t * p_ctx,
                           mission_evt_in_t const * p_evt, uint32_t type,
                           status_board_t const * p_snap, uint32_t now_us);
static void run_avoid_wait(mission_ctx_t * p_ctx, uint32_t type,
                           status_board_t const * p_snap, uint32_t now_us);
static void run_avoid_legs(mission_ctx_t * p_ctx, uint32_t type,
                           uint32_t now_us);
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
static void run_arm(mission_ctx_t * p_ctx, uint32_t value);
static void run_barcode_begin(mission_ctx_t * p_ctx, uint32_t advance_mm,
                               status_board_t const * p_snap,
                               uint32_t now_us);
static void run_barcode_try_rewind(mission_ctx_t * p_ctx,
                                    status_board_t const * p_snap,
                                    uint32_t now_us);
static void run_barcode_step(mission_ctx_t * p_ctx,
                              mission_evt_in_t const * p_evt,
                              status_board_t const * p_snap,
                              uint32_t now_us);
static bool run_barcode_scan_ok(status_board_t const * p_snap);
static bool run_jct_both(uint32_t jct_class);
static void run_barcode_back(mission_ctx_t * p_ctx, uint32_t now_us);
static void run_request(mission_ctx_t * p_ctx, uint32_t sys_state,
                        uint32_t end_reason);
static void run_turn(mission_ctx_t * p_ctx, uint32_t side, int32_t angle_deg,
                     uint32_t now_us);
static void run_post_profile(mission_ctx_t const * p_ctx,
                             status_board_t const * p_snap);
static void run_log_event(mission_ctx_t const * p_ctx,
                          char const * p_format, uint32_t value);

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
    p_ctx->barcode_phase     = RUN_BAR_IDLE;
    p_ctx->barcode_probe_after = 0u;
    p_ctx->avoid_step        = APP_AVOID_WAIT_PLAN;
    p_ctx->no_path_cnt       = 0u;
    p_ctx->b_post_avoid_slow = false;
    p_ctx->b_inhibit_pending = false;
    p_ctx->b_fr9_running     = false;
    p_ctx->b_wait_standstill = false;
    p_ctx->b_creeping        = false;
    p_ctx->b_crept           = false;
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

    if ((!b_is_done) && (RUN_BAR_IDLE != p_ctx->barcode_phase))
    {
        run_barcode_step(p_ctx, p_evt, p_snap, now_us);
        b_is_done = true;
    }

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
        run_barcode_try_rewind(p_ctx, p_snap, now_us);
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
    p_ctx->run_state          = run_state;
    p_ctx->substep            = RUN_SUB_FIRST;
    p_ctx->barcode_probe_mark = 0u;

    switch (run_state)
    {
        case APP_RUN_FOLLOW:
        case APP_RUN_ARMED:
            p_ctx->barcode_follow_path = status_board_path(p_snap);
            if (p_ctx->b_post_avoid_slow
                || (APP_SCAN_APPROACH == status_board_scanner_state(p_snap)))
            {
                (void) motion_follow(MOTION_SLOW_MM_S);
            }
            else
            {
                (void) motion_follow(MOTION_CRUISE_MM_S);
            }
            break;
        case APP_RUN_EXECUTE_TURN:
            mission_stop_motion(p_ctx);
            /* cast: 60 mm constant */
            (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                     (int32_t) MISSION_JCT_ADVANCE_MM, now_us);
            break;
        case APP_RUN_SEARCH_LINE:
            /* STOP first: motion is still in FOLLOW and would only latch
               the turn (review R06)                                     */
            p_ctx->search_start_us = now_us;
            /* Sweep first toward the side the line was last seen: the
               steering then holds line.c's recovery value (+ = left).
               Always-left-first swung the wrong way half the time
               (on-car 2026-11-03).                                     */
            p_ctx->search_first = (status_board_steering(p_snap) < 0)
                                  ? MISSION_MV_RIGHT : MISSION_MV_LEFT;
            mission_stop_motion(p_ctx);
            (void) mission_send_move(p_ctx, p_ctx->search_first,
                                     MISSION_SEARCH_SWEEP_DEG, now_us);
            break;
        case APP_RUN_AVOID:
            mission_stop_motion(p_ctx);    /* ESTOP is already set */
            mission_mode_set(RUN_INHIBITS);
            mission_mode_clear(APP_MODE_BARCODE_REREAD);
            p_ctx->barcode_phase    = RUN_BAR_IDLE;
            p_ctx->diff_at_stop      = status_board_pos_l(p_snap)
                                       - status_board_pos_r(p_snap);
            p_ctx->path_at_stop      = status_board_path(p_snap);
            p_ctx->avoid_step        = APP_AVOID_WAIT_PLAN;
            p_ctx->no_path_cnt       = 0u;
            p_ctx->b_wait_standstill = true;
            p_ctx->b_creeping        = false;
            p_ctx->b_crept           = false;
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
        if ((APP_EVT_JUNCTION == p_evt->type)
            || (APP_EVT_BARCODE == p_evt->type)
            || (APP_EVT_UNMAPPED == p_evt->type))
        {
            p_ctx->barcode_probe_mark = 0u;
        }
        if ((APP_EVT_JUNCTION == p_evt->type)
            && run_jct_both(p_evt->value))
        {
            p_ctx->barcode_follow_path = status_board_path(p_snap);
        }

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
        else if ((APP_EVT_BARCODE_PROBE == p_evt->type)
                 && (RUN_BAR_IDLE == p_ctx->barcode_phase)
                 && (0u == p_ctx->barcode_probe_mark)
                 && ((APP_RUN_FOLLOW == p_ctx->run_state)
                     || (APP_RUN_ARMED == p_ctx->run_state)))
        {
            run_barcode_begin(p_ctx, p_evt->value, p_snap, now_us);
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
                run_arm(p_ctx, p_evt->value);
                run_goto(p_ctx, APP_RUN_ARMED, p_snap, now_us);
                break;
            case APP_EVT_LINE_LOST:
                run_log_event(p_ctx, "LINE LOST (both sensors white 200 ms):"
                              " searching, path %d mm\n",
                              odometry_counts_to_mm(status_board_path(p_snap)));
                run_goto(p_ctx, APP_RUN_SEARCH_LINE, p_snap, now_us);
                break;
            case APP_EVT_UNMAPPED:
                run_log_event(p_ctx, "BARCODE char index %d decoded but has"
                              " no command: route unchanged\n", p_evt->value);
                break;
            case APP_EVT_INVALID:
                run_log_event(p_ctx, "BARCODE read but not valid Code 39"
                              " (%d): ignored\n", p_evt->value);
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
 * run_cross
 * @brief   A confirmed cross-junction consumes the stored barcode action.
 * @context mission_task
 */
static void run_cross(mission_ctx_t * p_ctx, status_board_t const * p_snap,
                      uint32_t now_us)
{
    if (!p_ctx->b_is_quiet)
    {
        /* casts: Code 39 index < 43, command <= 3 */
        app_log("CROSS: executing barcode char index %d, command %d\n",
                (int32_t) p_ctx->armed_char, (int32_t) p_ctx->armed_cmd, 0);
    }
    run_goto(p_ctx, APP_RUN_EXECUTE_TURN, p_snap, now_us);
}

/**
 * run_armed
 * @brief   ARMED: follow at cruise and wait for a cross-junction (FR3).
 * @context mission_task
 */
static void run_armed(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us)
{
    if (p_evt->b_has_event)
    {
        switch (p_evt->type)
        {
            case APP_EVT_JUNCTION:
                if (run_jct_both(p_evt->value))
                {
                    run_cross(p_ctx, p_snap, now_us);
                }
                break;
            case APP_EVT_BARCODE:
                run_arm(p_ctx, p_evt->value);          /* newer replaces */
                break;
            case APP_EVT_LINE_LOST:
                run_log_event(p_ctx, "LINE LOST while barcode is armed:"
                              " searching, path %d mm\n",
                              odometry_counts_to_mm(status_board_path(p_snap)));
                run_goto(p_ctx, APP_RUN_SEARCH_LINE, p_snap, now_us);
                break;
            case APP_EVT_UNMAPPED:
                run_log_event(p_ctx, "BARCODE char index %d decoded but has"
                              " no command: armed command kept\n",
                              p_evt->value);
                break;
            case APP_EVT_INVALID:
                run_log_event(p_ctx, "BARCODE read but not valid Code 39"
                              " (%d): ignored\n", p_evt->value);
                break;
            case APP_EVT_OBSTACLE_AHEAD:
                (void) motion_follow(MOTION_SLOW_MM_S);
                break;
            case APP_EVT_CONTINUE:
                if (!p_ctx->b_post_avoid_slow)
                {
                    (void) motion_follow(MOTION_CRUISE_MM_S);
                }
                break;
            default:
                break;
        }
    }
}

/**
 * run_barcode_begin
 * @brief   Store a rewind target for a Code 39 start guard.
 *          Keep that target fixed if the line is briefly uncertain.
 * @context mission_task
 */
static void run_barcode_begin(mission_ctx_t * p_ctx, uint32_t advance_mm,
                               status_board_t const * p_snap,
                               uint32_t now_us)
{
    uint32_t const path = status_board_path(p_snap);

    if ((advance_mm <= MISSION_BARCODE_REWIND_MM - MISSION_BARCODE_LEAD_MM)
        && (path >= p_ctx->barcode_probe_after))
    {
        uint32_t const counts = odometry_mm_to_counts(
            advance_mm + MISSION_BARCODE_LEAD_MM);

        if (path >= p_ctx->barcode_follow_path + counts
                    + odometry_mm_to_counts(MISSION_BARCODE_BACK_PAD_MM))
        {
            p_ctx->barcode_probe_mark = path - counts;
            p_ctx->barcode_since_us   = now_us;
            return;
        }
    }

    run_log_event(p_ctx, "BARCODE start at %d mm, no safe rewind\n",
                  advance_mm);
}

/**
 * run_barcode_try_rewind
 * @brief   Wait briefly for visible or centred line tracking before braking.
 *          Cancel the pending retry when the time or distance runs out,
 *          or motion is no longer clear and under the follower's control.
 * @context mission_task
 */
static void run_barcode_try_rewind(mission_ctx_t * p_ctx,
                                    status_board_t const * p_snap,
                                    uint32_t now_us)
{
    if (0u != p_ctx->barcode_probe_mark)
    {
        uint32_t const path = status_board_path(p_snap);
        uint32_t const need = odometry_counts_to_mm(
            path - p_ctx->barcode_probe_mark);
        uint32_t const line = status_board_line_status(p_snap);

        if ((need > MISSION_BARCODE_REWIND_MM - MISSION_BARCODE_BACK_PAD_MM)
            || ((now_us - p_ctx->barcode_since_us)
                >= MISSION_BARCODE_WAIT_MS * RUN_US_PER_MS)
            || (APP_MOT_FOLLOW != status_board_motion_state(p_snap))
            || (status_board_speed_l(p_snap) <= 0)
            || (status_board_speed_r(p_snap) <= 0)
            || (APP_LINE_JUNCTION == line)
            || (!run_barcode_scan_ok(p_snap))
            || mission_mode_is_set(RUN_INHIBITS) || (0u != safety_get()))
        {
            p_ctx->barcode_probe_mark = 0u;
            run_log_event(p_ctx, "BARCODE no safe rewind (%d mm needed)\n",
                          need);
        }
        else if (APP_LINE_ON == line)
        {
            p_ctx->barcode_mark       = p_ctx->barcode_probe_mark;
            p_ctx->barcode_probe_mark = 0u;
            p_ctx->barcode_phase      = RUN_BAR_BRAKING;
            p_ctx->barcode_since_us   = now_us;
            mission_mode_set(RUN_INHIBITS | APP_MODE_BARCODE_REREAD);
            mission_stop_motion(p_ctx);
        }
        else
        {
            /* The line tracker may recover before the deadline. */
        }
    }
}

/**
 * run_barcode_clear
 * @brief   Release the retry modes and suppress a nearby duplicate probe.
 * @context mission_task
 */
static void run_barcode_clear(mission_ctx_t * p_ctx, uint32_t path)
{
    p_ctx->barcode_phase       = RUN_BAR_IDLE;
    p_ctx->barcode_probe_after = path + odometry_mm_to_counts(
                                         MISSION_BARCODE_REARM_MM);
    mission_mode_clear(RUN_INHIBITS | APP_MODE_BARCODE_REREAD);
}

/**
 * run_barcode_still
 * @brief   Both wheels have stopped and the safety flags allow movement.
 * @context mission_task
 */
static bool run_barcode_still(status_board_t const * p_snap)
{
    return ((APP_MOT_BRAKE == status_board_motion_state(p_snap))
            && (0 == status_board_speed_l(p_snap))
            && (0 == status_board_speed_r(p_snap))
            && (0u == safety_get()));
}

/**
 * run_barcode_brake_timeout
 * @brief   A failed stop must not turn into a direction change.
 * @context mission_task
 */
static void run_barcode_brake_timeout(mission_ctx_t * p_ctx, uint32_t now_us)
{
    if ((now_us - p_ctx->barcode_since_us)
        >= MISSION_BARCODE_BRAKE_MS * RUN_US_PER_MS)
    {
        run_log_event(p_ctx, "BARCODE brake timed out (%d ms)\n",
                      MISSION_BARCODE_BRAKE_MS);
        run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
    }
}

/**
 * run_barcode_braked
 * @brief   Set the minimum rewind and bound it by the last follow stretch.
 * @context mission_task
 */
static void run_barcode_braked(mission_ctx_t * p_ctx,
                                status_board_t const * p_snap,
                                uint32_t now_us)
{
    if (run_barcode_still(p_snap))
    {
        uint32_t const path = status_board_path(p_snap);
        uint32_t const need = odometry_counts_to_mm(
            path - p_ctx->barcode_mark);
        uint32_t const follow_mm = odometry_counts_to_mm(
            path - p_ctx->barcode_follow_path);
        uint32_t const available = (follow_mm > MISSION_BARCODE_BACK_PAD_MM)
            ? (follow_mm - MISSION_BARCODE_BACK_PAD_MM) : 0u;
        uint32_t const limit = (available < MISSION_BARCODE_REWIND_MM)
            ? available : MISSION_BARCODE_REWIND_MM;
        uint32_t const back = ((need + MISSION_BARCODE_BACK_EXTRA_MM) < limit)
            ? (need + MISSION_BARCODE_BACK_EXTRA_MM) : limit;

        if ((need <= limit) && run_barcode_scan_ok(p_snap))
        {
            p_ctx->barcode_mark      = path + odometry_mm_to_counts(need);
            p_ctx->barcode_back_path = path;
            p_ctx->barcode_back_left = back;
            p_ctx->barcode_edge_path = path;
            p_ctx->barcode_edges     = barcode_edge_count();
            p_ctx->barcode_phase     = RUN_BAR_REVERSING;
            if (!p_ctx->b_is_quiet)
            {
                /* casts: both distances <= MISSION_BARCODE_REWIND_MM */
                app_log("BARCODE rewind: needs %d mm, backing %d mm\n",
                        (int32_t) need, (int32_t) back, 0);
            }
            run_barcode_back(p_ctx, now_us);
        }
        else
        {
            run_log_event(p_ctx, "BARCODE rewind skipped (%d mm needed)\n",
                          need);
            run_barcode_clear(p_ctx, path);
            run_follow_or_armed(p_ctx, p_snap, now_us);
        }
    }
    else
    {
        run_barcode_brake_timeout(p_ctx, now_us);
    }
}

/**
 * run_jct_both
 * @brief   A junction with branches on both sides (CROSS or T). Centred
 *          on the line, both sensors read white past the stripe, so a
 *          cross is often reported as T (on-car 2026-11-10 22:03).
 * @param   jct_class APP_JCT_*
 * @context mission_task
 */
static bool run_jct_both(uint32_t jct_class)
{
    return ((APP_JCT_CROSS == jct_class) || (APP_JCT_T == jct_class));
}

/**
 * run_barcode_scan_ok
 * @brief   The scanner allows a rewind: no obstacle stop or bypass. An
 *          obstacle ahead (APPROACH) is allowed, the car backs away from
 *          it and the sonar guard still stops the slow pass.
 * @context mission_task
 */
static bool run_barcode_scan_ok(status_board_t const * p_snap)
{
    uint32_t const state = status_board_scanner_state(p_snap);

    return ((APP_SCAN_MONITOR == state) || (APP_SCAN_APPROACH == state));
}

/**
 * run_barcode_back
 * @brief   Send the next reverse move of the rewind. One move is at most
 *          MOTION_REVERSE_MAX_MM, so a long rewind takes several.
 * @context mission_task
 */
static void run_barcode_back(mission_ctx_t * p_ctx, uint32_t now_us)
{
    uint32_t const max_mm = (uint32_t) MOTION_REVERSE_MAX_MM;
    uint32_t const step   = (p_ctx->barcode_back_left < max_mm)
                            ? p_ctx->barcode_back_left : max_mm;

    p_ctx->barcode_back_left -= step;
    /* cast: step <= MOTION_REVERSE_MAX_MM */
    (void) mission_send_move(p_ctx, MISSION_MV_BACK, (int32_t) step, now_us);
}

/**
 * run_barcode_white_lead
 * @brief   Require a quiet white stretch past the first observed bar.
 *          A white gap inside the strip is too short to count.
 * @context mission_task
 */
static bool run_barcode_white_lead(mission_ctx_t * p_ctx, uint32_t path)
{
    uint32_t const edges   = barcode_edge_count();
    bool const     b_white = (0u == barcode_do_level());

    if ((!b_white) || (edges != p_ctx->barcode_edges))
    {
        p_ctx->barcode_edge_path = path;
        p_ctx->barcode_edges     = edges;
    }
    return b_white && (path >= p_ctx->barcode_mark)
           && ((path - p_ctx->barcode_edge_path)
               >= odometry_mm_to_counts(MISSION_BARCODE_QUIET_MM));
}

/**
 * run_barcode_settled
 * @brief   Check that white held through braking, then start the slow pass.
 * @context mission_task
 */
static void run_barcode_settled(mission_ctx_t * p_ctx,
                                 status_board_t const * p_snap,
                                 uint32_t now_us)
{
    uint32_t const path = status_board_path(p_snap);
    bool const     b_lead = run_barcode_white_lead(p_ctx, path);

    if (run_barcode_still(p_snap))
    {
        uint32_t const backed = odometry_counts_to_mm(
            path - p_ctx->barcode_back_path);

        if (!b_lead)
        {
            run_log_event(p_ctx, "BARCODE rewind ended after %d mm"
                          " without a white lead-in\n", backed);
            run_barcode_clear(p_ctx, path);
            run_follow_or_armed(p_ctx, p_snap, now_us);
        }
        else if (motion_follow(MISSION_BARCODE_SPEED_MM_S))
        {
            if (!p_ctx->b_is_quiet)
            {
                /* casts: short rewind and white travel < 2047 mm */
                app_log("BARCODE white lead-in %d mm after rewinding %d mm\n",
                        (int32_t) odometry_counts_to_mm(
                            path - p_ctx->barcode_edge_path),
                        (int32_t) backed, 0);
            }
            p_ctx->barcode_mark     = path;
            p_ctx->barcode_since_us = now_us;
            p_ctx->barcode_phase    = RUN_BAR_READING;
            mission_mode_clear(RUN_INHIBITS);
            run_log_event(p_ctx, "BARCODE reread at %d mm/s\n",
                          MISSION_BARCODE_SPEED_MM_S);
        }
        else
        {
            run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
        }
    }
    else
    {
        run_barcode_brake_timeout(p_ctx, now_us);
    }
}

/**
 * run_barcode_read
 * @brief   End the slow pass on a decoder result, the cross or its limit.
 * @context mission_task
 */
static void run_barcode_read(mission_ctx_t * p_ctx,
                              mission_evt_in_t const * p_evt,
                              status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const path = status_board_path(p_snap);
    bool const     b_cross = (APP_EVT_JUNCTION == type)
                             && run_jct_both(p_evt->value);
    bool           b_finished = true;

    switch (type)
    {
        case APP_EVT_BARCODE:
            run_arm(p_ctx, p_evt->value);
            break;
        case APP_EVT_UNMAPPED:
            run_log_event(p_ctx, "BARCODE reread: char index %d has"
                          " no command\n", p_evt->value);
            break;
        case APP_EVT_INVALID:
            run_log_event(p_ctx, "BARCODE reread invalid (%d elements)\n",
                          p_evt->value);
            break;
        default:
            b_finished = b_cross
                || ((path - p_ctx->barcode_mark)
                    >= odometry_mm_to_counts(MISSION_BARCODE_READ_MM))
                || ((now_us - p_ctx->barcode_since_us)
                    >= MISSION_BARCODE_READ_MS * RUN_US_PER_MS);
            if (b_finished)
            {
                run_log_event(p_ctx, "BARCODE reread ended without decode"
                              " after %d mm\n",
                              odometry_counts_to_mm(path
                                                    - p_ctx->barcode_mark));
            }
            break;
    }

    if (b_finished)
    {
        run_barcode_clear(p_ctx, path);
        if (b_cross && p_ctx->b_is_armed)
        {
            run_cross(p_ctx, p_snap, now_us);
        }
        else
        {
            run_follow_or_armed(p_ctx, p_snap, now_us);
        }
    }
}

/**
 * run_barcode_step
 * @brief   Brake, back up once, then follow the line at the reread speed.
 *          Common STOP, obstacle and fault handling has priority.
 * @context mission_task
 */
static void run_barcode_step(mission_ctx_t * p_ctx,
                              mission_evt_in_t const * p_evt,
                              status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const path = status_board_path(p_snap);

    if (APP_EVT_LINE_LOST == type)
    {
        run_barcode_clear(p_ctx, path);
        run_goto(p_ctx, APP_RUN_SEARCH_LINE, p_snap, now_us);
    }
    else if (RUN_BAR_BRAKING == p_ctx->barcode_phase)
    {
        run_barcode_braked(p_ctx, p_snap, now_us);
    }
    else if (RUN_BAR_REVERSING == p_ctx->barcode_phase)
    {
        /* Keep updating the quiet stretch, but use the full bounded
           rewind. An edge during early braking could erase the lead. */
        (void) run_barcode_white_lead(p_ctx, path);
        if ((APP_EVT_MOVE_DONE == type) && (0u != p_ctx->barcode_back_left))
        {
            run_barcode_back(p_ctx, now_us);
        }
        else if (APP_EVT_MOVE_DONE == type)
        {
            p_ctx->barcode_phase    = RUN_BAR_SETTLING;
            p_ctx->barcode_since_us = now_us;
        }
        else
        {
            /* Reverse move still running. */
        }
    }
    else if (RUN_BAR_SETTLING == p_ctx->barcode_phase)
    {
        run_barcode_settled(p_ctx, p_snap, now_us);
    }
    else if (RUN_BAR_READING == p_ctx->barcode_phase)
    {
        run_barcode_read(p_ctx, p_evt, p_snap, now_us);
    }
    else
    {
        /* No active barcode retry phase. */
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
                    (void) mission_send_move(p_ctx, MISSION_MV_LEFT,
                                             RUN_TURN_RIGHT_ANGLE_DEG, now_us);
                    break;
                case APP_BAR_CMD_RIGHT:
                    (void) mission_send_move(p_ctx, MISSION_MV_RIGHT,
                                             RUN_TURN_RIGHT_ANGLE_DEG, now_us);
                    break;
                case APP_BAR_CMD_UTURN:
                    (void) mission_send_move(p_ctx, MISSION_MV_LEFT,
                                             RUN_UTURN_DEG, now_us);
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
 * @brief   SEARCH_LINE: sweep 60 toward the side the line was last seen,
 *          120 back, 60 again until the line is found; 5 s without it
 *          -> LINE_LOST_STOP (FR14).
 * @context mission_task
 */
static void run_search(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                       status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const limit_us = MISSION_SEARCH_TIMEOUT_MS * RUN_US_PER_MS;
    uint32_t const other    = (MISSION_MV_RIGHT == p_ctx->search_first)
                              ? MISSION_MV_LEFT : MISSION_MV_RIGHT;

    if (p_evt->b_has_event && (APP_EVT_LINE_FOUND == p_evt->type))
    {
        if (!p_ctx->b_is_quiet)
        {
            /* cast: elapsed ms < 2^31 */
            app_log("LINE FOUND again after %d ms of search\n",
                    (int32_t) ((now_us - p_ctx->search_start_us)
                               / RUN_US_PER_MS), 0, 0);
        }
        mission_stop_motion(p_ctx);
        run_follow_or_armed(p_ctx, p_snap, now_us);
    }
    else if (p_evt->b_has_event && (APP_EVT_MOVE_DONE == p_evt->type))
    {
        if (RUN_SUB_FIRST == p_ctx->substep)
        {
            (void) mission_send_move(p_ctx, other,
                                     2 * MISSION_SEARCH_SWEEP_DEG, /* 120 */
                                     now_us);
        }
        else if (RUN_SUB_SECOND == p_ctx->substep)
        {
            (void) mission_send_move(p_ctx, p_ctx->search_first,
                                     MISSION_SEARCH_SWEEP_DEG, now_us);
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
    else if (p_evt->b_has_event && ((APP_EVT_BARCODE == p_evt->type)
                                    || (APP_EVT_UNMAPPED == p_evt->type)
                                    || (APP_EVT_INVALID == p_evt->type)))
    {
        run_log_event(p_ctx, "BARCODE event (value %d) during the line"
                      " search: ignored\n", p_evt->value);
    }
    else
    {
        /* other events are ignored while searching */
    }

    if ((APP_RUN_SEARCH_LINE == p_ctx->run_state)
        && ((now_us - p_ctx->search_start_us) >= limit_us))
    {
        mission_stop_motion(p_ctx);
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
        run_creep_or_scan(p_ctx, p_snap, now_us);          /* S5-U54 */
    }

    if (run_creep_end(p_ctx, type, p_snap))
    {
        /* creep over: the scan has been requested this pass */
    }
    else if (APP_EVT_SIDE_TOO_CLOSE == type)
    {
        /* obstacle_task has already set ESTOP; no mid-bypass replanning:
           the chassis no longer faces the heading of the plan (R11)    */
        mission_stop_motion(p_ctx);
        run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
        p_ctx->avoid_step = APP_AVOID_WAIT_PLAN;
    }
    else
    {
        run_avoid_step(p_ctx, p_evt, type, p_snap, now_us);
    }

    if ((APP_RUN_AVOID == p_ctx->run_state) && p_ctx->b_fr9_running
        && ((now_us - p_ctx->fr9_start_us) > fr9us)
        && (MISSION_REQ_NONE == p_ctx->sys_request))
    {
        if (!p_ctx->b_is_quiet)
        {
            /* casts: limit 20000 ms, step 0..6, substep 0..3 */
            app_log("REJOIN FAIL: FR9 %d ms limit, avoid step %d substep"
                    " %d\n", (int32_t) MISSION_FR9_LIMIT_MS,
                    (int32_t) p_ctx->avoid_step, (int32_t) p_ctx->substep);
        }
        fault_set(APP_FAULT_REACQUIRE_FAIL);       /* FR9: over the limit */
        run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
    }
}

/**
 * run_creep_or_scan
 * @brief   S5-U54: at standstill after the obstacle stop (~OBS_STOP_MM),
 *          creep forward to OBS_SCAN_MM (once per stop; obstacle_task
 *          guards it, S5-U55), or, already within OBS_CREEP_TOL_MM of it
 *          or with no range, ask for the scan at once.
 * @context mission_task
 */
static void run_creep_or_scan(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap,
                              uint32_t now_us)
{
    uint32_t const dist_mm = status_board_distance(p_snap);

    if ((!p_ctx->b_crept) && (APP_DIST_NONE_MM != dist_mm)
        && (dist_mm > (OBS_SCAN_MM + OBS_CREEP_TOL_MM)))
    {
        p_ctx->b_crept    = true;
        p_ctx->b_creeping = true;
        mission_mode_set(APP_MODE_STOPPED);         /* no RESCAN yet */
        safety_clear(SAFETY_ESTOP);
        if (!p_ctx->b_is_quiet)
        {
            /* casts: 0..2047 mm fit int32_t */
            app_log("CREEP: obstacle at %d mm, moving %d mm closer to"
                    " scan\n", (int32_t) dist_mm,
                    (int32_t) (dist_mm - OBS_SCAN_MM), 0);
        }
        /* cast: < 2048 mm fits int32_t */
        (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                 (int32_t) (dist_mm - OBS_SCAN_MM), now_us);
    }
    else
    {
        mission_mode_set(APP_MODE_STOPPED | APP_MODE_RESCAN);
    }
}

/**
 * run_creep_end
 * @brief   S5-U54: the creep ends when its move is done, or when
 *          obstacle_task's guard braked it (ESTOP set, standstill); the
 *          car is then held (ESTOP) and the scan requested.
 * @param   type APP_EVT_* of this pass (APP_EVT_COUNT = none)
 * @return  true if the creep ended in this pass
 * @context mission_task
 */
static bool run_creep_end(mission_ctx_t * p_ctx, uint32_t type,
                          status_board_t const * p_snap)
{
    bool const b_guard = (0u != (safety_get() & SAFETY_ESTOP))
                         && (APP_MOT_ESTOP
                             == status_board_motion_state(p_snap))
                         && (0 == status_board_speed_l(p_snap))
                         && (0 == status_board_speed_r(p_snap));
    bool       b_ended = false;

    if (p_ctx->b_creeping && ((APP_EVT_MOVE_DONE == type) || b_guard))
    {
        if (b_guard)
        {
            mission_stop_motion(p_ctx);             /* cancel the move */
        }
        p_ctx->b_creeping = false;
        safety_set(SAFETY_ESTOP);                   /* hold for the scan */
        mission_mode_set(APP_MODE_STOPPED | APP_MODE_RESCAN);
        if (!p_ctx->b_is_quiet)
        {
            /* cast: 0..2047 mm fits int32_t */
            app_log("CREEP done: obstacle at %d mm, scanning\n",
                    (int32_t) status_board_distance(p_snap), 0, 0);
        }
        b_ended = true;
    }

    return b_ended;
}

/**
 * run_avoid_step
 * @brief   Dispatches the current AVOID step.
 * @param   type APP_EVT_* of this pass (APP_EVT_COUNT = none)
 * @context mission_task
 */
static void run_avoid_step(mission_ctx_t * p_ctx,
                           mission_evt_in_t const * p_evt, uint32_t type,
                           status_board_t const * p_snap, uint32_t now_us)
{
    int32_t const diff_now = status_board_pos_l(p_snap)
                             - status_board_pos_r(p_snap);

    if ((APP_EVT_MOVE_DONE == type) && (!p_ctx->b_is_quiet)
        && (APP_AVOID_TURN_OUT <= p_ctx->avoid_step)
        && (APP_AVOID_REJOIN >= p_ctx->avoid_step))
    {
        /* Bring-up diagnosis: where each bypass move ended. casts: step
           1..5, path since the stop < 2^31 mm                          */
        app_log("AVOID step %d done: heading %d cdeg from the stop"
                " (+ = right), %d mm since the stop\n",
                (int32_t) p_ctx->avoid_step,
                odometry_heading_cdeg(diff_now, 0)
                - odometry_heading_cdeg(p_ctx->diff_at_stop, 0),
                (int32_t) odometry_counts_to_mm(status_board_path(p_snap)
                                                - p_ctx->path_at_stop));
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
            run_avoid_legs(p_ctx, type, now_us);
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
            run_post_profile(p_ctx, p_snap);
            run_turn(p_ctx, p_ctx->side, RUN_TURN_RIGHT_ANGLE_DEG, now_us);
            p_ctx->avoid_step = APP_AVOID_TURN_OUT;
            break;
        case APP_EVT_CONTINUE:
            safety_clear(SAFETY_ESTOP);
            mission_mode_clear(APP_MODE_RESCAN | APP_MODE_STOPPED);
            run_post_profile(p_ctx, p_snap);
            mission_mode_clear(RUN_INHIBITS);
            run_follow_or_armed(p_ctx, p_snap, now_us);
            break;
        case APP_EVT_NO_PATH:
            /* S5-U54 (review 2026-11-09): no lane keeps the car's path
               plus margin clear of every return. The scan was taken from
               OBS_SCAN_MM after the creep; reversing (blind behind) to
               look again from farther off only widens the smear, so the
               uncertainty ends in an explicit stop.                    */
            run_post_profile(p_ctx, p_snap);   /* width also when blocked */
            p_ctx->no_path_cnt++;
            if (!p_ctx->b_is_quiet)
            {
                app_log("NO_PATH: no bypass lane clears every scan return"
                        " with margin: stopped\n", 0, 0, 0);
            }
            run_request(p_ctx, APP_SYS_FAULT, APP_END_FAULT);
            break;
        default:
            break;                       /* still scanning */
    }
}

/**
 * run_avoid_legs
 * @brief   TURN_OUT -> LEG_OUT -> TURN_PARALLEL -> LEG_PASS -> REJOIN,
 *          one step per tagged MOVE_DONE.
 * @param   type   APP_EVT_* of this pass (APP_EVT_COUNT = none)
 * @param   now_us this pass
 * @context mission_task
 */
static void run_avoid_legs(mission_ctx_t * p_ctx, uint32_t type,
                           uint32_t now_us)
{
    uint32_t const other = (APP_SIDE_LEFT == p_ctx->side)
                           ? APP_SIDE_RIGHT : APP_SIDE_LEFT;

    if (APP_EVT_MOVE_DONE == type)
    {
        switch (p_ctx->avoid_step)
        {
            case APP_AVOID_TURN_OUT:
                /* cast: leg <= 1020 mm */
                (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                         (p_ctx->leg_lat_mm > 0u)
                                         ? (int32_t) p_ctx->leg_lat_mm
                                         : RUN_LEG_MIN_MM, now_us);
                p_ctx->avoid_step = APP_AVOID_LEG_OUT;
                break;
            case APP_AVOID_LEG_OUT:
                run_turn(p_ctx, other, RUN_TURN_RIGHT_ANGLE_DEG, now_us);
                p_ctx->avoid_step = APP_AVOID_TURN_PARALLEL;
                break;
            case APP_AVOID_TURN_PARALLEL:
                /* cast: leg <= 1020 mm */
                (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                         (p_ctx->leg_fwd_mm > 0u)
                                         ? (int32_t) p_ctx->leg_fwd_mm
                                         : RUN_LEG_MIN_MM, now_us);
                p_ctx->avoid_step = APP_AVOID_LEG_PASS;
                break;
            default:                    /* LEG_PASS done: turn to rejoin */
                run_turn(p_ctx, other, MISSION_REJOIN_DEG, now_us);
                p_ctx->avoid_step = APP_AVOID_REJOIN;
                p_ctx->substep    = RUN_SUB_FIRST;
                break;
        }
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
    uint32_t const lo   = (p_ctx->rejoin_e_mm * MISSION_REJOIN_LO_PCT)
                          / RUN_PCT;
    uint32_t const hi   = (p_ctx->rejoin_e_mm
                           * (RUN_PCT + MISSION_REJOIN_WIN_PCT)) / RUN_PCT;
    uint32_t       done = 0u;

    if ((RUN_SUB_FIRST == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        p_ctx->rejoin_e_mm  = (p_ctx->leg_lat_mm * RUN_SQRT2_X1000) / 1000u;
        p_ctx->rejoin_start = status_board_path(p_snap);
        p_ctx->substep      = RUN_SUB_SECOND;
        if (!p_ctx->b_is_quiet)
        {
            /* casts: distances < 2047 mm */
            app_log("REJOIN: line expected at %d mm (accept %d..%d mm)\n",
                    (int32_t) p_ctx->rejoin_e_mm,
                    (int32_t) ((p_ctx->rejoin_e_mm
                                * MISSION_REJOIN_LO_PCT) / RUN_PCT),
                    (int32_t) ((p_ctx->rejoin_e_mm
                                * (RUN_PCT + MISSION_REJOIN_WIN_PCT))
                               / RUN_PCT));
        }
        /* cast: <= 1020 x 1.414 x 1.2 < 2047 */
        (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                 (int32_t) ((p_ctx->rejoin_e_mm
                                             * (RUN_PCT
                                                + MISSION_REJOIN_WIN_PCT))
                                            / RUN_PCT), now_us);
    }
    else if ((RUN_SUB_SECOND == p_ctx->substep)
             && (APP_EVT_LINE_FOUND == type))
    {
        done = odometry_counts_to_mm(status_board_path(p_snap)
                                     - p_ctx->rejoin_start);
        if (!p_ctx->b_is_quiet)
        {
            /* casts: distance < 2047 mm, width 0..255 mm */
            app_log("REJOIN: LINE_FOUND at %d mm, width %d mm (want %d)\n",
                    (int32_t) done, (int32_t) p_evt->value,
                    (int32_t) MISSION_REJOIN_WIDTH_MM);
        }
        if ((done >= lo) && (done <= hi)
            && (p_evt->value + MISSION_REJOIN_WIDTH_TOL_MM
                >= MISSION_REJOIN_WIDTH_MM)
            && (p_evt->value
                <= (MISSION_REJOIN_WIDTH_MM + MISSION_REJOIN_WIDTH_TOL_MM)))
        {
            if (!p_ctx->b_is_quiet)
            {
                /* casts: distance < 2047 mm, width <= 75 mm */
                app_log("REJOIN accepted at %d mm, width %d mm\n",
                        (int32_t) done, (int32_t) p_evt->value, 0);
            }
            mission_stop_motion(p_ctx);
            /* cast: 60 mm constant */
            (void) mission_send_move(p_ctx, MISSION_MV_FWD,
                                     (int32_t) MISSION_JCT_ADVANCE_MM, now_us);
            p_ctx->substep = RUN_SUB_THIRD;
        }
    }
    else if ((RUN_SUB_SECOND == p_ctx->substep) && (APP_EVT_MOVE_DONE == type))
    {
        if (!p_ctx->b_is_quiet)
        {
            app_log("REJOIN FAIL: drove the whole window, no line accepted\n",
                    0, 0, 0);
        }
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
            (void) mission_send_move(p_ctx, MISSION_MV_LEFT, turn_deg,
                                     now_us);
        }
        else
        {
            (void) mission_send_move(p_ctx, MISSION_MV_RIGHT, turn_deg,
                                     now_us);
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
        if (((APP_RUN_FOLLOW == p_ctx->run_state)
             || (APP_RUN_ARMED == p_ctx->run_state))
            && (APP_SCAN_APPROACH != status_board_scanner_state(p_snap)))
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
 * @brief   Keeps the decoded letter and its MQTT-mapped command.
 * @param   value BARCODE event value (char bits 0-5, cmd bits 6-7)
 * @context mission_task
 */
static void run_arm(mission_ctx_t * p_ctx, uint32_t value)
{
    p_ctx->armed_char = value & RUN_BAR_CHAR_MASK;
    p_ctx->armed_cmd  = (value >> RUN_BAR_CMD_SHIFT) & RUN_BAR_CMD_MASK;
    p_ctx->b_is_armed = true;
    run_log_event(p_ctx, "BARCODE value %d: armed for the next"
                  " cross-junction\n", value);
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
 * @brief   Turns toward a side through mission_send_move().
 * @param   side      APP_SIDE_LEFT or APP_SIDE_RIGHT
 * @param   angle_deg 1..360
 * @param   now_us    this pass
 * @context mission_task
 */
static void run_turn(mission_ctx_t * p_ctx, uint32_t side, int32_t angle_deg,
                     uint32_t now_us)
{
    if (APP_SIDE_LEFT == side)
    {
        (void) mission_send_move(p_ctx, MISSION_MV_LEFT, angle_deg, now_us);
    }
    else
    {
        (void) mission_send_move(p_ctx, MISSION_MV_RIGHT, angle_deg, now_us);
    }
}

/**
 * run_post_profile
 * @brief   Sends the obstacle profile from the board to telemetry (MQTT
 *          PROFILE message: width, nearest, free gaps, pass side) and
 *          prints the same values on the console, after every scan.
 * @param   p_ctx  mission context (quiet flag)
 * @param   p_snap status board snapshot of this pass
 * @context mission_task
 */
static void run_post_profile(mission_ctx_t const * p_ctx,
                             status_board_t const * p_snap)
{
    telem_bin_profile_t prof = {0u, 0u, 0u, 0u, 0u, 0u};

    prof.angle_step     = status_board_angle_step(p_snap);
    prof.width_mm       = status_board_width(p_snap);
    prof.nearest_mm     = status_board_nearest(p_snap);
    prof.clear_left_mm  = status_board_clear_left(p_snap);
    prof.clear_right_mm = status_board_clear_right(p_snap);
    prof.side           = status_board_plan_side(p_snap);
    (void) telem_post_profile(&prof);   /* a full K7 is counted there */

    if (!p_ctx->b_is_quiet)
    {
        /* casts: distances 0..1020 mm, side 0..1 fit int32_t */
        app_log("OBSTACLE width %d mm, nearest %d mm ahead, pass side %d"
                " (0 = left, 1 = right; 1020 = clear)\n",
                (int32_t) prof.width_mm, (int32_t) prof.nearest_mm,
                (int32_t) prof.side);
        app_log("OBSTACLE free gap left %d mm, right %d mm\n",
                (int32_t) prof.clear_left_mm, (int32_t) prof.clear_right_mm,
                0);
    }
}

/**
 * run_log_event
 * @brief   Bring-up diagnosis: one console line with one value, unless
 *          the test matrix runs quiet.
 * @param   p_format printf-style format with one %d
 * @param   value    the value (event value or distance, < 2^31)
 * @context mission_task
 */
static void run_log_event(mission_ctx_t const * p_ctx,
                          char const * p_format, uint32_t value)
{
    if (!p_ctx->b_is_quiet)
    {
        app_log(p_format, (int32_t) value, 0, 0); /* cast: < 2^31 */
    }
}

/*** end of file ***/
