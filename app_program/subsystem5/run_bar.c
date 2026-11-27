/**
 * @file    run_bar.c
 * @brief   Barcode rewind-and-reread recovery state machine (RUN helper).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR3, FR14, NFR15
 */

#include "run_bar.h"               /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/odometry.h"
#include "../subsystem3/barcode.h"
#include "cfg_mission.h"
#include "mission.h"

/* ---- private constants and types ------------------------------------ */

#define RUN_BAR_INHIBITS      (APP_MODE_BARCODE_INHIBIT \
                               | APP_MODE_JUNCTION_INHIBIT)

/* Active phases stored in mission_ctx_t.barcode_phase (0 = RUN_BAR_IDLE). */
#define RUN_BAR_BRAKING       (1u)
#define RUN_BAR_REVERSING     (2u)
#define RUN_BAR_SETTLING      (3u)
#define RUN_BAR_READING       (4u)

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

static bool     run_bar_still(status_board_t const * p_snap);
static bool     run_bar_scan_ok(status_board_t const * p_snap);
static uint32_t run_bar_brake_timeout(mission_ctx_t const * p_ctx,
                                      uint32_t now_us);
static uint32_t run_bar_braked(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap, uint32_t now_us);
static void     run_bar_back(mission_ctx_t * p_ctx, uint32_t now_us);
static bool     run_bar_white_lead(mission_ctx_t * p_ctx, uint32_t path);
static uint32_t run_bar_settled(mission_ctx_t * p_ctx,
                                status_board_t const * p_snap,
                                uint32_t now_us);
static uint32_t run_bar_read(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us);
static bool     run_bar_retry(mission_ctx_t * p_ctx,
                              status_board_t const * p_snap, uint32_t now_us);
static void     run_bar_log_event(mission_ctx_t const * p_ctx,
                                  char const * p_format, uint32_t value);

/* ---- public functions ------------------------------------------------- */

/**
 * run_bar_begin
 * @brief   Store a rewind target for a Code 39 start guard.
 *          Keep that target fixed if the line is briefly uncertain.
 * @param   p_ctx       mission context
 * @param   advance_mm  distance past the first observed bar
 * @param   p_snap      status board snapshot of this pass
 * @param   now_us      this pass
 * @context mission_task
 */
void run_bar_begin(mission_ctx_t * p_ctx, uint32_t advance_mm,
                   status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const path = status_board_path(p_snap);
    bool           b_started = false;

    if ((advance_mm <= MISSION_BARCODE_REWIND_MM - MISSION_BARCODE_LEAD_MM)
        && ((!p_ctx->b_rearm_wait)
            || (odometry_path_since(path, p_ctx->barcode_rearm_path)
                >= odometry_mm_to_counts(MISSION_BARCODE_REARM_MM))))
    {
        uint32_t const counts = odometry_mm_to_counts(
            advance_mm + MISSION_BARCODE_LEAD_MM);

        if (odometry_path_since(path, p_ctx->barcode_follow_path)
            >= (counts + odometry_mm_to_counts(MISSION_BARCODE_BACK_PAD_MM)))
        {
            p_ctx->barcode_probe_mark = odometry_path_sub(path, counts);
            p_ctx->b_probe_pending    = true;
            p_ctx->barcode_since_us   = now_us;
            b_started                 = true;
        }
    }

    if (!b_started)
    {
        run_bar_log_event(p_ctx, "BARCODE start at %d mm, no safe rewind\n",
                          advance_mm);
    }
}

/**
 * run_bar_try_rewind
 * @brief   Wait briefly for visible or centred line tracking before braking.
 *          Cancel the pending retry when the time or distance runs out,
 *          or motion is no longer clear and under the follower's control.
 * @param   p_ctx   mission context
 * @param   p_snap  status board snapshot of this pass
 * @param   now_us  this pass
 * @context mission_task
 */
void run_bar_try_rewind(mission_ctx_t * p_ctx, status_board_t const * p_snap,
                        uint32_t now_us)
{
    if (p_ctx->b_probe_pending)
    {
        uint32_t const path = status_board_path(p_snap);
        uint32_t const need = odometry_counts_to_mm(
            odometry_path_since(path, p_ctx->barcode_probe_mark));
        uint32_t const line = status_board_line_status(p_snap);

        if ((need > MISSION_BARCODE_REWIND_MM - MISSION_BARCODE_BACK_PAD_MM)
            || ((now_us - p_ctx->barcode_since_us)
                >= MISSION_BARCODE_WAIT_MS * APP_US_PER_MS)
            || (APP_MOT_FOLLOW != status_board_motion_state(p_snap))
            || (status_board_speed_l(p_snap) <= 0)
            || (status_board_speed_r(p_snap) <= 0)
            || (APP_LINE_JUNCTION == line)
            || (!run_bar_scan_ok(p_snap))
            || mission_mode_is_set(RUN_BAR_INHIBITS) || (0u != safety_get()))
        {
            p_ctx->b_probe_pending = false;
            run_bar_log_event(p_ctx, "BARCODE no safe rewind (%d mm needed)\n",
                              need);
        }
        else if (APP_LINE_ON == line)
        {
            p_ctx->barcode_mark     = p_ctx->barcode_probe_mark;
            p_ctx->b_probe_pending  = false;
            p_ctx->barcode_phase    = RUN_BAR_BRAKING;
            p_ctx->barcode_since_us = now_us;
            p_ctx->barcode_tries    = 0u;
            mission_mode_set(RUN_BAR_INHIBITS | APP_MODE_BARCODE_REREAD);
            mission_stop_motion(p_ctx);
        }
        else
        {
            /* The line tracker may recover before the deadline. */
        }
    }
}

/**
 * run_bar_step
 * @brief   Brake, back up once, then follow the line at the reread speed.
 *          Common STOP, obstacle and fault handling has priority.
 * @param   p_ctx   mission context
 * @param   p_evt   this pass's event
 * @param   p_snap  status board snapshot of this pass
 * @param   now_us  this pass
 * @return  RUN_BAR_OUT_* action for mission_run.c
 * @context mission_task
 */
uint32_t run_bar_step(mission_ctx_t * p_ctx, mission_evt_in_t const * p_evt,
                      status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const path = status_board_path(p_snap);
    uint32_t       out  = RUN_BAR_OUT_ACTIVE;

    if (APP_EVT_LINE_LOST == type)
    {
        run_bar_clear(p_ctx, path);
        out = RUN_BAR_OUT_SEARCH;
    }
    else if (RUN_BAR_BRAKING == p_ctx->barcode_phase)
    {
        out = run_bar_braked(p_ctx, p_snap, now_us);
    }
    else if (RUN_BAR_REVERSING == p_ctx->barcode_phase)
    {
        /* Keep updating the quiet stretch, but use the full bounded
           rewind. An edge during early braking could erase the lead. */
        (void) run_bar_white_lead(p_ctx, path);
        if ((APP_EVT_MOVE_DONE == type) && (0u != p_ctx->barcode_back_left))
        {
            run_bar_back(p_ctx, now_us);
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
        out = run_bar_settled(p_ctx, p_snap, now_us);
    }
    else if (RUN_BAR_READING == p_ctx->barcode_phase)
    {
        out = run_bar_read(p_ctx, p_evt, p_snap, now_us);
    }
    else
    {
        /* No active barcode retry phase. */
    }

    return out;
}

/**
 * run_bar_clear
 * @brief   Release the retry modes and suppress a nearby duplicate probe.
 * @param   p_ctx  mission context
 * @param   path   current odometry path counts
 * @context mission_task
 */
void run_bar_clear(mission_ctx_t * p_ctx, uint32_t path)
{
    p_ctx->barcode_phase      = RUN_BAR_IDLE;
    p_ctx->barcode_rearm_path = path;
    p_ctx->b_rearm_wait       = true;
    mission_mode_clear(RUN_BAR_INHIBITS | APP_MODE_BARCODE_REREAD);
}

/* ---- private functions ------------------------------------------------ */

/**
 * run_bar_still
 * @brief   Both wheels have stopped and the safety flags allow movement.
 * @param   p_snap  status board snapshot
 * @return  true if braked at zero speed with no safety flags
 * @context mission_task
 */
static bool run_bar_still(status_board_t const * p_snap)
{
    return ((APP_MOT_BRAKE == status_board_motion_state(p_snap))
            && (0 == status_board_speed_l(p_snap))
            && (0 == status_board_speed_r(p_snap))
            && (0u == safety_get()));
}

/**
 * run_bar_scan_ok
 * @brief   The scanner allows a rewind: no obstacle stop or bypass. An
 *          obstacle ahead (APPROACH) is allowed, the car backs away from
 *          it and the sonar guard still stops the slow pass.
 * @param   p_snap  status board snapshot
 * @return  true if scanner state allows rewinding
 * @context mission_task
 */
static bool run_bar_scan_ok(status_board_t const * p_snap)
{
    uint32_t const state = status_board_scanner_state(p_snap);

    return ((APP_SCAN_MONITOR == state) || (APP_SCAN_APPROACH == state));
}

/**
 * run_bar_brake_timeout
 * @brief   A failed stop must not turn into a direction change.
 * @param   p_ctx   mission context
 * @param   now_us  this pass
 * @return  RUN_BAR_OUT_FAULT on timeout, RUN_BAR_OUT_ACTIVE otherwise
 * @context mission_task
 */
static uint32_t run_bar_brake_timeout(mission_ctx_t const * p_ctx,
                                      uint32_t now_us)
{
    uint32_t out = RUN_BAR_OUT_ACTIVE;

    if ((now_us - p_ctx->barcode_since_us)
        >= MISSION_BARCODE_BRAKE_MS * APP_US_PER_MS)
    {
        run_bar_log_event(p_ctx, "BARCODE brake timed out (%d ms)\n",
                          MISSION_BARCODE_BRAKE_MS);
        out = RUN_BAR_OUT_FAULT;
    }
    return out;
}

/**
 * run_bar_braked
 * @brief   Set the minimum rewind and bound it by the last follow stretch.
 * @param   p_ctx   mission context
 * @param   p_snap  status board snapshot
 * @param   now_us  this pass
 * @return  RUN_BAR_OUT_*
 * @context mission_task
 */
static uint32_t run_bar_braked(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t out = RUN_BAR_OUT_ACTIVE;

    if (run_bar_still(p_snap))
    {
        uint32_t const path = status_board_path(p_snap);
        uint32_t const need = odometry_counts_to_mm(
            odometry_path_since(path, p_ctx->barcode_mark));
        uint32_t const follow_mm = odometry_counts_to_mm(
            odometry_path_since(path, p_ctx->barcode_follow_path));
        uint32_t const available = (follow_mm > MISSION_BARCODE_BACK_PAD_MM)
            ? (follow_mm - MISSION_BARCODE_BACK_PAD_MM) : 0u;
        uint32_t const limit = (available < MISSION_BARCODE_REWIND_MM)
            ? available : MISSION_BARCODE_REWIND_MM;
        /* A retry returns to the start of its failed pass (S5-U81). */
        uint32_t const extra = (0u == p_ctx->barcode_tries)
            ? MISSION_BARCODE_BACK_EXTRA_MM : MISSION_BARCODE_RETRY_EXTRA_MM;
        uint32_t const back = ((need + extra) < limit)
            ? (need + extra) : limit;

        if ((need <= limit) && run_bar_scan_ok(p_snap))
        {
            p_ctx->barcode_mark      = odometry_path_add(
                path, odometry_mm_to_counts(need));
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
            run_bar_back(p_ctx, now_us);
        }
        else
        {
            run_bar_log_event(p_ctx, "BARCODE rewind skipped (%d mm needed)\n",
                              need);
            run_bar_clear(p_ctx, path);
            out = RUN_BAR_OUT_RESUME;
        }
    }
    else
    {
        out = run_bar_brake_timeout(p_ctx, now_us);
    }

    return out;
}

/**
 * run_bar_back
 * @brief   Send the next reverse move of the rewind. One move is at most
 *          MOTION_REVERSE_MAX_MM, so a long rewind takes several.
 * @param   p_ctx   mission context
 * @param   now_us  this pass
 * @context mission_task
 */
static void run_bar_back(mission_ctx_t * p_ctx, uint32_t now_us)
{
    /* cast: MOTION_REVERSE_MAX_MM is a positive constant (150 mm) */
    uint32_t const max_mm = (uint32_t) MOTION_REVERSE_MAX_MM;
    uint32_t const step   = (p_ctx->barcode_back_left < max_mm)
                            ? p_ctx->barcode_back_left : max_mm;

    p_ctx->barcode_back_left -= step;
    /* cast: step <= MOTION_REVERSE_MAX_MM */
    (void) mission_send_move(p_ctx, MISSION_MV_BACK, (int32_t) step, now_us);
}

/**
 * run_bar_white_lead
 * @brief   Require a quiet white stretch past the first observed bar.
 *          A white gap inside the strip is too short to count.
 * @param   p_ctx  mission context
 * @param   path   current odometry path counts
 * @return  true when past barcode_mark with enough white margin
 * @context mission_task
 */
static bool run_bar_white_lead(mission_ctx_t * p_ctx, uint32_t path)
{
    uint32_t const edges   = barcode_edge_count();
    bool const     b_white = (0u == barcode_do_level());

    if ((!b_white) || (edges != p_ctx->barcode_edges))
    {
        p_ctx->barcode_edge_path = path;
        p_ctx->barcode_edges     = edges;
    }
    return b_white && odometry_path_reached(path, p_ctx->barcode_mark)
           && (odometry_path_since(path, p_ctx->barcode_edge_path)
               >= odometry_mm_to_counts(MISSION_BARCODE_QUIET_MM));
}

/**
 * run_bar_settled
 * @brief   Check that white held through braking, then start the slow pass.
 * @param   p_ctx   mission context
 * @param   p_snap  status board snapshot
 * @param   now_us  this pass
 * @return  RUN_BAR_OUT_*
 * @context mission_task
 */
static uint32_t run_bar_settled(mission_ctx_t * p_ctx,
                                status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const path   = status_board_path(p_snap);
    bool const     b_lead = run_bar_white_lead(p_ctx, path);
    uint32_t       out    = RUN_BAR_OUT_ACTIVE;

    if (run_bar_still(p_snap))
    {
        uint32_t const backed = odometry_counts_to_mm(
            odometry_path_since(path, p_ctx->barcode_back_path));

        if (!b_lead)
        {
            run_bar_log_event(p_ctx, "BARCODE rewind ended after %d mm"
                              " without a white lead-in\n", backed);
            run_bar_clear(p_ctx, path);
            out = RUN_BAR_OUT_RESUME;
        }
        else if (motion_follow(MISSION_BARCODE_SPEED_MM_S))
        {
            if (!p_ctx->b_is_quiet)
            {
                /* casts: short rewind and white travel < 2047 mm */
                app_log("BARCODE white lead-in %d mm after rewinding %d mm\n",
                        (int32_t) odometry_counts_to_mm(
                            odometry_path_since(
                                path, p_ctx->barcode_edge_path)),
                        (int32_t) backed, 0);
            }
            p_ctx->barcode_mark     = path;
            p_ctx->barcode_since_us = now_us;
            p_ctx->barcode_phase    = RUN_BAR_READING;
            mission_mode_clear(RUN_BAR_INHIBITS);
            run_bar_log_event(p_ctx, "BARCODE reread at %d mm/s\n",
                              MISSION_BARCODE_SPEED_MM_S);
        }
        else
        {
            out = RUN_BAR_OUT_FAULT;
        }
    }
    else
    {
        out = run_bar_brake_timeout(p_ctx, now_us);
    }

    return out;
}

/**
 * run_bar_read
 * @brief   End the slow pass on a decoder result, the cross or its limit.
 * @param   p_ctx   mission context
 * @param   p_evt   this pass's event
 * @param   p_snap  status board snapshot
 * @param   now_us  this pass
 * @return  RUN_BAR_OUT_*
 * @context mission_task
 */
static uint32_t run_bar_read(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type    = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const path    = status_board_path(p_snap);
    bool const     b_cross = (APP_EVT_JUNCTION == type)
                             && (APP_JCT_CROSS == p_evt->value);
    bool           b_finished = true;
    uint32_t       out        = RUN_BAR_OUT_ACTIVE;

    switch (type)
    {
        case APP_EVT_BARCODE:
            break;
        case APP_EVT_UNMAPPED:
            run_bar_log_event(p_ctx, "BARCODE reread: char index %d has"
                              " no command\n", p_evt->value);
            break;
        case APP_EVT_INVALID:
            run_bar_log_event(p_ctx, "BARCODE reread invalid (%d elements)\n",
                              p_evt->value);
            break;
        default:
            b_finished = b_cross
                || (odometry_path_since(path, p_ctx->barcode_mark)
                    >= odometry_mm_to_counts(MISSION_BARCODE_READ_MM))
                || ((now_us - p_ctx->barcode_since_us)
                    >= MISSION_BARCODE_READ_MS * APP_US_PER_MS);
            if (b_finished)
            {
                run_bar_log_event(p_ctx, "BARCODE reread ended without decode"
                                  " after %d mm\n",
                                  odometry_counts_to_mm(odometry_path_since(
                                      path, p_ctx->barcode_mark)));
            }
            break;
    }

    if (b_finished && (APP_EVT_BARCODE != type) && (APP_EVT_UNMAPPED != type)
        && run_bar_retry(p_ctx, p_snap, now_us))
    {
        /* Rewinding to the start of this pass for another read. */
    }
    else if (b_finished)
    {
        if (APP_EVT_BARCODE == type)
        {
            /* Caller arms the command first, then calls run_bar_clear(). */
            out = RUN_BAR_OUT_ARMED;
        }
        else
        {
            run_bar_clear(p_ctx, path);
            out = (b_cross && p_ctx->b_is_armed)
                  ? RUN_BAR_OUT_CROSS : RUN_BAR_OUT_RESUME;
        }
    }
    else
    {
        /* Slow pass continues. */
    }

    return out;
}

/**
 * run_bar_retry
 * @brief   A slow pass ended without a decode: brake and rewind to the
 *          start of that pass (barcode_mark) for another slow pass, at
 *          most MISSION_BARCODE_RETRIES times (S5-U68).
 * @param   p_ctx   mission context
 * @param   p_snap  status board snapshot
 * @param   now_us  this pass
 * @return  true if the retry started
 * @context mission_task
 */
static bool run_bar_retry(mission_ctx_t * p_ctx, status_board_t const * p_snap,
                          uint32_t now_us)
{
    bool b_retry = false;

    if ((p_ctx->barcode_tries < MISSION_BARCODE_RETRIES)
        && run_bar_scan_ok(p_snap) && (0u == safety_get()))
    {
        p_ctx->barcode_tries++;
        p_ctx->barcode_phase    = RUN_BAR_BRAKING;
        p_ctx->barcode_since_us = now_us;
        mission_mode_set(RUN_BAR_INHIBITS | APP_MODE_BARCODE_REREAD);
        mission_stop_motion(p_ctx);
        run_bar_log_event(p_ctx, "BARCODE reread failed: rewinding for"
                          " retry %d\n", p_ctx->barcode_tries);
        b_retry = true;
    }
    return b_retry;
}

/**
 * run_bar_log_event
 * @brief   Bring-up diagnosis: one console line with one value, unless
 *          the test matrix runs quiet.
 * @param   p_ctx     mission context
 * @param   p_format  printf-style format with one %d
 * @param   value     the value (< 2^31)
 * @context mission_task
 */
static void run_bar_log_event(mission_ctx_t const * p_ctx,
                              char const * p_format, uint32_t value)
{
    if (!p_ctx->b_is_quiet)
    {
        app_log(p_format, (int32_t) value, 0, 0); /* cast: < 2^31 */
    }
}

/*** end of file ***/
