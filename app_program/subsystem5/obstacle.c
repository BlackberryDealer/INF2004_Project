/**
 * @file    obstacle.c
 * @brief   obstacle_task (60 ms, R3): ping, echo timing through R1,
 *          confirmation, and the Obstacle Scanner (Figure 6.4).
 *          Baseline representation (natural types).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR6, FR7, FR8 (scan + plan), NFR6, NFR10b, NFR17
 */

#include "obstacle.h"          /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "../app_types.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "cfg_obstacle.h"
#include "obs_profile.h"
#include "status_board.h"
#include "mission.h"
#include "sys_tick.h"
#include "sys_health.h"
#include "drv_gpio_irq.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

/* ---- private constants and types ------------------------------------ */

#define OBSTACLE_STACK_WORDS   (APP_STACK_OBSTACLE_BYTES / 8u)

/* Echo width -> mm. Sound travels 1 mm out and back in 5.83 us (343 m/s,
   20 C), so mm = us x 65536 / 5.83 >> 16 = (us x 11239) >> 16.
   SPEC-QUESTION Q3: B5.2.3 writes (width_us x 1130) >> 16 "= / 58",
   which gives centimetres, not millimetres (10x too small; the robot
   would stop 2.5 m early). The mm factor is used here. 40 000 us x
   11239 fits in 32 bits.                                             */
#define OBS_ECHO_MM_Q16        (11239u)
#define OBS_ECHO_Q16_SHIFT     (16u)
#define OBS_ECHO_OPEN_US       (4095u * 4u) /* "started, not finished" */
#define OBS_SEQ_MASK           (0x3u)       /* 2-bit ping sequence     */
#define OBS_CLEAR_HYST_MM      (50u)        /* APPROACH -> MONITOR     */
#define OBS_CLEAR_READINGS     (2u)
#define OBS_COARSE_STRIDE      (3u)         /* 15 deg between points   */
#define OBS_CENTRE_DEG         (90u)
#define OBS_STEP_NONE          (0xFFu)

/* Mode bits remembered for edge detection */
#define OBS_SEEN_RESCAN        (0x1u)
#define OBS_SEEN_STOPPED       (0x2u)
#define OBS_SEEN_RESUME        (0x4u)

/* Echo record: written only by obstacle_echo_isr (baseline: 3 fields;
   Phase 4 packs it into one 16-bit store, correction T7).           */
typedef struct
{
    uint32_t width_us;
    uint32_t seq;
    bool     b_is_valid;
} obstacle_echo_t;

/* One reading after the echo wait */
typedef struct
{
    uint32_t distance_mm;   /* APP_DIST_NONE_MM = nothing in range */
    bool     b_is_lost;     /* no completed echo this ping         */
    bool     b_is_confirmed;/* agrees with the previous reading    */
} obstacle_reading_t;

/* ---- private data ---------------------------------------------------- */

static uint64_t g_obstacle_stack[OBSTACLE_STACK_WORDS];
static ID       gh_obstacle_task = 0;
static ID       gh_echo_sem      = 0;     /* K1 SEM_ECHO */

static volatile obstacle_echo_t g_echo   = {0u, 0u, false}; /* ISR only */
static volatile uint32_t        g_rise_us   = 0u;  /* ISR-private (E5)  */
static volatile uint8_t         g_rise_seq  = 0u;  /* ISR-private       */
static volatile uint8_t         g_ping_seq  = 0u;  /* task writes only  */

/* Everything below is private to obstacle_task. */
static uint32_t           g_state      = APP_SCAN_MONITOR;
static uint32_t           g_prev_mm    = APP_DIST_NONE_MM;
static uint32_t           g_lost       = 0u;
static uint32_t           g_silent     = 0u;
static uint32_t           g_far        = 0u;
static uint32_t           g_mode_seen  = 0u;
static uint32_t           g_step       = 0u;
static uint32_t           g_settle     = 0u;
static uint32_t           g_pass_mark  = 0u;   /* path counts */
static uint32_t           g_scan_start_us = 0u;
static bool               gb_side_close_sent = false;
static bool               gb_is_fine_step[APP_SCAN_STEPS];
static uint16_t           g_obs_scan[APP_SCAN_STEPS];
static status_board_obs_t g_part =
{
    .distance_mm    = APP_DIST_NONE_MM,
    .scanner_state  = APP_SCAN_MONITOR,
    .width_mm       = APP_PROFILE_CLEAR_MM,
    .nearest_mm     = APP_PROFILE_CLEAR_MM,
    .clear_left_mm  = APP_PROFILE_CLEAR_MM,
    .clear_right_mm = APP_PROFILE_CLEAR_MM,
};

/* ---- private function prototypes ------------------------------------- */

static void     obstacle_task(INT stacd, void * p_exinf);
static void     obstacle_cycle(void);
static void     obstacle_echo_isr(uint32_t events, uint32_t now_us);
static void     obstacle_ping(obstacle_reading_t * p_rd);
static void     obstacle_modes(uint32_t * p_rising);
static void     obstacle_enter(uint32_t state);
static void     obstacle_step_monitor(obstacle_reading_t const * p_rd);
static void     obstacle_step_approach(obstacle_reading_t const * p_rd);
static void     obstacle_step_scan(obstacle_reading_t const * p_rd);
static void     obstacle_step_passing(void);
static void     obstacle_step_watch(obstacle_reading_t const * p_rd,
                                    uint32_t rising);
static void     obstacle_scan_begin(void);
static bool     obstacle_next_fine(void);
static void     obstacle_scan_finish(void);
static void     obstacle_fill_gaps(void);
static void     obstacle_set_step(uint32_t step);
static uint32_t obstacle_path_counts(void);

/* ---- public functions ------------------------------------------------- */

/**
 * obstacle_init
 * @brief   Creates K1 SEM_ECHO; sets up the trigger and the servo.
 * @return  E_OK, or the kernel error
 * @context usermain (init phase)
 */
ER obstacle_init(void)
{
    T_CSEM csem = {0};
    ER     ercd = E_OK;

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = 1;
    gh_echo_sem  = tk_cre_sem(&csem);
    if (gh_echo_sem < E_OK)
    {
        ercd = (ER) gh_echo_sem;  /* cast: negative ID is an error code */
    }
    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        g_obs_scan[idx] = (uint16_t) APP_DIST_NONE_MM; /* cast: 2044 */
    }
    drv_ultrasonic_init();
    drv_servo_init();

    return ercd;
}

/**
 * obstacle_start
 * @brief   Routes GP1 echo edges to obstacle_echo_isr and starts the task.
 * @return  E_OK, or the first error
 * @context usermain (start phase)
 */
ER obstacle_start(void)
{
    ER ercd = drv_gpio_irq_attach(PIN_US_ECHO, DRV_GPIO_IRQ_BOTH,
                                  obstacle_echo_isr);

    if (E_OK == ercd)
    {
        gh_obstacle_task = app_start_task((FP) obstacle_task, /* cast */
                                          APP_PRI_OBSTACLE,
                                          g_obstacle_stack,
                                          APP_STACK_OBSTACLE_BYTES,
                                          "obstacle_task");
        if (gh_obstacle_task < E_OK)
        {
            ercd = (ER) gh_obstacle_task;  /* cast: negative ID = error */
        }
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * obstacle_task
 * @brief   Every 60 ms (R3): one obstacle_cycle(), then check in.
 * @param   stacd   unused (kernel start code)
 * @param   p_exinf unused
 * @context obstacle_task (priority 15, woken by R3)
 */
static void obstacle_task(INT stacd, void * p_exinf)
{
    (void) stacd;    /* unused: kernel start code */
    (void) p_exinf;  /* unused: no extended info  */

    (void) sys_tick_register(SYS_TICK_SLOT_OBSTACLE,
                             tk_get_tid());  /* valid slot constant */
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);  /* woken by R3 */
        sys_tick_cycle_start(SYS_TICK_SLOT_OBSTACLE);
        obstacle_cycle();
        sys_health_check_in(SYS_HEALTH_BIT_OBSTACLE);
    }
}

/**
 * obstacle_cycle
 * @brief   One 60 ms cycle: ping, read the mode bits, run the scanner
 *          state, publish the obstacle part of the board.
 * @context obstacle_task
 */
static void obstacle_cycle(void)
{
    obstacle_reading_t reading = {APP_DIST_NONE_MM, true, false};
    uint32_t           rising  = 0u;

    obstacle_ping(&reading);
    obstacle_modes(&rising);

    if (0u != (rising & OBS_SEEN_RESUME))
    {
        obstacle_enter(APP_SCAN_MONITOR);
    }
    else
    {
        switch (g_state)
        {
            case APP_SCAN_MONITOR:
                obstacle_step_monitor(&reading);
                break;
            case APP_SCAN_APPROACH:
                obstacle_step_approach(&reading);
                break;
            case APP_SCAN_STOPPED:
                if ((0u != (rising & OBS_SEEN_RESCAN))
                    && mission_mode_is_set(APP_MODE_STOPPED))
                {
                    obstacle_scan_begin();
                }
                break;
            case APP_SCAN_COARSE:
            case APP_SCAN_FINE:
                obstacle_step_scan(&reading);
                break;
            case APP_SCAN_PASSING:
                obstacle_step_passing();
                break;
            case APP_SCAN_BYPASS_WATCH:
                obstacle_step_watch(&reading, rising);
                break;
            default:
                obstacle_enter(APP_SCAN_MONITOR);
                break;
        }
    }

    g_part.distance_mm   = reading.distance_mm;
    g_part.scanner_state = g_state;
    status_board_write_obstacle(&g_part);
}

/**
 * obstacle_echo_isr
 * @brief   R1 callback for GP1. Rising edge: remember the time and mark
 *          the ping as started. Falling edge of the same ping: store the
 *          width and signal SEM_ECHO. A late falling edge from an older
 *          ping (its rise had another sequence) is ignored.
 * @param   events DRV_GPIO_IRQ_RISE and/or _FALL
 * @param   now_us timer value read at R1 entry
 * @context R1 interrupt (no logging, no waiting)
 */
static void obstacle_echo_isr(uint32_t events, uint32_t now_us)
{
    uint8_t const seq = g_ping_seq;

    if (0u != (events & DRV_GPIO_IRQ_RISE))
    {
        g_rise_us         = now_us;
        g_rise_seq        = seq;
        g_echo.width_us   = OBS_ECHO_OPEN_US;
        g_echo.seq        = (uint32_t) seq;   /* cast: widen 0..3 */
        g_echo.b_is_valid = false;
    }
    if ((0u != (events & DRV_GPIO_IRQ_FALL)) && (g_rise_seq == seq))
    {
        g_echo.width_us   = now_us - g_rise_us;   /* unsigned: wrap-safe */
        g_echo.seq        = (uint32_t) seq;   /* cast: widen 0..3 */
        g_echo.b_is_valid = true;
        (void) tk_sig_sem(gh_echo_sem, 1);        /* E_QOVR is harmless */
    }
}

/**
 * obstacle_ping
 * @brief   One measurement: trigger, wait for the echo (40 ms timeout),
 *          convert, count silent pings, confirm against the previous one.
 * @param   p_rd reading out
 * @context obstacle_task
 */
static void obstacle_ping(obstacle_reading_t * p_rd)
{
    obstacle_echo_t rec  = {0u, 0u, false};
    uint32_t        seq  = 0u;
    uint32_t        diff = 0u;

    (void) tk_wai_sem(gh_echo_sem, 1, TMO_POL);   /* drop a stale signal */
    seq        = ((uint32_t) g_ping_seq + 1u) & OBS_SEQ_MASK; /* cast: widen */
    g_ping_seq = (uint8_t) seq;                   /* cast: 0..3 */
    drv_ultrasonic_trigger();
    (void) tk_wai_sem(gh_echo_sem, 1,
                      (TMO) OBS_ECHO_TIMEOUT_MS); /* cast: kernel type */

    rec.width_us   = g_echo.width_us;             /* read the record once */
    rec.seq        = g_echo.seq;
    rec.b_is_valid = g_echo.b_is_valid;

    p_rd->distance_mm = APP_DIST_NONE_MM;
    p_rd->b_is_lost   = true;
    if (rec.seq != seq)
    {
        g_silent++;                               /* no edge at all */
        if (g_silent >= OBS_NO_ECHO_PINGS)
        {
            g_silent = OBS_NO_ECHO_PINGS;
            fault_set(APP_FAULT_NO_ECHO);
        }
    }
    else
    {
        g_silent = 0u;
        if (rec.b_is_valid)
        {
            p_rd->b_is_lost   = false;
            p_rd->distance_mm = (rec.width_us * OBS_ECHO_MM_Q16)
                                >> OBS_ECHO_Q16_SHIFT;
            if (p_rd->distance_mm > APP_DIST_NONE_MM)
            {
                p_rd->distance_mm = APP_DIST_NONE_MM;
            }
        }
    }

    diff = (p_rd->distance_mm > g_prev_mm)
           ? (p_rd->distance_mm - g_prev_mm)
           : (g_prev_mm - p_rd->distance_mm);
    p_rd->b_is_confirmed = (diff <= OBS_AGREE_MM);
    g_prev_mm            = p_rd->distance_mm;
}

/**
 * obstacle_modes
 * @brief   Samples RESCAN, STOPPED and RESUME and reports 0 -> 1 edges.
 * @param   p_rising OBS_SEEN_* bits that rose since the last cycle
 * @context obstacle_task
 */
static void obstacle_modes(uint32_t * p_rising)
{
    uint32_t now = 0u;

    if (mission_mode_is_set(APP_MODE_RESCAN))
    {
        now |= OBS_SEEN_RESCAN;
    }
    if (mission_mode_is_set(APP_MODE_STOPPED))
    {
        now |= OBS_SEEN_STOPPED;
    }
    if (mission_mode_is_set(APP_MODE_RESUME))
    {
        now |= OBS_SEEN_RESUME;
    }
    *p_rising   = now & ~g_mode_seen;
    g_mode_seen = now;
}

/**
 * obstacle_enter
 * @brief   Changes scanner state and does its entry action.
 * @param   state APP_SCAN_*
 * @context obstacle_task
 */
static void obstacle_enter(uint32_t state)
{
    g_state = state;
    g_far   = 0u;
    g_lost  = 0u;

    if ((APP_SCAN_MONITOR == state) || (APP_SCAN_PASSING == state))
    {
        drv_servo_set_deg(OBS_CENTRE_DEG);
    }
    if (APP_SCAN_PASSING == state)
    {
        g_pass_mark = obstacle_path_counts();
    }
    if (APP_SCAN_BYPASS_WATCH == state)
    {
        gb_side_close_sent = false;
        if (APP_SIDE_LEFT == g_part.plan_side)
        {
            drv_servo_set_deg(OBS_WATCH_DEG_LEFT_PASS);  /* obstacle right */
        }
        else
        {
            drv_servo_set_deg(OBS_WATCH_DEG_RIGHT_PASS); /* obstacle left  */
        }
    }
}

/**
 * obstacle_step_monitor
 * @brief   MONITOR: a confirmed reading under 400 mm -> OBSTACLE_AHEAD.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_monitor(obstacle_reading_t const * p_rd)
{
    if (p_rd->b_is_confirmed && (p_rd->distance_mm < OBS_SLOW_MM))
    {
        (void) mission_post_event(APP_EVT_OBSTACLE_AHEAD, 0u); /* counted */
        obstacle_enter(APP_SCAN_APPROACH);
    }
}

/**
 * obstacle_step_approach
 * @brief   APPROACH: confirmed under OBS_STOP_MM, or two lost echoes in a
 *          row -> ESTOP first, then OBSTACLE. Two far readings -> CONTINUE.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_approach(obstacle_reading_t const * p_rd)
{
    g_lost = p_rd->b_is_lost ? (g_lost + 1u) : 0u;
    g_far  = ((!p_rd->b_is_lost)
              && (p_rd->distance_mm > (OBS_SLOW_MM + OBS_CLEAR_HYST_MM)))
             ? (g_far + 1u) : 0u;

    if ((p_rd->b_is_confirmed && (p_rd->distance_mm < OBS_STOP_MM))
        || (g_lost >= OBS_LOST_PINGS))
    {
        safety_set(SAFETY_ESTOP);                 /* brake first (NFR6) */
        (void) mission_post_event(APP_EVT_OBSTACLE, 0u);   /* counted */
        obstacle_enter(APP_SCAN_STOPPED);
    }
    else if (g_far >= OBS_CLEAR_READINGS)
    {
        (void) mission_post_event(APP_EVT_CONTINUE, 0u);   /* counted */
        obstacle_enter(APP_SCAN_MONITOR);
    }
    else
    {
        /* keep approaching; mission has slowed down */
    }
}

/**
 * obstacle_step_scan
 * @brief   COARSE / FINE: settle the servo for OBS_SETTLE_PINGS cycles,
 *          then store this cycle's reading for the current step.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_scan(obstacle_reading_t const * p_rd)
{
    bool b_is_more = true;

    if (g_settle > 0u)
    {
        g_settle--;                               /* servo still moving */
    }
    else
    {
        g_obs_scan[g_step] = (uint16_t) p_rd->distance_mm; /* cast: <=2044 */
        if (APP_SCAN_COARSE == g_state)
        {
            if ((g_step + OBS_COARSE_STRIDE) < APP_SCAN_STEPS)
            {
                obstacle_set_step(g_step + OBS_COARSE_STRIDE);
            }
            else
            {
                g_state   = APP_SCAN_FINE;
                b_is_more = obstacle_next_fine();
            }
        }
        else
        {
            b_is_more = obstacle_next_fine();
        }
    }

    if (!b_is_more)
    {
        obstacle_scan_finish();
    }
}

/**
 * obstacle_step_passing
 * @brief   PASSING: no re-trigger until the robot has driven past the
 *          obstacle (nearest + assumed depth + robot length).
 * @context obstacle_task
 */
static void obstacle_step_passing(void)
{
    uint32_t const need_mm = g_part.nearest_mm + OBS_DEPTH_ASSUMED_MM
                             + OBS_ROBOT_LEN_MM;
    uint32_t const done_mm = odometry_counts_to_mm(obstacle_path_counts()
                                                   - g_pass_mark);

    if (done_mm >= need_mm)
    {
        obstacle_enter(APP_SCAN_MONITOR);
    }
}

/**
 * obstacle_step_watch
 * @brief   BYPASS_WATCH: sensor points at the obstacle side; too close ->
 *          SIDE_TOO_CLOSE once; RESCAN rising -> new scan.
 * @param   p_rd   this cycle's reading
 * @param   rising mode bits that rose this cycle
 * @context obstacle_task
 */
static void obstacle_step_watch(obstacle_reading_t const * p_rd,
                                uint32_t rising)
{
    if (0u != (rising & OBS_SEEN_RESCAN))
    {
        obstacle_scan_begin();
    }
    else if (p_rd->b_is_confirmed && (!gb_side_close_sent)
             && (p_rd->distance_mm < OBS_SIDE_TOO_CLOSE_MM))
    {
        (void) mission_post_event(APP_EVT_SIDE_TOO_CLOSE, 0u); /* counted */
        gb_side_close_sent = true;
    }
    else
    {
        /* keep watching */
    }
}

/**
 * obstacle_scan_begin
 * @brief   Starts a coarse scan at step 0 (30 deg); profile invalid until
 *          the scan is done.
 * @context obstacle_task (robot stopped)
 */
static void obstacle_scan_begin(void)
{
    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        g_obs_scan[idx]      = (uint16_t) APP_DIST_NONE_MM; /* cast: 2044 */
        gb_is_fine_step[idx] = false;
    }
    g_part.b_is_profile_valid = false;
    g_scan_start_us           = sys_tick_now_us();
    g_state                   = APP_SCAN_COARSE;
    obstacle_set_step(0u);
}

/**
 * obstacle_next_fine
 * @brief   Moves to the next 5-degree step between two coarse points whose
 *          near/far class differs.
 * @return  true if a fine step was started, false if the scan is done
 * @context obstacle_task
 */
static bool obstacle_next_fine(void)
{
    uint32_t next    = OBS_STEP_NONE;
    uint32_t coarse  = 0u;
    bool     b_near  = false;
    bool     b_near2 = false;

    for (uint32_t idx = 0u; (idx < APP_SCAN_STEPS) && (OBS_STEP_NONE == next);
         idx++)
    {
        coarse = idx - (idx % OBS_COARSE_STRIDE);
        if (((idx % OBS_COARSE_STRIDE) != 0u)
            && ((coarse + OBS_COARSE_STRIDE) < APP_SCAN_STEPS)
            && (!gb_is_fine_step[idx]))
        {
            b_near  = (g_obs_scan[coarse] < OBS_NEAR_MM);
            b_near2 = (g_obs_scan[coarse + OBS_COARSE_STRIDE] < OBS_NEAR_MM);
            if (b_near != b_near2)
            {
                next = idx;
            }
        }
    }

    if (OBS_STEP_NONE != next)
    {
        gb_is_fine_step[next] = true;
        obstacle_set_step(next);
    }

    return (OBS_STEP_NONE != next);
}

/**
 * obstacle_scan_finish
 * @brief   Builds the profile and plan, writes them to the board first,
 *          then posts PLAN / CONTINUE / NO_PATH.
 * @context obstacle_task
 */
static void obstacle_scan_finish(void)
{
    obs_profile_t prof = {false, 0u, 0u, 0, 0, 0u, 0u, 0u};
    obs_plan_t    plan = {OBS_OUT_NO_PATH, APP_SIDE_LEFT, 0u, 0u, false};

    obstacle_fill_gaps();
    obs_profile_build(g_obs_scan, &prof);
    obs_profile_plan(&prof, &plan);
    if (plan.b_was_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    g_part.b_is_profile_valid = prof.b_is_valid;
    g_part.angle_step         = prof.angle_step;
    g_part.width_mm           = prof.width_mm;
    g_part.nearest_mm         = prof.nearest_mm;
    g_part.clear_left_mm      = prof.clear_left_mm;
    g_part.clear_right_mm     = prof.clear_right_mm;
    g_part.plan_side          = plan.side;
    g_part.leg_lateral_mm     = plan.leg_lateral_mm;
    g_part.leg_forward_mm     = plan.leg_forward_mm;
    g_part.scanner_state      = g_state;
    status_board_write_obstacle(&g_part);         /* board before event */

    /* casts: scan ms, outcome and side are all small */
    app_log("SCAN %d ms out %d side %d\n",
            (int32_t) ((sys_tick_now_us() - g_scan_start_us) / 1000u),
            (int32_t) plan.outcome, (int32_t) plan.side);

    drv_servo_set_deg(OBS_CENTRE_DEG);
    if (OBS_OUT_PLAN == plan.outcome)
    {
        (void) mission_post_event(APP_EVT_PLAN, plan.side);   /* counted */
        obstacle_enter(APP_SCAN_BYPASS_WATCH);
    }
    else if (OBS_OUT_CONTINUE == plan.outcome)
    {
        (void) mission_post_event(APP_EVT_CONTINUE, 0u);      /* counted */
        obstacle_enter(APP_SCAN_PASSING);
    }
    else
    {
        (void) mission_post_event(APP_EVT_NO_PATH, 0u);       /* counted */
        obstacle_enter(APP_SCAN_STOPPED);
    }
}

/**
 * obstacle_fill_gaps
 * @brief   Gives every 5-degree step that was not scanned the larger of
 *          its two coarse neighbours: an object seen at both keeps its
 *          points contiguous, and no point is made nearer than measured.
 * @context obstacle_task
 */
static void obstacle_fill_gaps(void)
{
    uint32_t coarse = 0u;
    uint16_t near_a = 0u;
    uint16_t near_b = 0u;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        coarse = idx - (idx % OBS_COARSE_STRIDE);
        if (((idx % OBS_COARSE_STRIDE) != 0u) && (!gb_is_fine_step[idx])
            && ((coarse + OBS_COARSE_STRIDE) < APP_SCAN_STEPS))
        {
            near_a          = g_obs_scan[coarse];
            near_b          = g_obs_scan[coarse + OBS_COARSE_STRIDE];
            g_obs_scan[idx] = (near_a > near_b) ? near_a : near_b;
        }
    }
}

/**
 * obstacle_set_step
 * @brief   Points the servo at a scan step and starts its settle time.
 * @param   step 0..24 (angle 30 + 5 x step)
 * @context obstacle_task (robot stopped)
 */
static void obstacle_set_step(uint32_t step)
{
    g_step   = step;
    g_settle = OBS_SETTLE_PINGS;
    drv_servo_set_deg(APP_SCAN_ANGLE_MIN_DEG
                      + (APP_SCAN_ANGLE_STEP_DEG * step));
}

/**
 * obstacle_path_counts
 * @brief   Current path length from the status board.
 * @return  counts (sum of |travel|)
 * @context obstacle_task
 */
static uint32_t obstacle_path_counts(void)
{
    status_board_t snap = {0};

    status_board_read(&snap);

    return status_board_path(&snap);
}

/*** end of file ***/
