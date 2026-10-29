/**
 * @file    obstacle.c
 * @brief   obstacle_task (60 ms, R3): ping inside a measurement window,
 *          echo timing through R1, confirmation, and the Obstacle Scanner
 *          (Figure 6.4). Baseline representation (natural types, same
 *          4 mm and 4 us resolution as the packed build).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR6, FR7, FR8 (scan + plan), NFR6, NFR10b, NFR17
 */

/* Kernel side only (build rule, BARR_C_DEVIATIONS.md): no SDK headers;
   the echo pin is read through the kernel's in_w(GPIO_IN).           */
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

/* Echo width -> mm: sound needs 5.8 us per mm out and back, so
   d_mm = width_us x 10 / 58 = ((width_us x 11299) + 32768) >> 16,
   rounded (5800 us -> 1000 mm, 580 us -> 100 mm). 40 000 us x 11299
   fits in 32 bits. (Review R01: x 1130 gave centimetres.)           */
#define OBS_ECHO_MM_Q16        (11299u)
#define OBS_ECHO_ROUND_Q16     (32768u)
#define OBS_ECHO_Q16_SHIFT     (16u)
#define OBS_US_PER_MS          (1000u)
#define OBS_ECHO_TIMEOUT_US    (OBS_ECHO_TIMEOUT_MS * OBS_US_PER_MS)
#define OBS_WIDTH_SHIFT        (2u)          /* 4 us units            */
#define OBS_WIDTH_4US_MAX      (4095u)       /* 12 bits; SAT          */
#define OBS_SEQ_LOW_MASK       (0x3u)        /* record keeps 2 bits   */
#define OBS_MM_STEP            (4u)          /* stored resolution     */
#define OBS_CLEAR_HYST_MM      (50u)         /* APPROACH -> MONITOR   */
#define OBS_CLEAR_READINGS     (2u)
#define OBS_COARSE_STRIDE      (3u)          /* 15 deg between points */
#define OBS_CENTRE_DEG         (90u)
#define OBS_STEP_NONE          (0xFFu)
#define OBS_MASK_ALL           ((1u << APP_SCAN_STEPS) - 1u)  /* 25 bits */

/* Mode bits remembered for edge detection */
#define OBS_SEEN_RESCAN        (0x1u)
#define OBS_SEEN_STOPPED       (0x2u)
#define OBS_SEEN_RESUME        (0x4u)
#define OBS_DIST_LOG_PINGS     (8u)          /* bring-up print: ~0.5 s */

/* Echo record: written only by obstacle_echo_isr. Natural types, the
   same information as the packed MSG_ECHO word (Phase 4).            */
typedef struct
{
    uint32_t width_4us;     /* 0..4095 (4 us units, SAT)          */
    uint32_t seq;           /* low 2 bits of the ping number      */
    bool     b_is_valid;    /* a complete pulse was measured      */
} obstacle_echo_t;

/* One reading after the echo wait */
typedef struct
{
    uint32_t distance_mm;    /* 4 mm steps; APP_DIST_NONE_MM = nothing */
    bool     b_is_lost;      /* no completed echo this ping            */
    bool     b_is_confirmed; /* agrees with the previous reading       */
} obstacle_reading_t;

#define OBS_ECHO_PIN_MASK   (1u << PIN_US_ECHO)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_obstacle_stack[OBSTACLE_STACK_WORDS];
static ID       gh_obstacle_task = 0;
static ID       gh_echo_sem      = 0;     /* K1 SEM_ECHO */

/* Written only by the ISR (B5.2.2). */
static volatile obstacle_echo_t g_echo          = {0u, 0u, false};
static volatile uint8_t         g_echo_rise_cnt = 0u;  /* accepted rises */
static volatile uint8_t         g_echo_done_cnt = 0u;  /* accepted falls */
/* ISR-private. g_rise_us is E5: a stuck-high echo can exceed 65 ms. */
static volatile uint32_t        g_rise_us        = 0u;
static volatile uint8_t         g_rise_seq       = 0u;
static volatile bool            gb_is_rise_armed = false;
/* Written only by obstacle_task; the ISR only reads them. */
static volatile uint8_t         g_ping_seq         = 0u;  /* 8 bits, wraps */
static volatile uint8_t         g_echo_window_open = 0u;

/* Everything below is private to obstacle_task. */
static uint32_t           g_state         = APP_SCAN_MONITOR;
static uint32_t           g_prev_mm       = APP_DIST_NONE_MM;
static uint32_t           g_lost          = 0u;
static uint32_t           g_silent        = 0u;
static uint32_t           g_warmup_left   = OBS_WARMUP_PINGS; /* S5-U30 */
static uint32_t           g_far           = 0u;
static uint32_t           g_mode_seen     = 0u;
static uint32_t           g_step          = 0u;
static uint32_t           g_settle        = 0u;
static uint32_t           g_pass_mark     = 0u;   /* path counts */
static uint32_t           g_scan_start_us = 0u;
static uint32_t           g_measured      = 0u;   /* bit n: step measured */
static uint32_t           g_inferred      = 0u;   /* bit n: step filled   */
static uint32_t           g_dist_log_cnt  = 0u;   /* bring-up print divider */
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
static bool     obstacle_wait_echo(uint8_t done0);
static void     obstacle_read_record(obstacle_echo_t * p_rec);
static uint32_t obstacle_width_to_mm(uint32_t width_4us);
static uint32_t obstacle_round_mm(uint32_t distance_mm);
static void     obstacle_modes(uint32_t * p_rising);
static void     obstacle_enter(uint32_t state);
static void     obstacle_step_monitor(obstacle_reading_t const * p_rd);
static void     obstacle_step_approach(obstacle_reading_t const * p_rd);
static void     obstacle_step_scan(obstacle_reading_t const * p_rd);
static void     obstacle_step_settle(void);
static void     obstacle_step_passing(void);
static void     obstacle_step_watch(obstacle_reading_t const * p_rd);
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
        g_obs_scan[idx] = (uint16_t) APP_PROFILE_CLEAR_MM; /* cast: 1020 */
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
 * @brief   Every 60 ms (R3): check in first (before the echo wait, so the
 *          check-in spacing is the period, not the echo time), then one
 *          obstacle_cycle().
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
        sys_health_check_in(SYS_HEALTH_BIT_OBSTACLE);
        obstacle_cycle();
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

    bool               b_is_armed = false;

    obstacle_ping(&reading);
    obstacle_modes(&rising);

    /* The stop is armed only while the motors may drive (APP_MODE_MOTOR_EN:
       CAL_IR sweep and RUN). Before that there is nothing to stop, and an
       ESTOP latched in BOOT stayed set until RESET, so it blocked the
       CAL_IR sweep and the car went to FAULT without moving (on-car logs
       2026-10-28/29, test S5-U31). Pings, NO_ECHO and the board distance
       stay live while unarmed.                                          */
    b_is_armed = mission_mode_is_set(APP_MODE_MOTOR_EN);

    if (0u != (rising & OBS_SEEN_RESUME))
    {
        obstacle_enter(APP_SCAN_MONITOR);
    }
    else if (!b_is_armed)
    {
        if (APP_SCAN_MONITOR != g_state)
        {
            obstacle_enter(APP_SCAN_MONITOR);
        }

        /* Bring-up diagnosis: what the sensor sees while unarmed, every
           ~0.5 s. cast: distance 0..2047 mm, lost flag 0..1 fit int32_t */
        g_dist_log_cnt++;
        if (g_dist_log_cnt >= OBS_DIST_LOG_PINGS)
        {
            g_dist_log_cnt = 0u;
            app_log("US dist %d mm, lost %d\n",
                    (int32_t) reading.distance_mm,
                    reading.b_is_lost ? 1 : 0, 0);
        }
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
            case APP_SCAN_WATCH_SETTLE:
                obstacle_step_settle();
                break;
            case APP_SCAN_PASSING:
                obstacle_step_passing();
                break;
            case APP_SCAN_BYPASS_WATCH:
                obstacle_step_watch(&reading);
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
 * @brief   R1 callback for GP1 (B5.2.2). A rise counts only inside an
 *          open window; a fall only if the window is still open, a rise
 *          of this same ping is armed and the pulse is not older than the
 *          timeout. Accepted edges update the record first, then the
 *          freshness counter the task compares.
 * @param   events DRV_GPIO_IRQ_RISE and/or _FALL
 * @param   now_us timer value read at R1 entry
 * @context R1 interrupt (no logging, no waiting)
 */
static void obstacle_echo_isr(uint32_t events, uint32_t now_us)
{
    uint32_t width = 0u;
    uint32_t edges = events;

    /* Both edges in one R1 entry (R1 late by more than the shortest
       echo): their order is unknown and now_us - g_rise_us would be a
       false 0 mm width (fix brief, Fix 5). Do not measure. Pin high:
       the last edge was a rise, keep it (the fall before it is lost).
       Pin low: discard both; the ping counts as silent.               */
    if (DRV_GPIO_IRQ_BOTH == (edges & DRV_GPIO_IRQ_BOTH))
    {
        if (0u != (in_w(GPIO_IN) & OBS_ECHO_PIN_MASK))
        {
            edges = DRV_GPIO_IRQ_RISE;
        }
        else
        {
            edges            = 0u;
            gb_is_rise_armed = false;
        }
    }

    if (0u != (edges & DRV_GPIO_IRQ_RISE))
    {
        if (0u == g_echo_window_open)
        {
            gb_is_rise_armed = false;       /* rise outside a window */
        }
        else
        {
            g_rise_us         = now_us;
            g_rise_seq        = g_ping_seq;  /* captured at the rise */
            gb_is_rise_armed  = true;
            g_echo.width_4us  = OBS_WIDTH_4US_MAX;
            /* cast: widen the 8-bit ping number */
            g_echo.seq        = (uint32_t) g_rise_seq & OBS_SEQ_LOW_MASK;
            g_echo.b_is_valid = false;      /* started, not finished */
            g_echo_rise_cnt++;
        }
    }

    if (0u != (edges & DRV_GPIO_IRQ_FALL))
    {
        width = now_us - g_rise_us;         /* unsigned: wrap-safe */
        if ((0u != g_echo_window_open) && gb_is_rise_armed
            && (g_rise_seq == g_ping_seq) && (width <= OBS_ECHO_TIMEOUT_US))
        {
            width >>= OBS_WIDTH_SHIFT;      /* 4 us units */
            if (width > OBS_WIDTH_4US_MAX)
            {
                width = OBS_WIDTH_4US_MAX;
            }
            g_echo.width_4us  = width;
            /* cast: widen the 8-bit ping number */
            g_echo.seq        = (uint32_t) g_rise_seq & OBS_SEQ_LOW_MASK;
            g_echo.b_is_valid = true;
            g_echo_done_cnt++;
            gb_is_rise_armed  = false;
            (void) tk_sig_sem(gh_echo_sem, 1);      /* E_QOVR is harmless */
        }
        else
        {
            gb_is_rise_armed = false;       /* stale or stuck pulse */
        }
    }
}

/**
 * obstacle_ping
 * @brief   One measurement (B5.2.3 steps 2-4): drain stale tokens; if the
 *          echo line is already high do not trigger (silent); else open a
 *          window, trigger, wait to an absolute 40 ms deadline, close the
 *          window and judge only what happened inside it.
 * @param   p_rd reading out
 * @context obstacle_task
 */
static void obstacle_ping(obstacle_reading_t * p_rd)
{
    obstacle_echo_t rec    = {0u, 0u, false};
    uint8_t         rise0  = 0u;
    uint8_t         done0  = 0u;
    bool            b_rise = false;
    bool            b_done = false;
    uint32_t        diff   = 0u;

    g_echo_window_open = 0u;
    while (E_OK == tk_wai_sem(gh_echo_sem, 1, TMO_POL))
    {
        /* drop a stale token (maxsem 1: at most one) */
    }

    if (0u == (in_w(GPIO_IN) & OBS_ECHO_PIN_MASK))   /* SIO GPIO_IN */
    {
        g_ping_seq++;                       /* 8-bit wrap */
        rise0              = g_echo_rise_cnt;
        done0              = g_echo_done_cnt;
        g_echo_window_open = 1u;
        drv_ultrasonic_trigger();
        (void) obstacle_wait_echo(done0);
        g_echo_window_open = 0u;            /* later rises are disarmed */
        obstacle_read_record(&rec);
        b_rise = (rise0 != g_echo_rise_cnt);
        b_done = (done0 != g_echo_done_cnt);
    }

    p_rd->distance_mm = APP_DIST_NONE_MM;
    p_rd->b_is_lost   = true;
    /* cast: widen the 8-bit ping number */
    if (b_done && rec.b_is_valid
        && (rec.seq == ((uint32_t) g_ping_seq & OBS_SEQ_LOW_MASK)))
    {
        g_silent          = 0u;             /* echo complete */
        g_warmup_left     = 0u;             /* sensor is up */
        p_rd->b_is_lost   = false;
        p_rd->distance_mm = obstacle_width_to_mm(rec.width_4us);
    }
    else if (b_rise)
    {
        g_silent      = 0u;                 /* still high: nothing in range */
        g_warmup_left = 0u;                 /* an edge: sensor is up */
    }
    else if (0u != g_warmup_left)
    {
        g_warmup_left--;                    /* power-up grace (S5-U30) */
    }
    else
    {
        if (g_silent < OBS_NO_ECHO_PINGS)   /* no edge at all (or stuck) */
        {
            g_silent++;
        }
        if (g_silent >= OBS_NO_ECHO_PINGS)
        {
            fault_set(APP_FAULT_NO_ECHO);   /* again every silent ping */
        }
    }

    diff = (p_rd->distance_mm > g_prev_mm)
           ? (p_rd->distance_mm - g_prev_mm)
           : (g_prev_mm - p_rd->distance_mm);
    p_rd->b_is_confirmed = (diff <= OBS_AGREE_MM);
    g_prev_mm            = p_rd->distance_mm;
}

/**
 * obstacle_wait_echo
 * @brief   Waits for a completion in this window, to an absolute deadline
 *          of OBS_ECHO_TIMEOUT_MS after the trigger.
 * @param   done0 accepted-fall count saved when the window opened
 * @return  true if a fresh completion arrived
 * @context obstacle_task
 */
static bool obstacle_wait_echo(uint8_t done0)
{
    uint32_t const start_us  = sys_tick_now_us();
    uint32_t       elapsed   = 0u;
    uint32_t       left_ms   = 0u;
    bool           b_is_done = false;
    bool           b_is_over = false;

    while ((!b_is_done) && (!b_is_over))
    {
        elapsed = sys_tick_now_us() - start_us;     /* wrap-safe */
        if (elapsed >= OBS_ECHO_TIMEOUT_US)
        {
            b_is_over = true;
        }
        else
        {
            /* event time in a 60 ms task: the division is allowed */
            left_ms = ((OBS_ECHO_TIMEOUT_US - elapsed) + OBS_US_PER_MS - 1u)
                      / OBS_US_PER_MS;
            /* cast: <= 40 ms; a timeout is fine, the loop re-checks */
            (void) tk_wai_sem(gh_echo_sem, 1, (TMO) left_ms);
            b_is_done = (done0 != g_echo_done_cnt);
        }
    }

    return b_is_done;
}

/**
 * obstacle_read_record
 * @brief   Coherent read of the ISR's 3-field record: read twice until
 *          both copies match (the task cannot interrupt the ISR, so two
 *          equal reads are consistent). Phase 4: one 16-bit load.
 * @param   p_rec copy out
 * @context obstacle_task
 */
static void obstacle_read_record(obstacle_echo_t * p_rec)
{
    obstacle_echo_t again   = {0u, 0u, false};
    bool            b_equal = false;

    while (!b_equal)
    {
        p_rec->width_4us  = g_echo.width_4us;
        p_rec->seq        = g_echo.seq;
        p_rec->b_is_valid = g_echo.b_is_valid;
        again.width_4us   = g_echo.width_4us;
        again.seq         = g_echo.seq;
        again.b_is_valid  = g_echo.b_is_valid;
        b_equal = (p_rec->width_4us == again.width_4us)
                  && (p_rec->seq == again.seq)
                  && (p_rec->b_is_valid == again.b_is_valid);
    }
}

/**
 * obstacle_width_to_mm
 * @brief   Echo width (4 us units) -> distance, rounded, in 4 mm steps,
 *          saturated at APP_DIST_NONE_MM.
 * @param   width_4us 0..4095
 * @return  mm
 * @context obstacle_task (host tests)
 */
static uint32_t obstacle_width_to_mm(uint32_t width_4us)
{
    uint32_t const width_us = width_4us << OBS_WIDTH_SHIFT;
    uint32_t const dist_mm  = ((width_us * OBS_ECHO_MM_Q16)
                               + OBS_ECHO_ROUND_Q16) >> OBS_ECHO_Q16_SHIFT;

    return obstacle_round_mm(dist_mm);
}

/**
 * obstacle_round_mm
 * @brief   Rounds to the stored 4 mm step and saturates at the "nothing"
 *          sentinel, so the baseline holds what the packed build holds.
 * @param   distance_mm any
 * @return  0..APP_DIST_NONE_MM, a multiple of 4
 * @context obstacle_task
 */
static uint32_t obstacle_round_mm(uint32_t distance_mm)
{
    uint32_t rounded = ((distance_mm + (OBS_MM_STEP / 2u)) / OBS_MM_STEP)
                       * OBS_MM_STEP;

    if (rounded > APP_DIST_NONE_MM)
    {
        rounded = APP_DIST_NONE_MM;
    }

    return rounded;
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
    if (APP_SCAN_WATCH_SETTLE == state)
    {
        g_settle = OBS_WATCH_SETTLE_PINGS;
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
        /* Bring-up diagnosis (after the brake). casts: distance 0..2047
           and lost count 0..3 fit int32_t                              */
        app_log("OBSTACLE STOP (ESTOP): distance %d mm, lost echoes %d\n",
                (int32_t) p_rd->distance_mm, (int32_t) g_lost, 0);
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
 *          then store this cycle's reading for the current step (4 mm
 *          steps, >= 1020 = clear) and mark it measured.
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
        /* cast: <= 1020 after the clamp */
        g_obs_scan[g_step] = (uint16_t) ((p_rd->distance_mm
                                          < APP_PROFILE_CLEAR_MM)
                                         ? p_rd->distance_mm
                                         : APP_PROFILE_CLEAR_MM);
        g_measured |= (1u << g_step);
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
 * obstacle_step_settle
 * @brief   WATCH_SETTLE: the robot stays braked (ESTOP still set) while the
 *          servo travels to the watch angle; after exactly
 *          OBS_WATCH_SETTLE_PINGS cycles post PLAN (mission releases ESTOP
 *          only on PLAN).
 * @context obstacle_task
 */
static void obstacle_step_settle(void)
{
    if (g_settle > 0u)
    {
        g_settle--;
    }
    if (0u == g_settle)
    {
        (void) mission_post_event(APP_EVT_PLAN, g_part.plan_side); /* cnt */
        obstacle_enter(APP_SCAN_BYPASS_WATCH);
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
 * @brief   BYPASS_WATCH: the sensor points at the obstacle side; too close
 *          -> ESTOP first, then SIDE_TOO_CLOSE (mission goes to FAULT),
 *          then STOPPED.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_watch(obstacle_reading_t const * p_rd)
{
    if (p_rd->b_is_confirmed && (p_rd->distance_mm < OBS_SIDE_TOO_CLOSE_MM))
    {
        safety_set(SAFETY_ESTOP);
        (void) mission_post_event(APP_EVT_SIDE_TOO_CLOSE, 0u); /* counted */
        /* Bring-up diagnosis (after the brake). cast: 0..2047 mm */
        app_log("OBSTACLE SIDE TOO CLOSE (ESTOP): distance %d mm\n",
                (int32_t) p_rd->distance_mm, 0, 0);
        obstacle_enter(APP_SCAN_STOPPED);
    }
}

/**
 * obstacle_scan_begin
 * @brief   Starts a coarse scan at step 0 (30 deg) with every step
 *          unmeasured; the profile is invalid until the scan is done.
 * @context obstacle_task (robot stopped)
 */
static void obstacle_scan_begin(void)
{
    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        g_obs_scan[idx] = (uint16_t) APP_PROFILE_CLEAR_MM; /* cast: 1020 */
    }
    g_measured                = 0u;
    g_inferred                = 0u;
    g_part.b_is_profile_valid = false;
    g_scan_start_us           = sys_tick_now_us();
    g_state                   = APP_SCAN_COARSE;
    obstacle_set_step(0u);
}

/**
 * obstacle_next_fine
 * @brief   Moves to the next unmeasured 5-degree step between two coarse
 *          points whose near/far class differs.
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
            && (0u == (g_measured & (1u << idx))))
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
        obstacle_set_step(next);
    }

    return (OBS_STEP_NONE != next);
}

/**
 * obstacle_scan_finish
 * @brief   Fills unmeasured steps, builds the profile and plan, writes
 *          them to the board first, then: PLAN -> servo to the watch angle
 *          and WATCH_SETTLE (PLAN posted later); CONTINUE -> post, PASSING;
 *          NO_PATH -> post, STOPPED.
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

    /* stored in 4 mm steps (same information as the packed build);
       legs round up so a rounded leg is never shorter than planned */
    g_part.b_is_profile_valid = prof.b_is_valid;
    g_part.angle_step         = prof.angle_step;
    g_part.width_mm           = obstacle_round_mm(prof.width_mm);
    g_part.nearest_mm         = obstacle_round_mm(prof.nearest_mm);
    g_part.clear_left_mm      = obstacle_round_mm(prof.clear_left_mm);
    g_part.clear_right_mm     = obstacle_round_mm(prof.clear_right_mm);
    g_part.plan_side          = plan.side;
    g_part.leg_lateral_mm     = ((plan.leg_lateral_mm + OBS_MM_STEP - 1u)
                                 / OBS_MM_STEP) * OBS_MM_STEP;
    g_part.leg_forward_mm     = ((plan.leg_forward_mm + OBS_MM_STEP - 1u)
                                 / OBS_MM_STEP) * OBS_MM_STEP;
    g_part.scanner_state      = g_state;
    status_board_write_obstacle(&g_part);         /* board before event */

    /* casts: scan ms, outcome and the 25-bit mask all fit int32_t */
    app_log("SCAN %d ms out %d inferred 0x%x\n",
            (int32_t) ((sys_tick_now_us() - g_scan_start_us) / OBS_US_PER_MS),
            (int32_t) plan.outcome, (int32_t) g_inferred);

    if (OBS_OUT_PLAN == plan.outcome)
    {
        obstacle_enter(APP_SCAN_WATCH_SETTLE);    /* PLAN after settle */
    }
    else if (OBS_OUT_CONTINUE == plan.outcome)
    {
        (void) mission_post_event(APP_EVT_CONTINUE, 0u);      /* counted */
        obstacle_enter(APP_SCAN_PASSING);
    }
    else
    {
        (void) mission_post_event(APP_EVT_NO_PATH, 0u);       /* counted */
        drv_servo_set_deg(OBS_CENTRE_DEG);
        obstacle_enter(APP_SCAN_STOPPED);
    }
}

/**
 * obstacle_fill_gaps
 * @brief   Every unmeasured step takes the nearer (smaller) of the nearest
 *          measured steps either side and is marked inferred (review R12):
 *          a hole never splits one obstacle and is never "clear" by
 *          default.
 * @context obstacle_task
 */
static void obstacle_fill_gaps(void)
{
    uint32_t left  = 0u;
    uint32_t right = 0u;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        if (0u == (g_measured & (1u << idx)))
        {
            left  = idx;
            right = idx;
            while ((left > 0u) && (0u == (g_measured & (1u << left))))
            {
                left--;
            }
            while ((right < (APP_SCAN_STEPS - 1u))
                   && (0u == (g_measured & (1u << right))))
            {
                right++;
            }
            g_obs_scan[idx] = (g_obs_scan[left] < g_obs_scan[right])
                              ? g_obs_scan[left] : g_obs_scan[right];
            g_inferred |= (1u << idx);
        }
    }
    g_inferred &= OBS_MASK_ALL;
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
