/**
 * @file    mission.c
 * @brief   Mission queue K4, mode flags K10 and mission_task: the system
 *          statechart (Figure 5a). RUN substates live in mission_run.c.
 *          Baseline representation (natural types).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR11 (buttons), FR13, FR14, FR16, FR17, NFR10b, min-bit item 7
 */

#include "mission.h"           /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../subsystem1/telem.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/odometry.h"
#include "../subsystem3/barcode.h"
#include "../subsystem4/imu.h"
#include "cfg_mission.h"
#include "mission_run.h"
#include "status_board.h"
#include "sys_health.h"
#include "sys_tick.h"
#include "hmi.h"

/* ---- private constants and types ------------------------------------ */

/* K4 baseline message (natural types): 12 B, 16 B per entry with the
   4-byte kernel header, 16 entries = 256 B.                          */
typedef struct
{
    uint32_t type;      /* APP_EVT_* */
    int32_t  arg_a;     /* event value */
    int32_t  arg_b;     /* spare (always 0) */
} mission_evt_t;

#define MISSION_K4_ENTRIES     (16u)
#define MISSION_EVT_MSG_BYTES  (12u)   /* sizeof(mission_evt_t) */
#define MISSION_MBF_HDR_BYTES  (4u)    /* kernel header per message */
#define MISSION_MBF_BYTES      (MISSION_K4_ENTRIES                     \
                                * (MISSION_MBF_HDR_BYTES               \
                                   + MISSION_EVT_MSG_BYTES))
#define MISSION_WAKE_TMO_MS    (20)    /* backup if an R2 wake is lost  */
#define MISSION_EVTS_PER_WAKE  (8u)    /* bounds one wake's work        */
#define MISSION_STACK_WORDS    (APP_STACK_MISSION_BYTES / 8u)

#define MISSION_US_PER_MS      (1000u)
#define MISSION_US_PER_S       (1000000u)
#define MISSION_HMI_PERIOD_US  (10000u)  /* buttons polled every 10 ms */
#define MISSION_ODO_MAX_MM     (32767u)  /* K7_SUMM ODO_MM (A23)       */
#define MISSION_RUN_S_MAX      (1023u)   /* K7_SUMM RUN_S (A28)        */
#define MISSION_STATE_RUN_SHIFT (3u)     /* STATE_CHANGE: sys | run<<3 */
#define MISSION_BAR_CHAR_MASK  (0x3Fu)   /* event value bits 0-5       */
#define MISSION_BAR_CMD_SHIFT  (6u)      /* event value bits 6-7       */
#define MISSION_BAR_CMD_MASK   (0x3u)
#define MISSION_SYS_STATES     (8u)
#define MISSION_RUN_STATES     (5u)
#define MISSION_AVOID_STEPS    (7u)

/* ---- private data ---------------------------------------------------- */

static uint32_t g_mission_mbf_buf[MISSION_MBF_BYTES / 4u];
static uint64_t g_mission_stack[MISSION_STACK_WORDS];

static ID gh_mission_mbf  = 0;
static ID gh_mode_flg     = 0;
static ID gh_mission_task = 0;

/* Statistic only: several producers may update it, and a pre-empted
   update can at worst record a slightly smaller value.               */
static volatile uint32_t g_queue_high_water = 0u;

/* The mission context: written only by mission_task. */
static mission_ctx_t g_ctx;

/* Console names of the APP_FAULT_IDX_* bits (bring-up diagnosis: the
   fault mask otherwise only leaves the car over MQTT). No % in them,
   because each is passed to app_log() as the format.                 */
static char const * const g_fault_names[APP_FAULT_COUNT] =
{
    "FAULT bit 0 MOTOR_STALL (wheel not turning)\n",
    "FAULT bit 1 ENCODER (impossible encoder read)\n",
    "FAULT bit 2 IMU (3 I2C errors in a row)\n",
    "FAULT bit 3 NO_ECHO (ultrasonic silent 3 pings)\n",
    "FAULT bit 4 WIFI_LOSS (info only)\n",
    "FAULT bit 5 QUEUE_OVERFLOW (mission queue full)\n",
    "FAULT bit 6 RANGE_CLAMP (info only)\n",
    "FAULT bit 7 CAL_TIMEOUT (IMU or IR calibration > 10 s)\n",
    "FAULT bit 8 REACQUIRE_FAIL (line not found again)\n",
    "FAULT bit 9 PIO_LOAD (no PIO space for RGB)\n",
};

/* Console state names for startup diagnosis. No % conversions. */
#define MISSION_SYS_STATE_COUNT (APP_SYS_FAULT + 1u)

static char const * const g_state_names[MISSION_SYS_STATE_COUNT] =
{
    "STATE BOOT (white): waiting for task check-ins and radio init\n",
    "STATE CAL_IMU (yellow blink): keep the car still, 2 s\n",
    "STATE CAL_IR (yellow blink): sweeping over the line\n",
    "STATE READY (green): press and release RUN (GP20) to start\n",
    "STATE RUN (blue): following the line\n",
    "STATE FINISHED (green blink)\n",
    "STATE LINE_LOST_STOP (blue blink)\n",
    "STATE FAULT (red blink): check the FAULT console lines\n",
};

/* ---- private function prototypes ------------------------------------- */

static void mission_task(INT stacd, void * p_exinf);
static void mission_button_event(mission_ctx_t const * p_ctx, uint32_t press,
                                 mission_evt_in_t * p_btn);
static void mission_note_high_water(void);
static void mission_ctx_reset(mission_ctx_t * p_ctx);
static void mission_pass(mission_ctx_t * p_ctx,
                         mission_evt_in_t const * p_evt,
                         mission_evt_in_t const * p_btn,
                         status_board_t const * p_snap, uint32_t now_us);
static void mission_dispatch(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us);
static void mission_enter(mission_ctx_t * p_ctx, uint32_t sys_state,
                          status_board_t const * p_snap, uint32_t now_us);
static void mission_st_cal_imu(mission_ctx_t * p_ctx, uint32_t type,
                               uint32_t value, status_board_t const * p_snap,
                               uint32_t now_us);
static void mission_st_cal_ir(mission_ctx_t * p_ctx, uint32_t type,
                              uint32_t value, status_board_t const * p_snap,
                              uint32_t now_us);
static void mission_st_ready(mission_ctx_t * p_ctx, uint32_t type,
                             status_board_t const * p_snap, uint32_t now_us);
static void mission_st_ended(mission_ctx_t * p_ctx, uint32_t type,
                             status_board_t const * p_snap, uint32_t now_us);
static void mission_recover_done(mission_ctx_t * p_ctx, bool b_again,
                                 status_board_t const * p_snap,
                                 uint32_t now_us);
static void mission_moves(mission_ctx_t * p_ctx, uint32_t now_us);
static void mission_faults(mission_ctx_t * p_ctx,
                           status_board_t const * p_snap, uint32_t now_us);
static void mission_timers(mission_ctx_t * p_ctx, uint32_t now_us);
static void mission_clear_stale_safety(void);
static void mission_forward(mission_evt_in_t const * p_evt);
static void mission_publish(mission_ctx_t const * p_ctx,
                            status_board_mission_t const * p_before);
static void mission_part(mission_ctx_t const * p_ctx,
                         status_board_mission_t * p_part);
static void mission_summary(mission_ctx_t const * p_ctx,
                            status_board_t const * p_snap, uint32_t now_us);
static bool mission_part_differs(status_board_mission_t const * p_a,
                                 status_board_mission_t const * p_b);
#if APP_TEST
static void mission_log_ignored(mission_ctx_t const * p_ctx,
                                mission_evt_in_t const * p_evt,
                                status_board_mission_t const * p_before);
#endif

/* ---- public functions ------------------------------------------------- */

/**
 * mission_init
 * @brief   Creates K4 MBF_MISSION_EVT and K10 FLG_MODE.
 * @return  E_OK, or the first kernel error
 * @context usermain (init phase)
 */
ER mission_init(void)
{
    T_CMBF cmbf = {0};
    T_CFLG cflg = {0};
    ER     ercd = E_OK;

    cmbf.mbfatr    = TA_TFIFO | TA_USERBUF;
    cmbf.bufsz     = (SZ) MISSION_MBF_BYTES;      /* cast: kernel size */
    cmbf.maxmsz    = (INT) MISSION_EVT_MSG_BYTES; /* cast: kernel size */
    cmbf.bufptr    = g_mission_mbf_buf;
    gh_mission_mbf = tk_cre_mbf(&cmbf);

    if (gh_mission_mbf < E_OK)
    {
        ercd = (ER) gh_mission_mbf;  /* cast: negative ID is an error */
    }
    else
    {
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0u;
        gh_mode_flg  = tk_cre_flg(&cflg);
        if (gh_mode_flg < E_OK)
        {
            ercd = (ER) gh_mode_flg;  /* cast: negative ID is an error */
        }
    }

    return ercd;
}

/**
 * mission_start
 * @brief   Creates and starts mission_task on its static stack.
 * @return  E_OK, or the kernel error
 * @context usermain (start phase, called last)
 */
ER mission_start(void)
{
    ER ercd = E_OK;

    gh_mission_task = app_start_task((FP) mission_task, /* cast: kernel */
                                     APP_PRI_MISSION, g_mission_stack,
                                     APP_STACK_MISSION_BYTES,
                                     "mission_task");
    if (gh_mission_task < E_OK)
    {
        ercd = (ER) gh_mission_task;  /* cast: negative ID is an error */
    }

    return ercd;
}

/**
 * mission_post_event
 * @brief   Queues one event for mission_task without ever blocking.
 * @param   type  APP_EVT_*
 * @param   value event value (D.13)
 * @return  true if queued; false for an unknown type or a full K4
 *          (a full K4 also sets APP_FAULT_QUEUE_OVERFLOW)
 * @context any task, including the radio task (never an ISR)
 */
bool mission_post_event(uint32_t type, uint32_t value)
{
    mission_evt_t msg       = {0u, 0, 0};
    bool          b_is_sent = false;

    if (type < APP_EVT_COUNT)
    {
        msg.type  = type;
        msg.arg_a = (int32_t) value;  /* cast: values < 2048 (D.13) */
        if (E_OK == tk_snd_mbf(gh_mission_mbf, &msg,
                               (INT) MISSION_EVT_MSG_BYTES, /* cast: size */
                               TMO_POL))
        {
            b_is_sent = true;
            mission_note_high_water();
        }
        else
        {
            fault_set(APP_FAULT_QUEUE_OVERFLOW);
        }
    }

    return b_is_sent;
}

/**
 * mission_evt_barcode_value
 * @brief   Packs a BARCODE / MAP event value.
 * @param   cmd        APP_BAR_CMD_* (bits 6-7)
 * @param   char_index Code 39 index 0..42 (bits 0-5)
 * @return  the 8-bit event value
 * @context any task
 */
uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    return ((char_index & 0x3Fu) | ((cmd & 0x3u) << 6u));
}

/**
 * mission_mode_set
 * @brief   Sets mode request bits in FLG_MODE (K10).
 * @param   bits APP_MODE_* mask
 * @context mission_task only
 */
void mission_mode_set(uint32_t bits)
{
    /* set cannot fail with a valid ID */
    (void) tk_set_flg(gh_mode_flg,
                      (UINT) (bits & APP_MODE_ALL)); /* cast: kernel */
}

/**
 * mission_mode_clear
 * @brief   Clears mode request bits in FLG_MODE (K10).
 * @param   bits APP_MODE_* mask
 * @context mission_task only
 */
void mission_mode_clear(uint32_t bits)
{
    /* tk_clr_flg ANDs: pass the inverted mask */
    (void) tk_clr_flg(gh_mode_flg,
                      (UINT) ~(bits & APP_MODE_ALL)); /* cast: kernel */
}

/**
 * mission_mode_is_set
 * @brief   Tests one mode request bit.
 * @param   bit APP_MODE_* (one bit)
 * @return  true if the bit is set
 * @context any task
 */
bool mission_mode_is_set(uint32_t bit)
{
    T_RFLG rflg     = {0};
    bool   b_is_set = false;

    if (E_OK == tk_ref_flg(gh_mode_flg, &rflg))
    {
        /* cast: kernel pattern to fixed width */
        b_is_set = (0u != (((uint32_t) rflg.flgptn) & bit));
    }

    return b_is_set;
}

/**
 * mission_queue_high_water
 * @brief   Most K4 buffer bytes ever in use.
 * @return  bytes (each queued event costs 16 B in the baseline)
 * @context any task
 */
uint32_t mission_queue_high_water(void)
{
    return g_queue_high_water;
}

/* ---- private functions ------------------------------------------------ */

/**
 * mission_task
 * @brief   The mission loop: one pass at least every 10 ms (B5.2.6).
 * @param   stacd   unused (kernel start code)
 * @param   p_exinf unused
 * @context mission_task (priority 16)
 */
static void mission_task(INT stacd, void * p_exinf)
{
    mission_evt_t    msg      = {0u, 0, 0};
    mission_evt_in_t evt      = {false, 0u, 0u};
    mission_evt_in_t btn      = {false, 0u, 0u};
    status_board_t   snap     = {0};
    INT              size     = 0;
    uint32_t         now_us   = 0u;
    uint32_t         hmi_us   = 0u;
    uint32_t         wdt_log_us = 0u;
    uint32_t         n_pass   = 0u;
    uint32_t         press    = HMI_NONE;
    bool             b_is_rgb_ok = false;

    (void) stacd;    /* unused: kernel start code */
    (void) p_exinf;  /* unused: no extended info  */

    b_is_rgb_ok = hmi_init();
    if (!b_is_rgb_ok)
    {
        app_log("RGB: no PIO space, running without RGB\n", 0, 0, 0);
    }
    mission_ctx_reset(&g_ctx);
    status_board_read(&snap);
    now_us = sys_tick_now_us();
    hmi_us = now_us;
    mission_enter(&g_ctx, APP_SYS_BOOT, &snap, now_us);
    sys_tick_register_feeder(tk_get_tid());

    for (;;)
    {
        /* 1. Woken by R2 every 10 ms (sys_tick_register_feeder). Waiting
              in tk_rcv_mbf with a 10 ms timeout made passes 10-20 ms apart
              on the 10 ms kernel tick, leaving ~10 ms of the 30 ms
              watchdog window. Then up to MISSION_EVTS_PER_WAKE queued
              events, one pass each, never waiting; a wake with no event
              is one pass. Events now wait up to 10 ms (safety stops do
              not go through K4: they act on FLG_SAFETY directly). */
        (void) tk_slp_tsk(MISSION_WAKE_TMO_MS);
        n_pass = 0u;
        do
        {
            size            = tk_rcv_mbf(gh_mission_mbf, &msg, TMO_POL);
            n_pass++;
            evt.b_has_event = (size > 0);
            evt.type        = msg.type;
            evt.value       = (uint32_t) msg.arg_a; /* cast: 0..2047 */

            /* 2. buttons every 10 ms */
            now_us          = sys_tick_now_us();
            btn.b_has_event = false;
            if ((now_us - hmi_us) >= MISSION_HMI_PERIOD_US)
            {
                hmi_us = now_us;
                press  = hmi_poll();
                mission_button_event(&g_ctx, press, &btn);
            }

            status_board_read(&snap);
            mission_pass(&g_ctx, &evt, &btn, &snap, now_us);

            /* 7. LED, buzzer and watchdog */
            hmi_show(g_ctx.sys_state, now_us);
            if (!b_is_rgb_ok)
            {
                fault_set(APP_FAULT_PIO_LOAD); /* every pass (D.13) */
            }
            imu_diag_service();  /* bring-up: one INT1 report line, if any */
            sys_health_service();
#if (MISSION_WDT_LOG_MS != 0u)
            /* Bring-up diagnosis: watchdog margin (window 30 ms) */
            if ((now_us - wdt_log_us)
                >= (MISSION_WDT_LOG_MS * MISSION_US_PER_MS))
            {
                wdt_log_us = now_us;
                /* cast: a gap is far below INT32_MAX */
                app_log("WDT longest feed gap %d us in the last 2 s"
                        " (watchdog fires at 30000)\n",
                        (int32_t) sys_health_feed_gap_max_us(true), 0, 0);
            }
#endif
        } while ((size > 0) && (n_pass < MISSION_EVTS_PER_WAKE));
    }
}

/**
 * mission_button_event
 * @brief   RUN short press -> START in READY or STOP in RUN; RESET held
 *          2 s -> RESET (any state).
 * @param   press HMI_* from hmi_poll()
 * @param   p_btn event made from it (b_has_event false if none)
 * @context mission_task
 */
static void mission_button_event(mission_ctx_t const * p_ctx, uint32_t press,
                                 mission_evt_in_t * p_btn)
{
    p_btn->b_has_event = false;
    p_btn->value       = 0u;

    if (HMI_RESET_HOLD == press)
    {
        p_btn->b_has_event = true;
        p_btn->type        = APP_EVT_RESET;
    }
    else if ((HMI_RUN_PRESS == press) && (APP_SYS_READY == p_ctx->sys_state))
    {
        p_btn->b_has_event = true;
        p_btn->type        = APP_EVT_START;
    }
    else if ((HMI_RUN_PRESS == press) && (APP_SYS_RUN == p_ctx->sys_state))
    {
        p_btn->b_has_event = true;
        p_btn->type        = APP_EVT_STOP;
    }
    else
    {
        /* no button event */
    }
}

/**
 * mission_ctx_reset
 * @brief   Puts the context in its power-on state.
 * @context mission_task (start), test build
 */
static void mission_ctx_reset(mission_ctx_t * p_ctx)
{
    mission_ctx_t const blank =
    {
        .sys_state   = APP_SYS_BOOT,
        .run_state   = APP_RUN_FOLLOW,
        .avoid_step  = APP_AVOID_WAIT_PLAN,
        .end_reason  = APP_END_STOP,
        .armed_cmd   = APP_BAR_CMD_LEFT,
        .sys_request = MISSION_REQ_NONE,
        .side        = APP_SIDE_LEFT,
    };

    *p_ctx = blank;
}

/**
 * mission_pass
 * @brief   Steps 3-6 of B5.2.6 for one pass, plus the tracked-move rules:
 *          a MOVE_DONE counts only if it carries the awaited tag; a
 *          refused command is retried once; a missed deadline or a second
 *          refusal ends in FAULT.
 * @param   p_ctx  mission context
 * @param   p_evt  K4 event of this pass (may be empty)
 * @param   p_btn  button event of this pass (may be empty)
 * @param   p_snap status board snapshot
 * @param   now_us this pass
 * @context mission_task
 */
static void mission_pass(mission_ctx_t * p_ctx,
                         mission_evt_in_t const * p_evt,
                         mission_evt_in_t const * p_btn,
                         status_board_t const * p_snap, uint32_t now_us)
{
    status_board_mission_t before = {0u, 0u, 0u, 0u, false, 0u};
    mission_evt_in_t       evt    = *p_evt;

    if (evt.b_has_event && (APP_EVT_MOVE_DONE == evt.type))
    {
        if (p_ctx->b_move_pending && (evt.value == p_ctx->wait_tag))
        {
            p_ctx->b_move_pending = false;      /* the awaited command */
        }
        else
        {
            evt.b_has_event = false;            /* stale: cancelled move */
        }
    }

    mission_part(p_ctx, &before);
    mission_timers(p_ctx, now_us);
    mission_moves(p_ctx, now_us);
    mission_faults(p_ctx, p_snap, now_us);              /* 3 */
    mission_dispatch(p_ctx, &evt, p_snap, now_us);      /* 4 */
    if (p_btn->b_has_event)
    {
        mission_dispatch(p_ctx, p_btn, p_snap, now_us);
    }
    if (p_ctx->b_motion_failed && (APP_SYS_FAULT != p_ctx->sys_state))
    {
        /* Bring-up diagnosis: this FAULT sets no fault bit of its own.
           casts: motion state 0..7 and safety bits 0..7 fit int32_t   */
        app_log("MOVE FAILED (no MOVE_DONE in time, or refused twice):"
                " motion state %d (6 = ESTOP), safety bits %d"
                " (1 = ESTOP, 2 = IMPACT)\n",
                (int32_t) status_board_motion_state(p_snap),
                (int32_t) safety_get(), 0);
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    p_ctx->b_motion_failed = false;
    mission_publish(p_ctx, &before);                    /* 5 */
    mission_forward(p_evt);                             /* 6 */
#if APP_TEST
    mission_log_ignored(p_ctx, p_evt, &before);
#endif
}

/**
 * mission_dispatch
 * @brief   Hands the event (or "no event") to the current state's handler.
 *          Unknown states go to FAULT.
 * @context mission_task (and the S5-U05 test matrix)
 */
static void mission_dispatch(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const type  = p_evt->b_has_event ? p_evt->type : APP_EVT_COUNT;
    uint32_t const value = p_evt->value;

    switch (p_ctx->sys_state)
    {
        case APP_SYS_BOOT:
            if (sys_health_is_started())       /* startup window passed */
            {
                mission_enter(p_ctx, APP_SYS_CAL_IMU, p_snap, now_us);
            }
            break;
        case APP_SYS_CAL_IMU:
            mission_st_cal_imu(p_ctx, type, value, p_snap, now_us);
            break;
        case APP_SYS_CAL_IR:
            mission_st_cal_ir(p_ctx, type, value, p_snap, now_us);
            break;
        case APP_SYS_READY:
            mission_st_ready(p_ctx, type, p_snap, now_us);
            break;
        case APP_SYS_RUN:
            p_ctx->sys_request = MISSION_REQ_NONE;
            mission_run_handle(p_ctx, p_evt, p_snap, now_us);
            if (MISSION_REQ_NONE != p_ctx->sys_request)
            {
                mission_enter(p_ctx, p_ctx->sys_request, p_snap, now_us);
                p_ctx->sys_request = MISSION_REQ_NONE;
            }
            break;
        case APP_SYS_FINISHED:
        case APP_SYS_LINE_LOST_STOP:
        case APP_SYS_FAULT:
            mission_st_ended(p_ctx, type, p_snap, now_us);
            break;
        default:
            mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
            break;
    }
}

/**
 * mission_enter
 * @brief   Changes the system state and runs its entry action (B5.2.6).
 * @param   sys_state APP_SYS_*
 * @context mission_task
 */
static void mission_enter(mission_ctx_t * p_ctx, uint32_t sys_state,
                          status_board_t const * p_snap, uint32_t now_us)
{
    p_ctx->sys_state      = sys_state;
    p_ctx->substep        = 0u;
    p_ctx->b_run_starting = false;
    p_ctx->b_recovering   = false;

    if (sys_state < MISSION_SYS_STATE_COUNT)
    {
        app_log(g_state_names[sys_state], 0, 0, 0);
    }

    switch (sys_state)
    {
        case APP_SYS_BOOT:
            mission_stop_motion(p_ctx);           /* motors braked */
            if (!p_ctx->b_wdt_enabled)
            {
                sys_health_enable_watchdog();     /* first boot only */
                p_ctx->b_wdt_enabled = true;
            }
            mission_mode_clear(APP_MODE_ALL);
            p_ctx->b_resume_pending = false;      /* RESUME was cleared */
            mission_run_resume_pulse(p_ctx, now_us);
            p_ctx->b_imu_valid = false;
            p_ctx->b_ir_valid  = false;
            break;
        case APP_SYS_CAL_IMU:
            mission_mode_set(APP_MODE_CAL_IMU);
            p_ctx->cal_start_us = now_us;
            break;
        case APP_SYS_CAL_IR:
            mission_clear_stale_safety();         /* before MOTOR_EN */
            mission_mode_set(APP_MODE_CAL_IR | APP_MODE_MOTOR_EN);
            p_ctx->cal_start_us   = now_us;
            p_ctx->sweep_start_us = now_us;
            (void) mission_send_move(p_ctx, MISSION_MV_LEFT,
                                     MISSION_CAL_SWEEP_DEG, now_us);
            break;
        case APP_SYS_READY:
            p_ctx->b_is_warning = false;
            hmi_set_warning(false);
            break;
        case APP_SYS_RUN:
            mission_run_enter(p_ctx, p_snap, now_us);
            break;
        case APP_SYS_FINISHED:
        case APP_SYS_LINE_LOST_STOP:
            mission_stop_motion(p_ctx);
            mission_mode_clear(APP_MODE_MOTOR_EN);  /* every RUN exit */
            mission_summary(p_ctx, p_snap, now_us);
            break;
        case APP_SYS_FAULT:
            p_ctx->end_reason = APP_END_FAULT;
            mission_stop_motion(p_ctx);
            mission_mode_clear(APP_MODE_MOTOR_EN);
            safety_set(SAFETY_FAULT);
            mission_summary(p_ctx, p_snap, now_us);
            break;
        default:
            p_ctx->sys_state = APP_SYS_FAULT;     /* unknown: fail safe */
            mission_stop_motion(p_ctx);
            mission_mode_clear(APP_MODE_MOTOR_EN);
            safety_set(SAFETY_FAULT);
            break;
    }
}

/**
 * mission_st_cal_imu
 * @brief   CAL_IMU: wait for CAL_DONE(IMU); 10 s -> CAL_TIMEOUT fault.
 * @context mission_task
 */
static void mission_st_cal_imu(mission_ctx_t * p_ctx, uint32_t type,
                               uint32_t value, status_board_t const * p_snap,
                               uint32_t now_us)
{
    uint32_t const limit_us = MISSION_CAL_TIMEOUT_MS * MISSION_US_PER_MS;

    if ((APP_EVT_CAL_DONE == type) && (APP_CAL_SRC_IMU == value))
    {
        p_ctx->b_imu_valid = true;
        mission_mode_clear(APP_MODE_CAL_IMU);
        mission_enter(p_ctx, APP_SYS_CAL_IR, p_snap, now_us);
    }
    else if ((now_us - p_ctx->cal_start_us) >= limit_us)
    {
        mission_mode_clear(APP_MODE_CAL_IMU);
        fault_set(APP_FAULT_CAL_TIMEOUT);
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    else
    {
        /* still calibrating */
    }
}

/**
 * mission_st_cal_ir
 * @brief   CAL_IR: sweep left 30, right 60, left 30 over the line (each
 *          leg starts on the tagged MOVE_DONE), then clear CAL_IR and
 *          MOTOR_EN; CAL_DONE(IR) -> READY. Sweep over 3 s -> stop;
 *          10 s in total -> CAL_TIMEOUT fault.
 * @context mission_task
 */
static void mission_st_cal_ir(mission_ctx_t * p_ctx, uint32_t type,
                              uint32_t value, status_board_t const * p_snap,
                              uint32_t now_us)
{
    uint32_t const sweep_us = MISSION_CAL_SWEEP_MAX_MS * MISSION_US_PER_MS;
    uint32_t const limit_us = MISSION_CAL_TIMEOUT_MS * MISSION_US_PER_MS;
    uint32_t const cal_ir   = APP_MODE_CAL_IR | APP_MODE_MOTOR_EN;

    if ((APP_EVT_CAL_DONE == type) && (APP_CAL_SRC_IR == value))
    {
        mission_stop_motion(p_ctx);
        mission_mode_clear(cal_ir);
        p_ctx->b_ir_valid = true;
        if (p_ctx->b_imu_valid)
        {
            mission_enter(p_ctx, APP_SYS_READY, p_snap, now_us);
        }
        else
        {
            mission_enter(p_ctx, APP_SYS_BOOT, p_snap, now_us);
        }
    }
    else if (APP_EVT_MOVE_DONE == type)
    {
        if (0u == p_ctx->substep)
        {
            (void) mission_send_move(p_ctx, MISSION_MV_RIGHT,
                                     2 * MISSION_CAL_SWEEP_DEG, now_us);
        }
        else if (1u == p_ctx->substep)
        {
            (void) mission_send_move(p_ctx, MISSION_MV_LEFT,
                                     MISSION_CAL_SWEEP_DEG, now_us);
        }
        else
        {
            mission_mode_clear(cal_ir);           /* sweep done */
        }
        if (p_ctx->substep < 3u)
        {
            p_ctx->substep++;
        }
    }
    else
    {
        /* waiting for the sweep or the result */
    }

    if (APP_SYS_CAL_IR != p_ctx->sys_state)
    {
        /* left the state above */
    }
    else if ((now_us - p_ctx->cal_start_us) >= limit_us)
    {
        mission_mode_clear(cal_ir);
        mission_stop_motion(p_ctx);
        fault_set(APP_FAULT_CAL_TIMEOUT);
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    else if ((p_ctx->substep < 3u)
             && ((now_us - p_ctx->sweep_start_us) >= sweep_us))
    {
        mission_stop_motion(p_ctx);               /* sweep took > 3 s */
        mission_mode_clear(cal_ir);
        p_ctx->substep = 3u;
    }
    else
    {
        /* in time */
    }
}

/**
 * mission_st_ready
 * @brief   READY: START -> 1 s warning (buzzer + LED) -> run start: set
 *          clear a stale IMPACT, set RUN_START, ZERO the odometry and wait
 *          for its tagged MOVE_DONE (deadline 200 ms -> FAULT; an IMPACT
 *          from here on -> FAULT IMPACT), hold RUN_START >= 30 ms so line,
 *          barcode and IMU reset their per-run state, then clear it, set
 *          MOTOR_EN and enter RUN with run_start_path = 0.
 * @context mission_task
 */
static void mission_st_ready(mission_ctx_t * p_ctx, uint32_t type,
                             status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const warn_us = MISSION_START_WARN_MS * MISSION_US_PER_MS;
    uint32_t const hold_us = MISSION_RUN_START_HOLD_MS * MISSION_US_PER_MS;

    if ((APP_EVT_START == type) && (!p_ctx->b_is_warning)
        && (!p_ctx->b_run_starting))
    {
        p_ctx->b_is_warning  = true;
        p_ctx->warn_start_us = now_us;
        hmi_set_warning(true);
    }
    else if (p_ctx->b_is_warning
             && ((now_us - p_ctx->warn_start_us) >= warn_us))
    {
        p_ctx->b_is_warning     = false;
        hmi_set_warning(false);
        p_ctx->b_run_starting   = true;
        p_ctx->b_zero_done      = false;
        p_ctx->run_start_set_us = now_us;
        /* Before the ZERO: with IMPACT set, motion_task sits in ESTOP and
           only latches the ZERO, so no MOVE_DONE would come back and the
           200 ms deadline would end the start in FAULT (S5-U33). */
        mission_clear_stale_safety();
        mission_mode_set(APP_MODE_RUN_START);
        (void) mission_send_move(p_ctx, MISSION_MV_ZERO, 0, now_us);
    }
    else if (p_ctx->b_run_starting)
    {
        if (APP_EVT_MOVE_DONE == type)
        {
            p_ctx->b_zero_done = true;            /* tagged ack arrived */
        }
        if (p_ctx->b_zero_done
            && ((now_us - p_ctx->run_start_set_us) >= hold_us))
        {
            mission_mode_clear(APP_MODE_RUN_START);
            p_ctx->run_start_us   = now_us;
            p_ctx->run_start_path = 0u;           /* odometry rebased */
            p_ctx->b_has_run      = true;
            p_ctx->end_reason     = APP_END_STOP;
            mission_mode_set(APP_MODE_MOTOR_EN);
            mission_enter(p_ctx, APP_SYS_RUN, p_snap, now_us);
        }
    }
    else
    {
        /* waiting for START */
    }
}

/**
 * mission_st_ended
 * @brief   FINISHED, LINE_LOST_STOP and FAULT: RESET starts the common
 *          recovery (review V13-02): stop, clear faults (not Wi-Fi, the
 *          bridge owns it), safety bits and modes, pulse RESUME, then wait
 *          100 ms as a mission timer (the feeder never sleeps). Anything
 *          set again -> FAULT; else FAULT goes to BOOT (recalibration),
 *          FINISHED / LINE_LOST_STOP go to READY.
 * @context mission_task
 */
static void mission_st_ended(mission_ctx_t * p_ctx, uint32_t type,
                             status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const wait_us = MISSION_RECOVER_MS * MISSION_US_PER_MS;
    bool           b_again = false;

    if (p_ctx->b_recovering)
    {
        if ((now_us - p_ctx->recover_start_us) >= wait_us)
        {
            p_ctx->b_recovering = false;
            b_again = (0u != (fault_get() & APP_FAULT_STOP_MASK))
                      || (0u != safety_get());
            mission_recover_done(p_ctx, b_again, p_snap, now_us);
        }
    }
    else if (APP_EVT_RESET == type)
    {
        mission_stop_motion(p_ctx);
        fault_clear(APP_FAULT_ALL & ~APP_FAULT_WIFI_LOSS);
        safety_clear(SAFETY_ALL);
        mission_mode_clear(APP_MODE_ALL);
        p_ctx->b_resume_pending = false;          /* RESUME was cleared */
        mission_run_resume_pulse(p_ctx, now_us);
        p_ctx->b_recovering     = true;
        p_ctx->recover_start_us = now_us;
    }
    else
    {
        /* stay until RESET */
    }
}

/**
 * mission_recover_done
 * @brief   End of the 100 ms recovery wait.
 * @param   b_again true if a stop fault or a safety bit came back
 * @context mission_task
 */
static void mission_recover_done(mission_ctx_t * p_ctx, bool b_again,
                                 status_board_t const * p_snap,
                                 uint32_t now_us)
{
    if (b_again && (APP_SYS_FAULT == p_ctx->sys_state))
    {
        mission_stop_motion(p_ctx);               /* stay in FAULT */
        safety_set(SAFETY_FAULT);
    }
    else if (b_again)
    {
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    else if (APP_SYS_FAULT == p_ctx->sys_state)
    {
        p_ctx->b_has_run = false;
        mission_enter(p_ctx, APP_SYS_BOOT, p_snap, now_us);  /* recalibrate */
    }
    else
    {
        p_ctx->b_has_run = false;
        mission_enter(p_ctx, APP_SYS_READY, p_snap, now_us);
    }
}

/**
 * mission_faults
 * @brief   Reports newly set fault bits to telemetry; enters FAULT on any
 *          stop fault, and on an IMPACT during CAL_IR, the run start or RUN
 *          (review R07). In the run start the IMPACT was already cleared
 *          once (mission_clear_stale_safety), so a new one is real.
 * @context mission_task
 */
static void mission_faults(mission_ctx_t * p_ctx,
                           status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const faults = fault_get();
    uint32_t const fresh  = faults & ~p_ctx->last_faults;
    bool const     b_move = (APP_SYS_CAL_IR == p_ctx->sys_state)
                            || (APP_SYS_RUN == p_ctx->sys_state)
                            || ((APP_SYS_READY == p_ctx->sys_state)
                                && p_ctx->b_run_starting);

    for (uint32_t idx = 0u; idx < APP_FAULT_COUNT; idx++)
    {
        if (0u != (fresh & (1u << idx)))
        {
            (void) telem_post_event(APP_EVT_SENSOR_FAULT, idx); /* counted */
            app_log(g_fault_names[idx], 0, 0, 0);
        }
    }
    p_ctx->last_faults = faults;

    if ((0u != (faults & APP_FAULT_STOP_MASK))
        && (APP_SYS_FAULT != p_ctx->sys_state))
    {
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    else if (b_move && (0u != (safety_get() & SAFETY_IMPACT)))
    {
        app_log("FAULT IMPACT (IMU INT1 while moving or starting)\n",
                0, 0, 0);
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    else
    {
        /* no fault transition */
    }
}

/**
 * mission_moves
 * @brief   Retries a refused command once; ends in FAULT if the awaited
 *          MOVE_DONE misses its deadline.
 * @context mission_task
 */
static void mission_moves(mission_ctx_t * p_ctx, uint32_t now_us)
{
    if (p_ctx->b_retry_pending)
    {
        (void) mission_send_move(p_ctx, p_ctx->retry_kind,
                                 p_ctx->retry_param, now_us);
    }
    else if (p_ctx->b_move_pending
             && ((now_us - p_ctx->move_start_us)
                 >= (p_ctx->move_limit_ms * MISSION_US_PER_MS)))
    {
        mission_stop_motion(p_ctx);               /* deadline passed */
        p_ctx->b_motion_failed = true;
    }
    else
    {
        /* nothing outstanding, or still in time */
    }
}

/**
 * mission_send_move
 * @brief   Sends MOVE, TURN or ZERO (see mission_run.h).
 * @param   kind   MISSION_MV_*
 * @param   param  mm or degrees (ignored for ZERO)
 * @param   now_us this pass
 * @return  true if motion accepted the command
 * @context mission_task
 */
bool mission_send_move(mission_ctx_t * p_ctx, uint32_t kind, int32_t param,
                       uint32_t now_us)
{
    bool b_is_sent = false;

    switch (kind)
    {
        case MISSION_MV_FWD:
            b_is_sent = motion_move_forward(param);
            break;
        case MISSION_MV_BACK:
            b_is_sent = motion_move_backward(param);
            break;
        case MISSION_MV_LEFT:
            b_is_sent = motion_turn_left(param);
            break;
        case MISSION_MV_RIGHT:
            b_is_sent = motion_turn_right(param);
            break;
        default:
            b_is_sent = motion_reset_odometry();
            break;
    }

    if (b_is_sent)
    {
        p_ctx->wait_tag        = motion_last_tag();
        p_ctx->b_move_pending  = true;
        p_ctx->move_start_us   = now_us;
        p_ctx->move_limit_ms   = mission_move_deadline_ms(kind, param);
        p_ctx->b_retry_pending = false;
    }
    else if (!p_ctx->b_retry_pending)
    {
        p_ctx->b_retry_pending = true;            /* retry next pass */
        p_ctx->retry_kind      = kind;
        p_ctx->retry_param     = param;
    }
    else
    {
        p_ctx->b_retry_pending = false;           /* refused twice */
        fault_set(APP_FAULT_MOTOR_STALL);
        p_ctx->b_motion_failed = true;
    }

    return b_is_sent;
}

/**
 * mission_clear_stale_safety
 * @brief   Clears an IMPACT latched while the motors were not enabled,
 *          before the motors are used: on CAL_IR entry (before MOTOR_EN)
 *          and at the end of the READY warning (before the run-start ZERO,
 *          which motion_task only latches while it is in ESTOP). A bump
 *          while the car is handled or the RUN button is pressed is not a
 *          collision, and nothing else clears IMPACT before RUN (on-car
 *          log 2026-10-29, tests S5-U32 and S5-U33). An IMPACT after this
 *          point, in CAL_IR, the run start or RUN, is a FAULT
 *          (mission_faults).
 *          Only the IMPACT bit is cleared. ESTOP is NOT cleared:
 *          obstacle_task does not set it while MOTOR_EN is clear, and an
 *          ESTOP that is set means a hazard that only obstacle_task or a
 *          RESET may release. Clearing the bit does not re-arm the
 *          LSM303D: imu_task reads IG_SRC1 (outside CAL_IMU and RUN_START)
 *          to release the latched INT1. mission_task stays the only task
 *          that clears safety bits.
 * @context mission_task
 */
static void mission_clear_stale_safety(void)
{
    if (0u != (safety_get() & SAFETY_IMPACT))
    {
        /* Bring-up diagnosis */
        app_log("SAFETY IMPACT latched before the motors were enabled:"
                " cleared\n", 0, 0, 0);
        safety_clear(SAFETY_IMPACT);
    }
}

/**
 * mission_stop_motion
 * @brief   motion_stop() (always acts) and forget any outstanding or
 *          retried command, so a later MOVE_DONE of it is ignored.
 * @context mission_task
 */
void mission_stop_motion(mission_ctx_t * p_ctx)
{
    (void) motion_stop();                         /* always acts */
    p_ctx->b_move_pending  = false;
    p_ctx->b_retry_pending = false;
}

/**
 * mission_move_deadline_ms
 * @brief   Completion deadline from the command itself (review V14-01):
 *          distance (or wheel arc for a turn) / speed x 1.5 + 1 s; ZERO
 *          200 ms. Example: 800 mm at 150 mm/s -> about 9 000 ms.
 * @param   kind  MISSION_MV_*
 * @param   param mm or degrees (<= 0 treated as 0)
 * @return  ms
 * @context mission_task (event time: division allowed)
 */
uint32_t mission_move_deadline_ms(uint32_t kind, int32_t param)
{
    /* casts: positive speed constants and a non-negative parameter */
    uint32_t const amount = (param > 0) ? (uint32_t) param : 0u;
    uint32_t       dist   = amount;
    uint32_t       speed  = (uint32_t) MOTION_MOVE_SPEED_MM_S;
    uint32_t       limit  = MISSION_ACK_TIMEOUT_MS;

    if (MISSION_MV_ZERO != kind)
    {
        if (MISSION_MV_BACK == kind)
        {
            speed = (uint32_t) MOTION_REVERSE_SPEED_MM_S;
        }
        else if ((MISSION_MV_LEFT == kind) || (MISSION_MV_RIGHT == kind))
        {
            dist  = (amount * MISSION_TURN_UM_PER_DEG) / 1000u;
            speed = (uint32_t) MOTION_TURN_SPEED_MM_S;
        }
        else
        {
            /* forward: move speed */
        }
        limit = (((dist * 1000u) / speed) * 3u) / 2u
                + MISSION_MOVE_MARGIN_MS;
    }

    return limit;
}

/**
 * mission_timers
 * @brief   Ends the RESUME pulse 150 ms after it started.
 * @context mission_task
 */
static void mission_timers(mission_ctx_t * p_ctx, uint32_t now_us)
{
    uint32_t const pulse_us = MISSION_RESUME_PULSE_MS * MISSION_US_PER_MS;

    if (p_ctx->b_resume_pending
        && ((now_us - p_ctx->resume_start_us) >= pulse_us))
    {
        mission_mode_clear(APP_MODE_RESUME);
        p_ctx->b_resume_pending = false;
    }
}

/**
 * mission_forward
 * @brief   Step 6: copies selected events to telemetry; MAP remaps a
 *          barcode character (optional on-day remapping).
 * @context mission_task
 */
static void mission_forward(mission_evt_in_t const * p_evt)
{
    if (p_evt->b_has_event)
    {
        switch (p_evt->type)
        {
            case APP_EVT_BARCODE:
            case APP_EVT_UNMAPPED:
            case APP_EVT_INVALID:
            case APP_EVT_HUMP:
            case APP_EVT_MOTION_CLASS:
            case APP_EVT_STALL:
            case APP_EVT_SIDE_TOO_CLOSE:
                (void) telem_post_event(p_evt->type, p_evt->value); /* cnt */
                break;
            case APP_EVT_MAP:
                barcode_map_set(p_evt->value & MISSION_BAR_CHAR_MASK,
                                (p_evt->value >> MISSION_BAR_CMD_SHIFT)
                                & MISSION_BAR_CMD_MASK);
                break;
            default:
                break;                    /* not forwarded */
        }
    }
}

/**
 * mission_publish
 * @brief   Step 5: on any change, writes the mission part of the board;
 *          a state or substate change is also sent to telemetry.
 * @param   p_before mission part at the start of the pass
 * @context mission_task
 */
static void mission_publish(mission_ctx_t const * p_ctx,
                            status_board_mission_t const * p_before)
{
    status_board_mission_t now = {0u, 0u, 0u, 0u, false, 0u};

    mission_part(p_ctx, &now);
    if (mission_part_differs(&now, p_before))
    {
        status_board_write_mission(&now);
    }
    if ((now.sys_state != p_before->sys_state)
        || (now.run_state != p_before->run_state))
    {
        (void) telem_post_event(APP_EVT_STATE_CHANGE,
                                now.sys_state
                                | (now.run_state
                                   << MISSION_STATE_RUN_SHIFT)); /* counted */
    }
}

/**
 * mission_part
 * @brief   Builds the board's mission part from the context.
 * @context mission_task
 */
static void mission_part(mission_ctx_t const * p_ctx,
                         status_board_mission_t * p_part)
{
    p_part->sys_state  = p_ctx->sys_state;
    p_part->run_state  = p_ctx->run_state;
    p_part->avoid_step = p_ctx->avoid_step;
    p_part->end_reason = p_ctx->end_reason;
    p_part->b_is_armed = p_ctx->b_is_armed;
    p_part->armed_cmd  = p_ctx->armed_cmd;
}

/**
 * mission_part_differs
 * @brief   Field-by-field comparison (no memcmp over padding).
 * @return  true if any field differs
 * @context mission_task
 */
static bool mission_part_differs(status_board_mission_t const * p_a,
                                 status_board_mission_t const * p_b)
{
    return (p_a->sys_state != p_b->sys_state)
           || (p_a->run_state != p_b->run_state)
           || (p_a->avoid_step != p_b->avoid_step)
           || (p_a->end_reason != p_b->end_reason)
           || (p_a->b_is_armed != p_b->b_is_armed)
           || (p_a->armed_cmd != p_b->armed_cmd);
}

/**
 * mission_summary
 * @brief   End-of-run summary to telemetry: end reason, peak hump,
 *          distance travelled this run, faults, run time.
 * @context mission_task (event time: division allowed)
 */
static void mission_summary(mission_ctx_t const * p_ctx,
                            status_board_t const * p_snap, uint32_t now_us)
{
    telem_bin_summary_t summ = {0u, 0u, 0u, 0u, 0u};
    uint32_t            odo  = 0u;
    uint32_t            secs = 0u;

    if (p_ctx->b_has_run)
    {
        odo  = odometry_counts_to_mm(status_board_path(p_snap)
                                     - p_ctx->run_start_path);
        secs = (now_us - p_ctx->run_start_us) / MISSION_US_PER_S;
    }
    summ.end_reason   = p_ctx->end_reason;
    summ.hump_peak_mm = status_board_hump_peak(p_snap);
    summ.odometry_mm  = (odo > MISSION_ODO_MAX_MM) ? MISSION_ODO_MAX_MM : odo;
    summ.faults       = fault_get();
    summ.run_time_s   = (secs > MISSION_RUN_S_MAX) ? MISSION_RUN_S_MAX : secs;
    (void) telem_post_summary(&summ);    /* a full K7 is counted there */
}

/**
 * mission_note_high_water
 * @brief   Updates the K4 high-water mark after a successful send.
 * @context any posting task
 */
static void mission_note_high_water(void)
{
    T_RMBF   rmbf     = {0};
    uint32_t used     = 0u;

    if (E_OK == tk_ref_mbf(gh_mission_mbf, &rmbf))
    {
        used = MISSION_MBF_BYTES
               - (uint32_t) rmbf.frbufsz;  /* cast: 0..bufsz */
        if (used > g_queue_high_water)
        {
            g_queue_high_water = used;
        }
    }
}

#if APP_TEST
/**
 * mission_log_ignored
 * @brief   Test builds: logs a K4 event that changed nothing and is not a
 *          telemetry-only or forwarded type ("unknown events are ignored,
 *          logged in test builds").
 * @context mission_task (TEST=1)
 */
static void mission_log_ignored(mission_ctx_t const * p_ctx,
                                mission_evt_in_t const * p_evt,
                                status_board_mission_t const * p_before)
{
    status_board_mission_t now = {0u, 0u, 0u, 0u, false, 0u};

    mission_part(p_ctx, &now);
    if (p_evt->b_has_event && (!mission_part_differs(&now, p_before))
        && (APP_EVT_SENSOR_FAULT > p_evt->type) && (!p_ctx->b_is_quiet))
    {
        /* casts: both < 32 */
        app_log("EVT %d ignored in %d\n", (int32_t) p_evt->type,
                (int32_t) p_ctx->sys_state, 0);
    }
}
#endif /* APP_TEST */

#if defined(MISSION_HOST_TEST)
/**
 * mission_test_matrix
 * @brief   S5-U05: feeds every event type to every state (RUN: every
 *          substate; AVOID: every step) on a scratch copy of the context
 *          and checks the result is a defined state. Host test only
 *          (MISSION_HOST_TEST): the side effects (motion commands, mode
 *          bits, faults, watchdog) are real on the target, so S5-U05
 *          never runs there (fix brief, Fix 2). The host harness gives
 *          each case fresh mocks through mission_host_case_reset().
 * @param   p_cases number of combinations tried
 * @return  number of combinations with an undefined outcome
 * @context host test (sim_mission_host.c)
 */
uint32_t mission_test_matrix(uint32_t * p_cases)
{
    mission_ctx_t    ctx    = {0};
    mission_evt_in_t evt    = {true, 0u, 0u};
    status_board_t   snap   = {0};
    uint32_t const   now_us = sys_tick_now_us();
    uint32_t         bad    = 0u;
    uint32_t         cases  = 0u;
    uint32_t         run_n  = 1u;
    uint32_t         av_n   = 1u;

    status_board_read(&snap);
    for (uint32_t sys = 0u; sys < MISSION_SYS_STATES; sys++)
    {
        run_n = (APP_SYS_RUN == sys) ? MISSION_RUN_STATES : 1u;
        for (uint32_t run = 0u; run < run_n; run++)
        {
            av_n = ((APP_SYS_RUN == sys) && (APP_RUN_AVOID == run))
                   ? MISSION_AVOID_STEPS : 1u;
            for (uint32_t step = 0u; step < av_n; step++)
            {
                for (uint32_t type = 0u; type < APP_EVT_COUNT; type++)
                {
                    mission_host_case_reset();
                    mission_ctx_reset(&ctx);
                    ctx.b_is_quiet  = true;
                    ctx.sys_state   = sys;
                    ctx.run_state   = run;
                    ctx.avoid_step  = step;
                    ctx.last_faults = fault_get();
                    evt.type        = type;
                    mission_dispatch(&ctx, &evt, &snap, now_us);
                    cases++;
                    if ((ctx.sys_state >= MISSION_SYS_STATES)
                        || (ctx.run_state >= MISSION_RUN_STATES)
                        || (ctx.avoid_step >= MISSION_AVOID_STEPS))
                    {
                        bad++;
                    }
                }
            }
        }
    }

    *p_cases = cases;

    return bad;
}
#endif /* MISSION_HOST_TEST */

/*** end of file ***/
