/**
 * @file    motion_task.c
 * @brief   motion_task, the Motion Executor (Figure 6.1): closed-loop wheel
 *          speed, line following, move by distance, turn by angle, stall
 *          and encoder checks, and the safety interlock, every 10 ms.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR13, FR15, FR17, NFR2, NFR3, NFR10a, NFR10c, NFR15, NFR16
 *
 * Every cycle runs in this fixed order (B2.2.4; do not reorder: the order
 * is the interlock):
 *   1 wake (R2), cycle start, health check-in
 *   2 sense      encoders, time, plausibility, window, path, progress
 *   3 safety     any FLG_SAFETY bit -> ESTOP; MOTOR_EN gate (FR17)
 *   4 commands   drain K5 (latched while moving or in ESTOP)
 *   5 state      set-points of the current state; MOVE/TURN completion
 *   6 control    PI speed loop per wheel
 *   7 stall      a driven wheel that does not turn for 300 ms
 *   8 actuate    drive in a moving state, otherwise brake
 *   9 publish    status board motion part
 * A safety flag set at any time is acted on at the next cycle start:
 * <= 10 ms + <= 1 ms higher-priority work, inside NFR10a's 15 ms.
 *
 * Phase 2 (baseline) representation: plain static variables, all written
 * only by motion_task. Phase 4 packs them into the PRV_MOT_* / PRV_WIN_*
 * layout rows without changing behaviour.
 */

#include "motion_task.h"       /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "cfg_motion.h"
#include "motion.h"
#include "odometry.h"
#include "speed_pid.h"
#include "safety.h"
#include "fault.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/sys_tick.h"
#include "../subsystem5/sys_health.h"

/* ---- private constants and types ------------------------------------ */

#define MOTION_Q8_ONE          (256)        /* Q8 scale                    */
#define MOTION_DT_US_MAX       (63000u)     /* dt clamp before us -> ms    */
#define MOTION_US_TO_MS_SHIFT  (20u)        /* (us x 1049) >> 20 = ms      */
#define MOTION_DELTA_ABS_MAX   (0xFFFFu)    /* |delta| cap in checks       */
#define MOTION_RATE_SCALE_US   (10000u)     /* one 10 ms tick, in us       */
#define MOTION_RATE_DT_MAX_US  (1000000u)   /* plausibility dt cap         */
#define MOTION_PATH2_MAX       (262142u)    /* 2 x 131071 (A23, PATH2 row) */
#define MOTION_STREAK_MAX      (3u)         /* 2-bit streak counter        */
#define MOTION_HALF_SHIFT      (1u)         /* path = path2 / 2            */

#if ((MOTION_DELTA_ABS_MAX * MOTION_RATE_SCALE_US) > 0xFFFFFFFFu)
#error "Plausibility product no longer fits 32 bits"
#endif
#if ((2u * APP_TICK_COUNTS_MAX * MOTION_RATE_DT_MAX_US) > 0xFFFFFFFFu)
#error "Plausibility limit no longer fits 32 bits"
#endif
#if ((MOTION_DT_US_MAX * MOTION_US_TO_MS_Q20) > 0xFFFFFFFFu)
#error "dt conversion no longer fits 32 bits"
#endif

/* One wheel's sensed values for this cycle. */
typedef struct
{
    uint32_t raw;        /* last trusted E3 raw image                */
    int32_t  delta;      /* counts since last cycle (modular)        */
    uint32_t abs_delta;  /* min(|delta|, 0xFFFF)                     */
    int32_t  pos;        /* counts since the last ZERO (modular)     */
    int32_t  speed_cpw;  /* window speed, counts per 50 ms           */
} motion_wheel_t;

/* ---- private data (written only by motion_task) ---------------------- */

static uint32_t g_state       = APP_MOT_BRAKE;   /* APP_MOT_*            */
static bool     gb_is_enc_ok  = false;           /* PIO program loaded   */
static bool     gb_is_motor_en = false;          /* APP_MODE_MOTOR_EN    */

/* Active command */
static uint32_t g_act_type    = APP_CMD_STOP;
static uint32_t g_act_tag     = 0u;
static int32_t  g_base_cpw    = 0;     /* base set-point, cpw          */
static uint32_t g_target2     = 0u;    /* 2 x target counts            */
static uint32_t g_prog2       = 0u;    /* this command's own progress  */
static int32_t  g_diff_start  = 0;     /* pos_l - pos_r at the start   */

/* One-slot latch for a command that arrives while moving */
static bool     gb_is_pend_valid = false;
static uint32_t g_pend_type   = APP_CMD_STOP;
static uint32_t g_pend_param  = 0u;
static uint32_t g_pend_tag    = 0u;

/* Control */
static speed_pid_t       g_pid_l     = {0};   /* E1 integrators */
static speed_pid_t       g_pid_r     = {0};
static int32_t           g_duty_l    = 0;     /* permille       */
static int32_t           g_duty_r    = 0;
static uint32_t          g_stall_l   = 0u;    /* ticks          */
static uint32_t          g_stall_r   = 0u;
static uint32_t          g_strk_l    = 0u;    /* impossible reads in a row */
static uint32_t          g_strk_r    = 0u;
static odometry_window_t g_window    = {{0}, {0}, {0}, 0u};
static uint32_t          g_path2     = 0u;    /* 2 x path length, half counts */

/* Encoder images (E3) and time (E5) */
static uint32_t g_raw0_l   = 0u;     /* raw counts at the last ZERO */
static uint32_t g_raw0_r   = 0u;
static uint32_t g_prev_l   = 0u;     /* last trusted raw counts     */
static uint32_t g_prev_r   = 0u;
static uint32_t g_prev_l_us = 0u;    /* time of each trusted count  */
static uint32_t g_prev_r_us = 0u;
static uint32_t g_prev_us  = 0u;     /* time of the last read (window) */

/* This cycle's sensed values */
static motion_wheel_t g_left  = {0u, 0, 0u, 0, 0};
static motion_wheel_t g_right = {0u, 0, 0u, 0, 0};

/* ---- private function prototypes ------------------------------------- */

static void motion_startup(void);
static void motion_cycle(void);
static void motion_sense(void);
static void motion_sense_wheel(motion_wheel_t * p_wheel, uint32_t raw,
                               uint32_t now_us, uint32_t * p_prev,
                               uint32_t * p_prev_us, uint32_t * p_raw0,
                               uint32_t * p_streak);
static bool motion_check_safety(void);
static void motion_commands(void);
static void motion_handle_cmd(uint32_t type, uint32_t param, uint32_t tag);
static void motion_start_or_latch(uint32_t type, uint32_t param,
                                  uint32_t tag);
static void motion_start_cmd(uint32_t type, uint32_t param, uint32_t tag);
static void motion_start_move(uint32_t state, int32_t base_cpw,
                              uint32_t target2, uint32_t tag);
static void motion_zero(uint32_t tag);
static void motion_cancel_active(void);
static void motion_state_step(int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_state_follow(int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_state_move(int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_state_turn(int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_finish_if_done(void);
static void motion_control(int32_t sp_l, int32_t sp_r);
static void motion_stall_check(void);
static uint32_t motion_stall_count(uint32_t count, int32_t duty,
                                   int32_t delta);
static void motion_actuate(void);
static void motion_publish(void);
static bool motion_is_moving(uint32_t state);
static void motion_reset_control(void);
static uint32_t motion_abs_capped(int32_t value);

/* ---- public functions ------------------------------------------------- */

/**
 * motion_task
 * @brief   Task body: start-up, then one cycle per R2 wake-up (10 ms).
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context motion_task
 */
void motion_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    motion_startup();

    for (;;)
    {
        /* 1. Released by R2 every 10 ms; E_OK or a queued wake-up. */
        (void) tk_slp_tsk(TMO_FEVR);
        sys_tick_cycle_start(SYS_TICK_SLOT_MOTION);
        sys_health_check_in(SYS_HEALTH_BIT_MOTION);

        motion_cycle();
    }
}

/* ---- private functions ------------------------------------------------ */

/**
 * motion_startup
 * @brief   Motors braked first (NFR10c), encoders loaded, odometry based
 *          on the current counts, registration with the 10 ms tick.
 * @return  none
 * @context motion_task (once)
 */
static void motion_startup(void)
{
    uint32_t pio_index = 0u;
    uint32_t sm_l      = 0u;
    uint32_t sm_r      = 0u;

    drv_motor_init();                        /* all four inputs low */
    gb_is_enc_ok = (0 == drv_encoder_init());

    if (drv_encoder_location(&pio_index, &sm_l, &sm_r))
    {
        app_log("ENC PIO%d SM %d %d offset 0\n",
                (int32_t) pio_index,         /* cast: 0..1 */
                (int32_t) sm_l, (int32_t) sm_r); /* cast: 0..3 */
    }
    else
    {
        app_log("ENC no PIO space: motors stay braked\n", 0, 0, 0);
    }

    drv_encoder_read(&g_raw0_l, &g_raw0_r);
    g_prev_l  = g_raw0_l;
    g_prev_r  = g_raw0_r;
    g_prev_us = sys_tick_now_us();
    g_prev_l_us = g_prev_us;
    g_prev_r_us = g_prev_us;
    odometry_window_reset(&g_window);
    motion_reset_control();
    g_state = APP_MOT_BRAKE;

    app_check(sys_tick_register(SYS_TICK_SLOT_MOTION, tk_get_tid()),
              "motion sys_tick_register");
}

/**
 * motion_cycle
 * @brief   Steps 2-9 of one 10 ms cycle, in the fixed order.
 * @return  none
 * @context motion_task
 */
static void motion_cycle(void)
{
    int32_t sp_l          = 0;
    int32_t sp_r          = 0;
    bool    b_is_stopped  = false;

    if (gb_is_enc_ok)
    {
        motion_sense();                         /* 2 */
        b_is_stopped = motion_check_safety();   /* 3 */
        motion_commands();                      /* 4 */

        if (!b_is_stopped)
        {
            motion_state_step(&sp_l, &sp_r);    /* 5 */
            motion_control(sp_l, sp_r);         /* 6 */
            motion_stall_check();               /* 7 */
        }
        else
        {
            motion_reset_control();             /* ESTOP: skip 5-7 */
        }
    }
    else
    {
        /* No encoder program: stay braked for ever, but keep the cycle
           and the check-in alive. Set the fault every cycle so a RESET
           can never hide it (D.13). Commands are drained and dropped.  */
        fault_set(APP_FAULT_PIO_LOAD);
        motion_cancel_active();
        gb_is_pend_valid = false;
        motion_commands();
        g_state = APP_MOT_BRAKE;
        motion_reset_control();
    }

    motion_actuate();                           /* 8 */
    motion_publish();                           /* 9 */
}

/**
 * motion_sense
 * @brief   Step 2: encoder deltas, tick length, plausibility, speed window,
 *          path and the active command's own progress.
 * @return  none
 * @context motion_task
 */
static void motion_sense(void)
{
    uint32_t raw_l  = 0u;
    uint32_t raw_r  = 0u;
    uint32_t now_us = 0u;
    uint32_t dt_us  = 0u;
    uint32_t dt_ms  = 0u;
    uint32_t step2  = 0u;

    drv_encoder_read(&raw_l, &raw_r);
    now_us    = sys_tick_now_us();
    dt_us     = now_us - g_prev_us;      /* unsigned: wraps correctly */
    g_prev_us = now_us;

    motion_sense_wheel(&g_left, raw_l, now_us, &g_prev_l, &g_prev_l_us,
                       &g_raw0_l, &g_strk_l);
    motion_sense_wheel(&g_right, raw_r, now_us, &g_prev_r, &g_prev_r_us,
                       &g_raw0_r, &g_strk_r);

    /* (us x 1049) >> 20 = ms, no division; <= 63000 x 1049 < 2^32 */
    dt_ms = (((dt_us > MOTION_DT_US_MAX) ? MOTION_DT_US_MAX : dt_us)
             * MOTION_US_TO_MS_Q20) >> MOTION_US_TO_MS_SHIFT;

    odometry_window_push(&g_window, g_left.delta, g_right.delta, dt_ms);
    odometry_window_speed(&g_window, &g_left.speed_cpw, &g_right.speed_cpw);

    /* Path: sum of |travel| of both wheels (half counts). Each |delta| is
       capped at 0xFFFF, so the sum cannot wrap. Clamp is RNG (A23).     */
    step2 = g_left.abs_delta + g_right.abs_delta;
    if ((MOTION_PATH2_MAX - g_path2) < step2)
    {
        g_path2 = MOTION_PATH2_MAX;
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
    else
    {
        g_path2 += step2;
    }

    /* Progress of a MOVE or TURN counts from 0 for this command only,
       never from the possibly saturated path (review R03).            */
    if ((APP_MOT_FOLLOW != g_state) && motion_is_moving(g_state))
    {
        g_prog2 = ((0xFFFFFFFFu - g_prog2) < step2) ? 0xFFFFFFFFu
                                                    : (g_prog2 + step2);
    }
}

/**
 * motion_sense_wheel
 * @brief   One wheel: modular delta, position, and the plausibility check.
 *          A read whose rate is impossible is NOT trusted: it counts as no
 *          movement, so it can never advance the path, a MOVE/TURN's
 *          progress or the position (review finding: one glitch of 5000
 *          counts completed a 1 m MOVE). The last trusted count is kept,
 *          so the next read is judged against it. Two impossible reads in
 *          a row raise APP_FAULT_ENCODER (again every cycle it lasts) and
 *          resynchronise: the new count becomes the baseline and raw0
 *          moves by the same jump, so the position stays continuous and
 *          a real counter jump cannot lock the wheel out for ever.
 *          The count and the time of the last trusted read are kept as a
 *          pair per wheel: after a rejected read the next delta spans
 *          several cycles, so it is judged against the time since that
 *          trusted read, not the last cycle (follow-up review: a valid
 *          recovery after a late tick was raising a false ENCODER fault).
 *          SPEC-QUESTION: B2.2.4 step 2 pushes every delta; the spec needs
 *          this rejection rule and the count/time pairing (test S2-U19
 *          proposed).
 * @param   p_wheel   receives this cycle's values
 * @param   raw        raw image read now
 * @param   now_us     time of this read
 * @param   p_prev     last trusted raw image (updated)
 * @param   p_prev_us  time of the last trusted raw image (updated)
 * @param   p_raw0     raw image at the last ZERO (moved on a resync)
 * @param   p_streak   impossible-read streak (updated)
 * @return  none
 * @context motion_task
 */
static void motion_sense_wheel(motion_wheel_t * p_wheel, uint32_t raw,
                               uint32_t now_us, uint32_t * p_prev,
                               uint32_t * p_prev_us, uint32_t * p_raw0,
                               uint32_t * p_streak)
{
    uint32_t const dt_us  = now_us - *p_prev_us;  /* since trusted read */
    uint32_t const dt_cap = (dt_us > MOTION_RATE_DT_MAX_US)
                            ? MOTION_RATE_DT_MAX_US : dt_us;
    /* cast: modular difference of E3 images, reinterpreted as signed */
    int32_t const  delta  = (int32_t) (raw - *p_prev);

    /* Impossible if |delta| / dt > 2 x the physical maximum per tick:
       |delta| x 10000 > 2 x APP_TICK_COUNTS_MAX x dt_us (no division). */
    bool const b_is_impossible =
        ((motion_abs_capped(delta) * MOTION_RATE_SCALE_US)
         > ((2u * APP_TICK_COUNTS_MAX) * dt_cap));

    if (!b_is_impossible)
    {
        *p_streak  = 0u;
        *p_prev    = raw;               /* trusted: accept the movement */
        *p_prev_us = now_us;
        p_wheel->delta = delta;
    }
    else
    {
        if (*p_streak < MOTION_STREAK_MAX)
        {
            (*p_streak)++;
        }
        p_wheel->delta = 0;             /* untrusted: no movement */
    }

    if (*p_streak >= MOTION_ENC_FAULT_STREAK)
    {
        fault_set(APP_FAULT_ENCODER);   /* again every cycle it lasts */
        *p_raw0   += raw - *p_prev;     /* resync, jump left out of pos */
        *p_prev    = raw;
        *p_prev_us = now_us;
    }

    p_wheel->raw       = *p_prev;       /* last trusted image (ZERO) */
    p_wheel->abs_delta = motion_abs_capped(p_wheel->delta);
    /* cast: modular difference of E3 images, reinterpreted as signed */
    p_wheel->pos       = (int32_t) (*p_prev - *p_raw0);
}

/**
 * motion_check_safety
 * @brief   Step 3: FLG_SAFETY first, then the FR17 motor permission.
 * @return  true if a safety bit is set (skip steps 5-7, brake)
 * @context motion_task
 */
static bool motion_check_safety(void)
{
    bool b_is_stopped = false;

    if (safety_is_active())
    {
        if (APP_MOT_ESTOP != g_state)
        {
            motion_cancel_active();   /* the latch is kept */
            g_state = APP_MOT_ESTOP;
        }
        b_is_stopped = true;
    }
    else if (APP_MOT_ESTOP == g_state)
    {
        /* Flags cleared: BRAKE now, so a command sent together with the
           clearing (e.g. a bypass MOVE) starts in step 4 of this cycle. */
        g_state = APP_MOT_BRAKE;
    }
    else
    {
        /* no change */
    }

    gb_is_motor_en = mission_mode_is_set(APP_MODE_MOTOR_EN);
    if ((!gb_is_motor_en) && motion_is_moving(g_state))
    {
        motion_cancel_active();
        g_state = APP_MOT_BRAKE;
    }

    return b_is_stopped;
}

/**
 * motion_commands
 * @brief   Step 4: drains K5 (at most MOTION_K5_ENTRIES), then starts a
 *          latched command if the robot is braked.
 * @return  none
 * @context motion_task
 */
static void motion_commands(void)
{
    uint32_t type      = APP_CMD_STOP;
    uint32_t param     = 0u;
    uint32_t tag       = 0u;
    uint32_t count     = 0u;
    bool     b_has_cmd = true;

    while (b_has_cmd && (count < MOTION_K5_ENTRIES))
    {
        b_has_cmd = motion_cmd_receive(&type, &param, &tag);
        if (b_has_cmd)
        {
            motion_handle_cmd(type, param, tag);
            count++;
        }
    }

    if ((APP_MOT_BRAKE == g_state) && gb_is_pend_valid)
    {
        gb_is_pend_valid = false;
        motion_start_cmd(g_pend_type, g_pend_param, g_pend_tag);
    }
}

/**
 * motion_handle_cmd
 * @brief   One K5 command: STOP always acts; ZERO acts in BRAKE; a FOLLOW
 *          in FOLLOW only changes the speed; others start or are latched.
 * @param   type   APP_CMD_*
 * @param   param  mm, deg or mm/s
 * @param   tag    command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_handle_cmd(uint32_t type, uint32_t param, uint32_t tag)
{
    switch (type)
    {
        case APP_CMD_STOP:
            motion_cancel_active();
            gb_is_pend_valid = false;
            motion_reset_control();
            if (APP_MOT_ESTOP != g_state)
            {
                g_state = APP_MOT_BRAKE;  /* ESTOP stays until flags clear */
            }
            break;

        case APP_CMD_ZERO:
            if (APP_MOT_BRAKE == g_state)
            {
                motion_zero(tag);
            }
            else
            {
                motion_start_or_latch(type, param, tag);
            }
            break;

        case APP_CMD_FOLLOW:
            if (APP_MOT_FOLLOW == g_state)
            {
                /* Speed change only, no stop. cast: param <= 300 */
                g_base_cpw = odometry_mm_s_to_cpw((int32_t) param);
                g_act_tag  = tag;
            }
            else
            {
                motion_start_or_latch(type, param, tag);
            }
            break;

        case APP_CMD_FWD:
        case APP_CMD_BACK:
        case APP_CMD_LEFT:
        case APP_CMD_RIGHT:
            motion_start_or_latch(type, param, tag);
            break;

        default:
            /* Unknown type: cannot come from motion.c; ignored. */
            break;
    }
}

/**
 * motion_start_or_latch
 * @brief   Starts a command now from BRAKE or STALLED, otherwise puts it
 *          in the one-slot latch (the newest replaces an older one).
 * @param   type   APP_CMD_*
 * @param   param  mm, deg or mm/s
 * @param   tag    command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_start_or_latch(uint32_t type, uint32_t param,
                                  uint32_t tag)
{
    if ((APP_MOT_BRAKE == g_state) || (APP_MOT_STALLED == g_state))
    {
        motion_start_cmd(type, param, tag);
    }
    else
    {
        gb_is_pend_valid = true;
        g_pend_type      = type;
        g_pend_param     = param;
        g_pend_tag       = tag;
    }
}

/**
 * motion_start_cmd
 * @brief   Starts a command. ZERO always acts. While APP_MODE_MOTOR_EN is
 *          clear, any driving command is dropped (FR17) and is not started
 *          later when the bit is set.
 * @param   type   APP_CMD_*
 * @param   param  mm, deg or mm/s (validated by motion.c)
 * @param   tag    command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_start_cmd(uint32_t type, uint32_t param, uint32_t tag)
{
    /* casts below: param <= 2047 (validated), fits int32_t */
    if (APP_CMD_ZERO == type)
    {
        g_state = APP_MOT_BRAKE;
        motion_zero(tag);
    }
    else if (!gb_is_motor_en)
    {
        /* Dropped: motors are not allowed to drive (FR17). */
    }
    else if (APP_CMD_FOLLOW == type)
    {
        motion_reset_control();
        g_act_type = type;
        g_act_tag  = tag;
        g_base_cpw = odometry_mm_s_to_cpw((int32_t) param);
        g_state    = APP_MOT_FOLLOW;
    }
    else if ((APP_CMD_FWD == type) || (APP_CMD_BACK == type))
    {
        g_act_type = type;
        motion_start_move((APP_CMD_FWD == type) ? APP_MOT_MOVE_FWD
                                                : APP_MOT_MOVE_BACK,
                          (APP_CMD_FWD == type)
                          ? odometry_mm_s_to_cpw(MOTION_MOVE_SPEED_MM_S)
                          : -odometry_mm_s_to_cpw(MOTION_REVERSE_SPEED_MM_S),
                          2u * odometry_mm_to_counts(param), tag);
    }
    else if ((APP_CMD_LEFT == type) || (APP_CMD_RIGHT == type))
    {
        g_act_type = type;
        motion_start_move((APP_CMD_LEFT == type) ? APP_MOT_TURN_L
                                                 : APP_MOT_TURN_R,
                          odometry_mm_s_to_cpw(MOTION_TURN_SPEED_MM_S),
                          2u * odometry_turn_counts(param), tag);
    }
    else
    {
        /* STOP is handled in motion_handle_cmd(); nothing else exists. */
    }
}

/**
 * motion_start_move
 * @brief   Common start of a MOVE or TURN: progress counts from 0.
 * @param   state     APP_MOT_MOVE_FWD/BACK or APP_MOT_TURN_L/R
 * @param   base_cpw  base set-point (negative for MOVE_BACK)
 * @param   target2   2 x the counts to travel (half counts)
 * @param   tag       command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_start_move(uint32_t state, int32_t base_cpw,
                              uint32_t target2, uint32_t tag)
{
    motion_reset_control();
    g_act_tag    = tag;
    g_base_cpw   = base_cpw;
    g_target2    = target2;
    g_prog2      = 0u;
    g_diff_start = g_left.pos - g_right.pos;
    g_state      = state;
}

/**
 * motion_zero
 * @brief   APP_CMD_ZERO in BRAKE: positions and path to 0 for a new run,
 *          then MOVE_DONE with this command's tag.
 * @param   tag  command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_zero(uint32_t tag)
{
    g_raw0_l     = g_left.raw;
    g_raw0_r     = g_right.raw;
    g_left.pos   = 0;
    g_right.pos  = 0;
    g_path2      = 0u;
    g_diff_start = 0;

    /* Full K4 is counted by mission as APP_FAULT_QUEUE_OVERFLOW. */
    (void) mission_post_event(APP_EVT_MOVE_DONE, tag);
}

/**
 * motion_cancel_active
 * @brief   Forgets the active command without MOVE_DONE (review R06).
 * @return  none
 * @context motion_task
 */
static void motion_cancel_active(void)
{
    g_act_type = APP_CMD_STOP;
    g_base_cpw = 0;
    g_target2  = 0u;
    g_prog2    = 0u;
}

/**
 * motion_state_step
 * @brief   Step 5: set-points of the current state (one handler each).
 * @param   p_sp_l  receives the left set-point, cpw
 * @param   p_sp_r  receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_step(int32_t * p_sp_l, int32_t * p_sp_r)
{
    *p_sp_l = 0;
    *p_sp_r = 0;

    switch (g_state)
    {
        case APP_MOT_BRAKE:
        case APP_MOT_STALLED:
            /* braked: set-points 0 */
            break;

        case APP_MOT_FOLLOW:
            motion_state_follow(p_sp_l, p_sp_r);
            break;

        case APP_MOT_MOVE_FWD:
        case APP_MOT_MOVE_BACK:
            motion_state_move(p_sp_l, p_sp_r);
            break;

        case APP_MOT_TURN_L:
        case APP_MOT_TURN_R:
            motion_state_turn(p_sp_l, p_sp_r);
            break;

        case APP_MOT_ESTOP:
            /* braked; left in step 3 once no safety bit is set */
            break;

        default:
            motion_cancel_active();     /* corrupted state: stop safely */
            g_state = APP_MOT_ESTOP;
            break;
    }
}

/**
 * motion_state_follow
 * @brief   FOLLOW: base speed corrected by the steering error. + steering
 *          = line left of centre = slow the left wheel (app_types.h).
 * @param   p_sp_l  receives the left set-point, cpw
 * @param   p_sp_r  receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_follow(int32_t * p_sp_l, int32_t * p_sp_r)
{
    status_board_t snap = {0};
    int32_t        diff = 0;

    status_board_read(&snap);

    /* Signed division by 256: shift with sign fix-up, no library call. */
    diff    = (status_board_steering(&snap) * MOTION_KP_STEER_Q8)
              / MOTION_Q8_ONE;
    *p_sp_l = g_base_cpw - diff;
    *p_sp_r = g_base_cpw + diff;
}

/**
 * motion_state_move
 * @brief   MOVE_FWD / MOVE_BACK: straight line, wheel difference held at
 *          its start value. Completion caps the distance (NFR16).
 * @param   p_sp_l  receives the left set-point, cpw
 * @param   p_sp_r  receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_move(int32_t * p_sp_l, int32_t * p_sp_r)
{
    int32_t const drift = (g_left.pos - g_right.pos) - g_diff_start;

    /* Signed division by 256: shift with sign fix-up, no library call.
       |drift| <= 262142 here (A23), x 64 < 2^31.                       */
    int32_t const corr = (drift * MOTION_K_HEADING_Q8) / MOTION_Q8_ONE;

    *p_sp_l = g_base_cpw - corr;
    *p_sp_r = g_base_cpw + corr;

    motion_finish_if_done();
}

/**
 * motion_state_turn
 * @brief   TURN_L / TURN_R: wheels in opposite directions.
 * @param   p_sp_l  receives the left set-point, cpw
 * @param   p_sp_r  receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_turn(int32_t * p_sp_l, int32_t * p_sp_r)
{
    if (APP_MOT_TURN_L == g_state)
    {
        *p_sp_l = -g_base_cpw;
        *p_sp_r = g_base_cpw;
    }
    else
    {
        *p_sp_l = g_base_cpw;
        *p_sp_r = -g_base_cpw;
    }

    motion_finish_if_done();
}

/**
 * motion_finish_if_done
 * @brief   A MOVE or TURN whose own progress reached its target: brake,
 *          BRAKE, MOVE_DONE with the command's tag.
 * @return  none
 * @context motion_task
 */
static void motion_finish_if_done(void)
{
    if (g_prog2 >= g_target2)
    {
        uint32_t const tag = g_act_tag;

        motion_cancel_active();
        g_state = APP_MOT_BRAKE;
        (void) mission_post_event(APP_EVT_MOVE_DONE, tag); /* see zero */
    }
}

/**
 * motion_control
 * @brief   Step 6: PI speed loop in a moving state; otherwise duty 0.
 * @param   sp_l  left set-point, cpw
 * @param   sp_r  right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_control(int32_t sp_l, int32_t sp_r)
{
    if (motion_is_moving(g_state))
    {
        g_duty_l = speed_pid_step(&g_pid_l, sp_l, g_left.speed_cpw,
                                  g_duty_l);
        g_duty_r = speed_pid_step(&g_pid_r, sp_r, g_right.speed_cpw,
                                  g_duty_r);
    }
    else
    {
        motion_reset_control();
    }
}

/**
 * motion_stall_check
 * @brief   Step 7: a wheel driven with >= STALL_DUTY_MIN that has not
 *          moved for MOTION_STALL_TICKS cycles -> brake, STALLED, fault
 *          bit first, then the STALL event with the wheel bits.
 * @return  none
 * @context motion_task
 */
static void motion_stall_check(void)
{
    uint32_t bits = 0u;

    if (motion_is_moving(g_state))
    {
        g_stall_l = motion_stall_count(g_stall_l, g_duty_l, g_left.delta);
        g_stall_r = motion_stall_count(g_stall_r, g_duty_r, g_right.delta);
    }
    else
    {
        g_stall_l = 0u;
        g_stall_r = 0u;
    }

    bits = ((g_stall_l >= MOTION_STALL_TICKS) ? APP_STALL_LEFT : 0u)
           | ((g_stall_r >= MOTION_STALL_TICKS) ? APP_STALL_RIGHT : 0u);

    if (0u != bits)
    {
        motion_cancel_active();
        motion_reset_control();
        g_state = APP_MOT_STALLED;
        fault_set(APP_FAULT_MOTOR_STALL);
        (void) mission_post_event(APP_EVT_STALL, bits); /* see zero */
        g_stall_l = 0u;
        g_stall_r = 0u;
    }
}

/**
 * motion_stall_count
 * @brief   One wheel's stall counter: +1 while driven hard and not
 *          moving, else 0. Saturates at MOTION_STALL_TICKS.
 * @param   count  counter so far
 * @param   duty   this cycle's duty, permille
 * @param   delta  this cycle's counts
 * @return  new counter
 * @context motion_task
 */
static uint32_t motion_stall_count(uint32_t count, int32_t duty,
                                   int32_t delta)
{
    uint32_t result = 0u;

    if ((motion_abs_capped(duty) >= (uint32_t) MOTION_STALL_DUTY_MIN_PERMILLE)
        && (0 == delta))   /* cast: positive constant 300 */
    {
        result = (count < MOTION_STALL_TICKS) ? (count + 1u) : count;
    }

    return result;
}

/**
 * motion_actuate
 * @brief   Step 8: drive in FOLLOW/MOVE/TURN, otherwise brake.
 * @return  none
 * @context motion_task
 */
static void motion_actuate(void)
{
    if (gb_is_enc_ok && motion_is_moving(g_state))
    {
        drv_motor_set(g_duty_l, g_duty_r);
    }
    else
    {
        drv_motor_brake();
    }
}

/**
 * motion_publish
 * @brief   Step 9: motion part of the status board (natural units; the
 *          board clamps and converts).
 * @return  none
 * @context motion_task
 */
static void motion_publish(void)
{
    status_board_motion_t part = {0, 0, 0u, 0, 0, 0u};

    part.speed_l_cpw  = g_left.speed_cpw;
    part.speed_r_cpw  = g_right.speed_cpw;
    part.motion_state = g_state;
    part.pos_l_counts = g_left.pos;
    part.pos_r_counts = g_right.pos;
    part.path_counts  = g_path2 >> MOTION_HALF_SHIFT;  /* path2 / 2 */

    status_board_write_motion(&part);
}

/**
 * motion_is_moving
 * @brief   FOLLOW, MOVE_FWD/BACK or TURN_L/R.
 * @param   state  APP_MOT_*
 * @return  true if the wheels are driven in this state
 * @context motion_task
 */
static bool motion_is_moving(uint32_t state)
{
    return ((APP_MOT_FOLLOW == state) || (APP_MOT_MOVE_FWD == state)
            || (APP_MOT_MOVE_BACK == state) || (APP_MOT_TURN_L == state)
            || (APP_MOT_TURN_R == state));
}

/**
 * motion_reset_control
 * @brief   Both PI integrators and duties to 0.
 * @return  none
 * @context motion_task
 */
static void motion_reset_control(void)
{
    speed_pid_reset(&g_pid_l);
    speed_pid_reset(&g_pid_r);
    g_duty_l = 0;
    g_duty_r = 0;
}

/**
 * motion_abs_capped
 * @brief   |value| as unsigned, capped at 0xFFFF (safe for INT32_MIN).
 * @param   value  any int32_t
 * @return  0..0xFFFF
 * @context any
 */
static uint32_t motion_abs_capped(int32_t value)
{
    /* casts: two's-complement negate in unsigned arithmetic is defined
       for every value, including INT32_MIN                            */
    uint32_t const mag = (value < 0) ? (0u - (uint32_t) value)
                                     : (uint32_t) value;

    return ((mag > MOTION_DELTA_ABS_MAX) ? MOTION_DELTA_ABS_MAX : mag);
}

/*** end of file ***/
