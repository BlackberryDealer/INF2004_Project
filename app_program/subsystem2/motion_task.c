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
 * Representation: the private state, written only by motion_task, is
 * packed into the PRV_MOT_* / PRV_WIN_* rows of tools/layout.csv (minimum
 * bits per field). What step 2 senses in a cycle is passed on the stack
 * (motion_sense_t); a wheel position is the difference of two encoder
 * images and is not stored. Behaviour is that of the unpacked code.
 */

#include "motion_task.h"       /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "../app_bits.h"
#include "../app_layout.h"
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
/* S2-U26: the encoder read takes a few us; a longer gap between the clock
   reads around it means the task was held up, so counts and time do not
   belong together and the read is skipped (not trusted, not counted). */
#define MOTION_READ_SKEW_US    (500u)
/* path2 = 2 x path; wraps with the 17-bit PATH (odometry.h).         */
#define MOTION_PATH2_MASK      ((ODOMETRY_PATH_MASK << 1u) | 1u)
#define MOTION_STREAK_MAX      ((uint32_t) PRV_MOT_C_STRK_L_MAX) /* saturates */
#define MOTION_HALF_SHIFT      (1u)         /* path = path2 / 2            */
#define MOTION_WHEEL_L         (0u)         /* index of the left wheel     */
#define MOTION_WHEEL_R         (1u)
#define MOTION_WHEELS          (2u)
#define MOTION_UM_PER_MM       (1000u)
/* 2 x the counts of the creep zone, as odometry_mm_to_counts() rounds it
   (checked by S2-U32); a constant, so it is not stored.                */
#define MOTION_CREEP_ZONE2                                                 \
    (2u * (((MOTION_CREEP_ZONE_MM * MOTION_UM_PER_MM)                      \
            + (APP_UM_PER_COUNT / 2u)) / APP_UM_PER_COUNT))
#define MOTION_PROG2_MAX       ((uint32_t) PRV_MOT_C_PROG2_MAX)
#define MOTION_DIFF_MASK       ((uint32_t) PRV_MOT_E_DIFF_START_MAX)
#define MOTION_DIFF_SIGN       ((MOTION_DIFF_MASK + 1u) >> 1u)

#if ((MOTION_DELTA_ABS_MAX * MOTION_RATE_SCALE_US) > 0xFFFFFFFFu)
#error "Plausibility product no longer fits 32 bits"
#endif
#if ((2u * APP_TICK_COUNTS_MAX * MOTION_RATE_DT_MAX_US) > 0xFFFFFFFFu)
#error "Plausibility limit no longer fits 32 bits"
#endif
#if ((MOTION_DT_US_MAX * MOTION_US_TO_MS_Q20) > 0xFFFFFFFFu)
#error "dt conversion no longer fits 32 bits"
#endif
/* The packed private fields (layout groups PRV_MOT_*) must hold the full
   range of what is stored in them.                                      */
#if (MOTION_I_CLAMP > PRV_MOT_PI_INTEG_MAX) \
    || (MOTION_DUTY_MAX_PERMILLE > PRV_MOT_PI_DUTY_MAX)
#error "PRV_MOT_PI is too narrow for the integrator clamp or the duty"
#endif
#if (MOTION_STALL_TICKS > PRV_MOT_B_STALL_L_MAX) \
    || (MOTION_STALL_TICKS > PRV_MOT_B_STALL_R_MAX) \
    || (MOTION_CREEP_STUCK_TICKS > PRV_MOT_A_CREEP_STILL_MAX)
#error "A PRV_MOT tick counter is too narrow for its limit"
#endif
#if (MOTION_ENC_FAULT_STREAK > PRV_MOT_C_STRK_L_MAX) \
    || (PRV_MOT_C_STRK_L_MAX != PRV_MOT_C_STRK_R_MAX)
#error "PRV_MOT_C streak fields do not hold MOTION_ENC_FAULT_STREAK"
#endif
#if (PRV_MOT_C_PROG2_MAX < PRV_MOT_C_TARGET2_MAX)
#error "PROG2 must saturate at or above every TARGET2"
#endif
#if (MOTION_RATE_DT_MAX_US > PRV_MOT_F_LOG_DT_MAX) \
    || (APP_CMD_ZERO > PRV_MOT_A_PEND_TYPE_MAX) \
    || (APP_MOT_STALLED > PRV_MOT_A_STATE_MAX)
#error "A PRV_MOT field is too narrow for its source"
#endif
#if (PRV_MOT_D_PATH2_MAX != ((2 * SB_MOT3_PATH_MAX) + 1))
#error "PATH2 must be one bit wider than the 17-bit PATH"
#endif

/* Words with every field at its reset value (signed fields hold their
   offset, which stands for 0).                                         */
#define MOTION_B_RESET                                                     \
    ((((uint32_t) PRV_MOT_B_BASE_CPW_OFFSET) << PRV_MOT_B_BASE_CPW_POS)    \
     | (((uint32_t) PRV_MOT_B_CREEP_CPW_OFFSET) << PRV_MOT_B_CREEP_CPW_POS))
#define MOTION_PI_RESET                                                    \
    ((((uint32_t) PRV_MOT_PI_INTEG_OFFSET) << PRV_MOT_PI_INTEG_POS)        \
     | (((uint32_t) PRV_MOT_PI_DUTY_OFFSET) << PRV_MOT_PI_DUTY_POS))

/* What step 2 sensed in this cycle; lives on the stack of motion_cycle. */
typedef struct
{
    int32_t delta_l;      /* trusted counts since the last cycle      */
    int32_t delta_r;
    int32_t speed_l_cpw;  /* window speed, counts per 50 ms           */
    int32_t speed_r_cpw;
} motion_sense_t;

/* ---- private data (written only by motion_task) ---------------------- */

/* Packed private state: one row per field in tools/layout.csv, read and
   written only through app_bits_get / app_bits_put with the generated
   PRV_MOT_* macros (BARR-C 5.4: no bit-fields, no casts into words).   */
static uint32_t g_mot_a = 0u;              /* PRV_MOT_A: state, command */
static uint32_t g_mot_b = MOTION_B_RESET;  /* PRV_MOT_B: set-points     */
static uint32_t g_mot_c = 0u;              /* PRV_MOT_C: target, streak */
static uint32_t g_path2 = 0u;              /* PRV_MOT_D: 2 x path, wraps */
static uint16_t g_diff_start = 0u;         /* PRV_MOT_E: modulo 2^16    */
static uint32_t g_enc_log_dt_us = 0u;      /* PRV_MOT_F: <= 1 s         */
static uint32_t g_pi[MOTION_WHEELS] =      /* PRV_MOT_PI: integ, duty   */
{
    MOTION_PI_RESET, MOTION_PI_RESET
};
static odometry_window_t g_window = ODOMETRY_WINDOW_EMPTY; /* PRV_WIN_* */

/* Full-width values (class E rows E_MOT_RAW, E_MOT_US, E_MOT_LOG): the
   PIO counters and the 1 us timer are 32 bits and wrap; only modular
   differences are used, and a counter jump is judged on its full width. */
static uint32_t g_zero_raw[MOTION_WHEELS]  = {0u, 0u};  /* image at ZERO */
static uint32_t g_trust_raw[MOTION_WHEELS] = {0u, 0u};  /* last trusted  */
static uint32_t g_trust_us[MOTION_WHEELS]  = {0u, 0u};  /* and its time  */
static uint32_t g_prev_us       = 0u;      /* time of the last read     */
static int32_t  g_enc_log_delta = 0;       /* counts of the logged read */

/* ---- private function prototypes ------------------------------------- */

static void motion_startup(void);
static void motion_cycle(void);
static void motion_sense(motion_sense_t * p_sense);
static int32_t motion_sense_wheel(uint32_t wheel, uint32_t raw,
                                  uint32_t now_us, bool b_is_skewed);
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
static void motion_state_step(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_state_follow(int32_t * p_sp_l, int32_t * p_sp_r);
static int32_t motion_follow_split(int32_t steer);
static void motion_state_move(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_state_turn(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r);
static void motion_finish_if_done(motion_sense_t const * p_sense);
static bool motion_in_creep_zone(void);
static int32_t motion_move_base(void);
static void motion_control(motion_sense_t const * p_sense, int32_t sp_l,
                           int32_t sp_r);
static void motion_pi_step(uint32_t wheel, int32_t setpoint_cpw,
                           int32_t speed_cpw);
static void motion_stall_check(motion_sense_t const * p_sense);
static uint32_t motion_stall_count(uint32_t count, int32_t duty,
                                   int32_t delta);
static void motion_actuate(void);
static void motion_publish(motion_sense_t const * p_sense);
static bool motion_is_moving(uint32_t state);
static void motion_reset_control(void);
static uint32_t motion_abs_capped(int32_t value);
static uint32_t motion_a(uint32_t pos, uint32_t width);
static void motion_a_set(uint32_t pos, uint32_t width, uint32_t value);
static uint32_t motion_b(uint32_t pos, uint32_t width);
static void motion_b_set(uint32_t pos, uint32_t width, uint32_t value);
static uint32_t motion_c(uint32_t pos, uint32_t width);
static void motion_c_set(uint32_t pos, uint32_t width, uint32_t value);
static uint32_t motion_state(void);
static void motion_set_state(uint32_t state);
static int32_t motion_base_cpw(void);
static void motion_set_base_cpw(int32_t base_cpw);
static int32_t motion_creep_cpw(void);
static void motion_set_creep_cpw(int32_t creep_cpw);
static int32_t motion_duty(uint32_t wheel);
static int32_t motion_pos(uint32_t wheel);

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
    motion_a_set(PRV_MOT_A_ENC_OK_POS, PRV_MOT_A_ENC_OK_BITS,
                 (0 == drv_encoder_init()) ? 1u : 0u);

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

    drv_encoder_read(&g_zero_raw[MOTION_WHEEL_L],
                     &g_zero_raw[MOTION_WHEEL_R]);
    g_trust_raw[MOTION_WHEEL_L] = g_zero_raw[MOTION_WHEEL_L];
    g_trust_raw[MOTION_WHEEL_R] = g_zero_raw[MOTION_WHEEL_R];
    g_prev_us = sys_tick_now_us();
    g_trust_us[MOTION_WHEEL_L] = g_prev_us;
    g_trust_us[MOTION_WHEEL_R] = g_prev_us;
    odometry_window_reset(&g_window);
    motion_reset_control();
    motion_set_state(APP_MOT_BRAKE);

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
    motion_sense_t sense  = {0, 0, 0, 0};
    int32_t sp_l          = 0;
    int32_t sp_r          = 0;
    bool    b_is_stopped  = false;

    if (0u != motion_a(PRV_MOT_A_ENC_OK_POS, PRV_MOT_A_ENC_OK_BITS))
    {
        motion_sense(&sense);                   /* 2 */
        b_is_stopped = motion_check_safety();   /* 3 */
        motion_commands();                      /* 4 */

        if (!b_is_stopped)
        {
            motion_state_step(&sense, &sp_l, &sp_r);  /* 5 */
            motion_control(&sense, sp_l, sp_r);       /* 6 */
            motion_stall_check(&sense);               /* 7 */
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
        motion_a_set(PRV_MOT_A_PEND_VALID_POS, PRV_MOT_A_PEND_VALID_BITS, 0u);
        motion_commands();
        motion_set_state(APP_MOT_BRAKE);
        motion_reset_control();
    }

    motion_actuate();                           /* 8 */
    motion_publish(&sense);                     /* 9 */

    /* Bring-up diagnosis, after the fault bit and the motor command of
       this cycle (see PRV_MOT_B LOG_PEND).                             */
    if (0u != motion_b(PRV_MOT_B_LOG_PEND_POS, PRV_MOT_B_LOG_PEND_BITS))
    {
        motion_b_set(PRV_MOT_B_LOG_PEND_POS, PRV_MOT_B_LOG_PEND_BITS, 0u);
        /* casts: wheel is 0..1 and dt is <= 1 000 000, both fit int32_t */
        app_log("ENC impossible: wheel %d (0 = L, 1 = R), %d counts in"
                " %d us\n",
                (int32_t) motion_b(PRV_MOT_B_LOG_WHEEL_POS,
                                   PRV_MOT_B_LOG_WHEEL_BITS),
                g_enc_log_delta, (int32_t) g_enc_log_dt_us);
    }
}

/**
 * motion_sense
 * @brief   Step 2: encoder deltas, tick length, plausibility, speed window,
 *          path and the active command's own progress.
 * @param   p_sense  receives this cycle's deltas and window speeds
 * @return  none
 * @context motion_task
 */
static void motion_sense(motion_sense_t * p_sense)
{
    uint32_t raw_l       = 0u;
    uint32_t raw_r       = 0u;
    uint32_t before_us   = 0u;
    uint32_t now_us      = 0u;
    uint32_t dt_us       = 0u;
    uint32_t dt_ms       = 0u;
    uint32_t step2       = 0u;
    uint32_t prog2       = 0u;
    bool     b_is_skewed = false;

    /* S2-U26: clock read on both sides of the encoder read */
    before_us   = sys_tick_now_us();
    drv_encoder_read(&raw_l, &raw_r);
    now_us      = sys_tick_now_us();
    b_is_skewed = ((now_us - before_us) > MOTION_READ_SKEW_US);
    dt_us       = now_us - g_prev_us;    /* unsigned: wraps correctly */
    g_prev_us   = now_us;

    p_sense->delta_l = motion_sense_wheel(MOTION_WHEEL_L, raw_l, now_us,
                                          b_is_skewed);
    p_sense->delta_r = motion_sense_wheel(MOTION_WHEEL_R, raw_r, now_us,
                                          b_is_skewed);

    /* (us x 1049) >> 20 = ms, no division; <= 63000 x 1049 < 2^32 */
    dt_ms = (((dt_us > MOTION_DT_US_MAX) ? MOTION_DT_US_MAX : dt_us)
             * MOTION_US_TO_MS_Q20) >> MOTION_US_TO_MS_SHIFT;

    odometry_window_push(&g_window, p_sense->delta_l, p_sense->delta_r,
                         dt_ms);
    odometry_window_speed(&g_window, &p_sense->speed_l_cpw,
                          &p_sense->speed_r_cpw);

    /* Path: sum of |travel| of both wheels (half counts). Each |delta| is
       capped at 0xFFFF, so the sum fits 32 bits. The path wraps at 2^17
       counts (about 42.5 m): the track has no line end, and readers
       take distances modulo 2^17 (odometry_path_since).                */
    step2   = motion_abs_capped(p_sense->delta_l)
              + motion_abs_capped(p_sense->delta_r);
    g_path2 = (g_path2 + step2) & MOTION_PATH2_MASK;

    /* Progress of a MOVE or TURN counts from 0 for this command only,
       never from the wrapping path (review R03). It saturates at the
       top of PROG2, which is at or above every target (SAT).          */
    if ((APP_MOT_FOLLOW != motion_state())
        && motion_is_moving(motion_state()))
    {
        prog2 = motion_c(PRV_MOT_C_PROG2_POS, PRV_MOT_C_PROG2_BITS);
        prog2 = ((MOTION_PROG2_MAX - prog2) < step2) ? MOTION_PROG2_MAX
                                                     : (prog2 + step2);
        motion_c_set(PRV_MOT_C_PROG2_POS, PRV_MOT_C_PROG2_BITS, prog2);
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
 * @param   wheel      MOTION_WHEEL_L or MOTION_WHEEL_R
 * @param   raw        raw image read now
 * @param   now_us     time of this read
 * @param   b_is_skewed  the task was held up during the read (S2-U26):
 *                     count and time do not match, skip this read
 * @return  trusted counts since the last cycle (0 if not trusted)
 * @context motion_task
 */
static int32_t motion_sense_wheel(uint32_t wheel, uint32_t raw,
                                  uint32_t now_us, bool b_is_skewed)
{
    uint32_t const strk_pos = (MOTION_WHEEL_L == wheel)
                              ? PRV_MOT_C_STRK_L_POS : PRV_MOT_C_STRK_R_POS;
    uint32_t const dt_us  = now_us - g_trust_us[wheel]; /* since trusted */
    uint32_t const dt_cap = (dt_us > MOTION_RATE_DT_MAX_US)
                            ? MOTION_RATE_DT_MAX_US : dt_us;
    /* cast: modular difference of E3 images, reinterpreted as signed */
    int32_t const  delta  = (int32_t) (raw - g_trust_raw[wheel]);

    /* Impossible if |delta| / dt > 2 x the physical maximum per tick:
       |delta| x 10000 > 2 x APP_TICK_COUNTS_MAX x dt_us (no division). */
    bool const b_is_impossible =
        ((motion_abs_capped(delta) * MOTION_RATE_SCALE_US)
         > ((2u * APP_TICK_COUNTS_MAX) * dt_cap));
    uint32_t streak = motion_c(strk_pos, PRV_MOT_C_STRK_L_BITS);
    int32_t  result = 0;

    if (b_is_skewed)
    {
        /* S2-U26: count and timestamp may be up to the hold-up apart.
           Neither trusted nor counted as impossible; the next good read
           measures the movement against the last trusted pair.       */
        result = 0;
    }
    else if (!b_is_impossible)
    {
        streak             = 0u;
        g_trust_raw[wheel] = raw;       /* trusted: accept the movement */
        g_trust_us[wheel]  = now_us;
        result             = delta;
    }
    else
    {
        if (streak < MOTION_STREAK_MAX)
        {
            streak++;
        }
        result = 0;                     /* untrusted: no movement */
    }

    if (MOTION_ENC_FAULT_STREAK == streak)
    {
        /* Bring-up diagnosis, once per episode (the streak saturates
           above this value): record now, print after step 9.           */
        if (0u == motion_b(PRV_MOT_B_LOG_PEND_POS, PRV_MOT_B_LOG_PEND_BITS))
        {
            motion_b_set(PRV_MOT_B_LOG_PEND_POS, PRV_MOT_B_LOG_PEND_BITS, 1u);
            motion_b_set(PRV_MOT_B_LOG_WHEEL_POS, PRV_MOT_B_LOG_WHEEL_BITS,
                         wheel);
            g_enc_log_delta = delta;
            g_enc_log_dt_us = dt_cap;   /* <= 1 s: fits LOG_DT (20 bits) */
        }
    }

    if (streak >= MOTION_ENC_FAULT_STREAK)
    {
        fault_set(APP_FAULT_ENCODER);   /* again every cycle it lasts */
        g_zero_raw[wheel] += raw - g_trust_raw[wheel]; /* resync: the jump
                                                          stays out of pos */
        g_trust_raw[wheel] = raw;
        g_trust_us[wheel]  = now_us;
    }

    motion_c_set(strk_pos, PRV_MOT_C_STRK_L_BITS, streak);

    return result;
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
        if (APP_MOT_ESTOP != motion_state())
        {
            motion_cancel_active();   /* the latch is kept */
            motion_set_state(APP_MOT_ESTOP);
        }
        b_is_stopped = true;
    }
    else if (APP_MOT_ESTOP == motion_state())
    {
        /* Flags cleared: BRAKE now, so a command sent together with the
           clearing (e.g. a bypass MOVE) starts in step 4 of this cycle. */
        motion_set_state(APP_MOT_BRAKE);
    }
    else
    {
        /* no change */
    }

    if ((!mission_mode_is_set(APP_MODE_MOTOR_EN))
        && motion_is_moving(motion_state()))
    {
        motion_cancel_active();
        motion_set_state(APP_MOT_BRAKE);
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

    if ((APP_MOT_BRAKE == motion_state())
        && (0u != motion_a(PRV_MOT_A_PEND_VALID_POS,
                           PRV_MOT_A_PEND_VALID_BITS)))
    {
        motion_a_set(PRV_MOT_A_PEND_VALID_POS, PRV_MOT_A_PEND_VALID_BITS, 0u);
        motion_start_cmd(motion_a(PRV_MOT_A_PEND_TYPE_POS,
                                  PRV_MOT_A_PEND_TYPE_BITS),
                         motion_a(PRV_MOT_A_PEND_PARAM_POS,
                                  PRV_MOT_A_PEND_PARAM_BITS),
                         motion_a(PRV_MOT_A_PEND_TAG_POS,
                                  PRV_MOT_A_PEND_TAG_BITS));
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
            motion_a_set(PRV_MOT_A_PEND_VALID_POS, PRV_MOT_A_PEND_VALID_BITS,
                         0u);
            motion_reset_control();
            if (APP_MOT_ESTOP != motion_state())
            {
                /* ESTOP stays until the flags clear */
                motion_set_state(APP_MOT_BRAKE);
            }
            break;

        case APP_CMD_ZERO:
            if (APP_MOT_BRAKE == motion_state())
            {
                motion_zero(tag);
            }
            else
            {
                motion_start_or_latch(type, param, tag);
            }
            break;

        case APP_CMD_FOLLOW:
            if (APP_MOT_FOLLOW == motion_state())
            {
                /* Speed change only, no stop. cast: param <= 300 */
                motion_set_base_cpw(odometry_mm_s_to_cpw((int32_t) param));
                motion_a_set(PRV_MOT_A_ACT_TAG_POS, PRV_MOT_A_ACT_TAG_BITS,
                             tag);
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
    if ((APP_MOT_BRAKE == motion_state())
        || (APP_MOT_STALLED == motion_state()))
    {
        motion_start_cmd(type, param, tag);
    }
    else
    {
        /* type, tag and param come from one K5 word (MSG_CMD), whose
           fields have the widths of the latch fields.                 */
        motion_a_set(PRV_MOT_A_PEND_VALID_POS, PRV_MOT_A_PEND_VALID_BITS, 1u);
        motion_a_set(PRV_MOT_A_PEND_TYPE_POS, PRV_MOT_A_PEND_TYPE_BITS, type);
        motion_a_set(PRV_MOT_A_PEND_PARAM_POS, PRV_MOT_A_PEND_PARAM_BITS,
                     param);
        motion_a_set(PRV_MOT_A_PEND_TAG_POS, PRV_MOT_A_PEND_TAG_BITS, tag);
    }
}

/**
 * motion_start_cmd
 * @brief   Starts a command. ZERO always acts. While APP_MODE_MOTOR_EN is
 *          clear, any driving command is dropped (FR17) and is not started
 *          later when the bit is set.
 *          The bit is read again here, not taken from step 3: on the
 *          dual-core build mission_task can set (or clear) MOTOR_EN and
 *          send this command on the other core after step 3 has read it.
 *          mission_task always changes the bit before it sends, so this
 *          read sees the value the command was sent under (test S2-U20).
 * @param   type   APP_CMD_*
 * @param   param  mm, deg or mm/s (validated by motion.c)
 * @param   tag    command tag 0..3
 * @return  none
 * @context motion_task
 */
static void motion_start_cmd(uint32_t type, uint32_t param, uint32_t tag)
{
    bool const b_is_motor_en = mission_mode_is_set(APP_MODE_MOTOR_EN);

    /* casts below: param <= 2047 (validated), fits int32_t */
    if (APP_CMD_ZERO == type)
    {
        motion_set_state(APP_MOT_BRAKE);
        motion_zero(tag);
    }
    else if (!b_is_motor_en)
    {
        /* Dropped: motors are not allowed to drive (FR17). */
    }
    else if (APP_CMD_FOLLOW == type)
    {
        motion_reset_control();
        motion_a_set(PRV_MOT_A_ACT_TYPE_POS, PRV_MOT_A_ACT_TYPE_BITS, type);
        motion_a_set(PRV_MOT_A_ACT_TAG_POS, PRV_MOT_A_ACT_TAG_BITS, tag);
        motion_set_base_cpw(odometry_mm_s_to_cpw((int32_t) param));
        motion_set_state(APP_MOT_FOLLOW);
    }
    else if ((APP_CMD_FWD == type) || (APP_CMD_BACK == type))
    {
        motion_a_set(PRV_MOT_A_ACT_TYPE_POS, PRV_MOT_A_ACT_TYPE_BITS, type);
        motion_start_move((APP_CMD_FWD == type) ? APP_MOT_MOVE_FWD
                                                : APP_MOT_MOVE_BACK,
                          (APP_CMD_FWD == type)
                          ? odometry_mm_s_to_cpw(MOTION_MOVE_SPEED_MM_S)
                          : -odometry_mm_s_to_cpw(MOTION_REVERSE_SPEED_MM_S),
                          2u * odometry_mm_to_counts(param), tag);
    }
    else if ((APP_CMD_LEFT == type) || (APP_CMD_RIGHT == type))
    {
        motion_a_set(PRV_MOT_A_ACT_TYPE_POS, PRV_MOT_A_ACT_TYPE_BITS, type);
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
    int32_t const creep = odometry_mm_s_to_cpw(MOTION_CREEP_MM_S);
    int32_t const mag   = (base_cpw < 0) ? -base_cpw : base_cpw;
    bool          b_was_clamped = false;
    uint32_t const target = app_bits_clamp_u(target2,
                                             (uint32_t) PRV_MOT_C_TARGET2_MAX,
                                             &b_was_clamped);

    if (b_was_clamped)
    {
        /* Cannot happen for a validated command: 2047 mm and 360 deg
           are both below the top of TARGET2 (RNG, S2-U32).            */
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    motion_reset_control();
    motion_a_set(PRV_MOT_A_ACT_TAG_POS, PRV_MOT_A_ACT_TAG_BITS, tag);
    motion_set_base_cpw(base_cpw);
    motion_c_set(PRV_MOT_C_TARGET2_POS, PRV_MOT_C_TARGET2_BITS, target);
    motion_c_set(PRV_MOT_C_PROG2_POS, PRV_MOT_C_PROG2_BITS, 0u);
    /* casts: the signed difference is stored modulo 2^16; converting a
       negative int32_t to uint32_t is defined (modular), and the masked
       value fits uint16_t. motion_state_move() takes the drift modulo
       2^16 as well, which is exact while |drift| < 32768 counts; one
       MOVE covers at most 2047 mm (6318 counts).                       */
    g_diff_start = (uint16_t) (((uint32_t) (motion_pos(MOTION_WHEEL_L)
                                            - motion_pos(MOTION_WHEEL_R)))
                               & MOTION_DIFF_MASK);
    motion_set_state(state);

    /* Creep never faster than the command itself; same sign as base. */
    if (creep >= mag)
    {
        motion_set_creep_cpw(base_cpw);
    }
    else
    {
        motion_set_creep_cpw((base_cpw < 0) ? -creep : creep);
    }
    motion_a_set(PRV_MOT_A_CREEP_STILL_POS, PRV_MOT_A_CREEP_STILL_BITS, 0u);
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
    /* The last trusted images become the base, so both positions read
       0 from here on.                                                  */
    g_zero_raw[MOTION_WHEEL_L] = g_trust_raw[MOTION_WHEEL_L];
    g_zero_raw[MOTION_WHEEL_R] = g_trust_raw[MOTION_WHEEL_R];
    g_path2      = 0u;
    g_diff_start = 0u;

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
    motion_a_set(PRV_MOT_A_ACT_TYPE_POS, PRV_MOT_A_ACT_TYPE_BITS,
                 APP_CMD_STOP);
    motion_set_base_cpw(0);
    motion_c_set(PRV_MOT_C_TARGET2_POS, PRV_MOT_C_TARGET2_BITS, 0u);
    motion_c_set(PRV_MOT_C_PROG2_POS, PRV_MOT_C_PROG2_BITS, 0u);
}

/**
 * motion_state_step
 * @brief   Step 5: set-points of the current state (one handler each).
 * @param   p_sense  this cycle's sensed values
 * @param   p_sp_l   receives the left set-point, cpw
 * @param   p_sp_r   receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_step(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r)
{
    *p_sp_l = 0;
    *p_sp_r = 0;

    switch (motion_state())
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
            motion_state_move(p_sense, p_sp_l, p_sp_r);
            break;

        case APP_MOT_TURN_L:
        case APP_MOT_TURN_R:
            motion_state_turn(p_sense, p_sp_l, p_sp_r);
            break;

        case APP_MOT_ESTOP:
            /* braked; left in step 3 once no safety bit is set */
            break;

        default:
            motion_cancel_active();     /* corrupted state: stop safely */
            motion_set_state(APP_MOT_ESTOP);
            break;
    }
}

/**
 * motion_state_follow
 * @brief   FOLLOW: base speed, cut by up to MOTION_CURVE_SLOW_PCT in
 *          proportion to |steering|, corrected by the steering error.
 *          + steering = line left of centre = slow the left wheel
 *          (app_types.h). Limit the split to retain at least one third
 *          of the slowed base speed on the inside wheel.
 * @param   p_sp_l  receives the left set-point, cpw
 * @param   p_sp_r  receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_follow(int32_t * p_sp_l, int32_t * p_sp_r)
{
    status_board_t snap     = {0};
    int32_t        steer    = 0;
    int32_t        cut      = 0;
    int32_t        base     = 0;
    int32_t        diff     = 0;
    int32_t        diff_max = 0;

    status_board_read(&snap);
    steer = status_board_steering(&snap);

    /* |steer| <= 1000 (board domain) x 131 fits; result <= 128. cast:
       small non-negative value fits int32_t                            */
    cut  = (int32_t) ((motion_abs_capped(steer) * MOTION_CURVE_SLOW_Q10)
                      >> 10);
    /* Signed division by 256: shift with sign fix-up, no library call. */
    base = (motion_base_cpw() * (MOTION_Q8_ONE - cut)) / MOTION_Q8_ONE;
    diff = motion_follow_split(steer);
    diff_max = (2 * base) / 3;
    if (diff > diff_max)
    {
        diff = diff_max;
    }
    else if (diff < -diff_max)
    {
        diff = -diff_max;
    }
    else
    {
        /* The requested split leaves enough inside speed. */
    }
    *p_sp_l = base - diff;
    *p_sp_r = base + diff;
}

/**
 * motion_follow_split
 * @brief   Steering split from the steering error. Below MOTION_SLOW_MM_S
 *          it shrinks in proportion to the base speed, so a steering error
 *          bends the path as much per mm as at slow speed (S2-U28: the same
 *          split at 70 mm/s swung the car across the line). That scaling
 *          is rounded to nearest: truncating it gave no correction below
 *          |steer| 256 at 60 mm/s (128 at slow speed) and the barcode
 *          reread crawled 8.4 s through a bend with the line under one
 *          sensor (on-car 12 Nov, S2-U30). At slow speed and above the
 *          split is unchanged (S2-U25, S2-U27).
 * @param   steer  steering error, -1000..+1000 (+ = line left)
 * @return  split, cpw
 * @context motion_task
 */
static int32_t motion_follow_split(int32_t steer)
{
    int32_t const slow = odometry_mm_s_to_cpw(MOTION_SLOW_MM_S);
    int32_t const base = motion_base_cpw();
    /* Signed division by 256 truncates toward zero, as before. */
    int32_t const full = (steer * MOTION_KP_STEER_Q8) / MOTION_Q8_ONE;
    int32_t       out  = full;

    if ((base > 0) && (base < slow))
    {
        /* |full| <= 7 cpw and base < slow: the product is small. */
        int32_t const prod = full * base;
        int32_t const half = slow / 2;

        out = (prod >= 0) ? ((prod + half) / slow)
                          : (-((half - prod) / slow));
    }
    else
    {
        /* Slow speed and above keep the tuned gain. */
    }

    return out;
}

/**
 * motion_state_move
 * @brief   MOVE_FWD / MOVE_BACK: straight line, wheel difference held at
 *          its start value. Completion caps the distance (NFR16).
 * @param   p_sense  this cycle's sensed values
 * @param   p_sp_l   receives the left set-point, cpw
 * @param   p_sp_r   receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_move(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r)
{
    /* Drift = (pos_l - pos_r) - start difference, taken modulo 2^16 and
       read as a signed 16-bit value (see motion_start_move). casts:
       int32_t to uint32_t is modular; d16 < 65536 fits int32_t.        */
    uint32_t const d16 = (((uint32_t) (motion_pos(MOTION_WHEEL_L)
                                       - motion_pos(MOTION_WHEEL_R)))
                          - g_diff_start) & MOTION_DIFF_MASK;
    int32_t const drift = (d16 >= MOTION_DIFF_SIGN)
                          ? (((int32_t) d16) - ((int32_t) MOTION_DIFF_MASK) - 1)
                          : ((int32_t) d16);

    /* Signed division by 256: shift with sign fix-up, no library call.
       |drift| < 32768 here, x 64 < 2^31.                               */
    int32_t const corr = (drift * MOTION_K_HEADING_Q8) / MOTION_Q8_ONE;
    int32_t const base = motion_move_base();

    *p_sp_l = base - corr;
    *p_sp_r = base + corr;

    motion_finish_if_done(p_sense);
}

/**
 * motion_state_turn
 * @brief   TURN_L / TURN_R: wheels in opposite directions.
 * @param   p_sense  this cycle's sensed values
 * @param   p_sp_l   receives the left set-point, cpw
 * @param   p_sp_r   receives the right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_state_turn(motion_sense_t const * p_sense,
                              int32_t * p_sp_l, int32_t * p_sp_r)
{
    int32_t const base = motion_move_base();

    if (APP_MOT_TURN_L == motion_state())
    {
        *p_sp_l = -base;
        *p_sp_r = base;
    }
    else
    {
        *p_sp_l = base;
        *p_sp_r = -base;
    }

    motion_finish_if_done(p_sense);
}

/**
 * motion_finish_if_done
 * @brief   A MOVE or TURN whose own progress reached its target, or that
 *          stood still for MOTION_CREEP_STUCK_TICKS in the creep zone
 *          after moving: brake, BRAKE, MOVE_DONE with the command's tag.
 * @param   p_sense  this cycle's sensed values
 * @return  none
 * @context motion_task
 */
static void motion_finish_if_done(motion_sense_t const * p_sense)
{
    uint32_t const prog2   = motion_c(PRV_MOT_C_PROG2_POS,
                                      PRV_MOT_C_PROG2_BITS);
    uint32_t const target2 = motion_c(PRV_MOT_C_TARGET2_POS,
                                      PRV_MOT_C_TARGET2_BITS);
    uint32_t       still   = motion_a(PRV_MOT_A_CREEP_STILL_POS,
                                      PRV_MOT_A_CREEP_STILL_BITS);

    if ((prog2 > 0u) && motion_in_creep_zone()
        && (0 == p_sense->delta_l) && (0 == p_sense->delta_r))
    {
        /* SAT: the MOVE ends below when the limit is reached */
        still = (still < (uint32_t) PRV_MOT_A_CREEP_STILL_MAX)
                ? (still + 1u) : still;
    }
    else
    {
        still = 0u;
    }
    motion_a_set(PRV_MOT_A_CREEP_STILL_POS, PRV_MOT_A_CREEP_STILL_BITS,
                 still);

    if ((prog2 >= target2) || (still >= MOTION_CREEP_STUCK_TICKS))
    {
        uint32_t const tag = motion_a(PRV_MOT_A_ACT_TAG_POS,
                                      PRV_MOT_A_ACT_TAG_BITS);

        motion_cancel_active();
        motion_set_state(APP_MOT_BRAKE);
        (void) mission_post_event(APP_EVT_MOVE_DONE, tag); /* see zero */
    }
}

/**
 * motion_in_creep_zone
 * @brief   True while a MOVE or TURN has at most MOTION_CREEP_ZONE_MM of
 *          wheel travel left (and is not yet done).
 * @return  true inside the creep zone
 * @context motion_task
 */
static bool motion_in_creep_zone(void)
{
    uint32_t const prog2   = motion_c(PRV_MOT_C_PROG2_POS,
                                      PRV_MOT_C_PROG2_BITS);
    uint32_t const target2 = motion_c(PRV_MOT_C_TARGET2_POS,
                                      PRV_MOT_C_TARGET2_BITS);

    return ((target2 > prog2) && ((target2 - prog2) <= MOTION_CREEP_ZONE2));
}

/**
 * motion_move_base
 * @brief   Base set-point of the running MOVE or TURN: the command speed,
 *          or the creep speed in the last MOTION_CREEP_ZONE_MM.
 * @return  base set-point, cpw (negative for MOVE_BACK)
 * @context motion_task
 */
static int32_t motion_move_base(void)
{
    int32_t base = motion_base_cpw();

    if (motion_in_creep_zone())
    {
        base = motion_creep_cpw();
    }
    else
    {
        /* full command speed */
    }

    return base;
}

/**
 * motion_control
 * @brief   Step 6: PI speed loop in a moving state; otherwise duty 0.
 * @param   p_sense  this cycle's sensed values
 * @param   sp_l     left set-point, cpw
 * @param   sp_r     right set-point, cpw
 * @return  none
 * @context motion_task
 */
static void motion_control(motion_sense_t const * p_sense, int32_t sp_l,
                           int32_t sp_r)
{
    if (motion_is_moving(motion_state()))
    {
        motion_pi_step(MOTION_WHEEL_L, sp_l, p_sense->speed_l_cpw);
        motion_pi_step(MOTION_WHEEL_R, sp_r, p_sense->speed_r_cpw);
    }
    else
    {
        motion_reset_control();
    }
}

/**
 * motion_pi_step
 * @brief   One PI step of one wheel: unpacks the integrator and the last
 *          duty from PRV_MOT_PI, runs speed_pid_step(), packs both again.
 *          speed_pid_step() keeps the integrator within +-MOTION_I_CLAMP
 *          and the duty within +-1000, so neither store clamps (RNG: a
 *          clamp would set RANGE_CLAMP).
 * @param   wheel         MOTION_WHEEL_L or MOTION_WHEEL_R
 * @param   setpoint_cpw  wanted speed, cpw
 * @param   speed_cpw     measured speed, cpw
 * @return  none
 * @context motion_task
 */
static void motion_pi_step(uint32_t wheel, int32_t setpoint_cpw,
                           int32_t speed_cpw)
{
    speed_pid_t pid  = {0};
    int32_t     duty = 0;
    uint32_t    word = g_pi[wheel];
    bool        b_integ_clamped = false;
    bool        b_duty_clamped  = false;

    pid.integ = app_bits_decode(app_bits_get(word, PRV_MOT_PI_INTEG_POS,
                                             PRV_MOT_PI_INTEG_BITS),
                                PRV_MOT_PI_INTEG_OFFSET);
    duty = speed_pid_step(&pid, setpoint_cpw, speed_cpw, motion_duty(wheel));

    word = app_bits_put(word, PRV_MOT_PI_INTEG_POS, PRV_MOT_PI_INTEG_BITS,
                        app_bits_encode(pid.integ, PRV_MOT_PI_INTEG_MIN,
                                        PRV_MOT_PI_INTEG_MAX,
                                        PRV_MOT_PI_INTEG_OFFSET,
                                        &b_integ_clamped));
    word = app_bits_put(word, PRV_MOT_PI_DUTY_POS, PRV_MOT_PI_DUTY_BITS,
                        app_bits_encode(duty, PRV_MOT_PI_DUTY_MIN,
                                        PRV_MOT_PI_DUTY_MAX,
                                        PRV_MOT_PI_DUTY_OFFSET,
                                        &b_duty_clamped));
    g_pi[wheel] = word;

    if (b_integ_clamped || b_duty_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
}

/**
 * motion_stall_check
 * @brief   Step 7: a wheel driven with >= STALL_DUTY_MIN that has not
 *          moved for MOTION_STALL_TICKS cycles -> brake, STALLED, fault
 *          bit first, then the STALL event with the wheel bits.
 * @param   p_sense  this cycle's sensed values
 * @return  none
 * @context motion_task
 */
static void motion_stall_check(motion_sense_t const * p_sense)
{
    uint32_t bits    = 0u;
    uint32_t stall_l = 0u;
    uint32_t stall_r = 0u;

    if (motion_is_moving(motion_state()))
    {
        stall_l = motion_stall_count(motion_b(PRV_MOT_B_STALL_L_POS,
                                              PRV_MOT_B_STALL_L_BITS),
                                     motion_duty(MOTION_WHEEL_L),
                                     p_sense->delta_l);
        stall_r = motion_stall_count(motion_b(PRV_MOT_B_STALL_R_POS,
                                              PRV_MOT_B_STALL_R_BITS),
                                     motion_duty(MOTION_WHEEL_R),
                                     p_sense->delta_r);
    }
    else
    {
        /* not driven: both counters 0 */
    }

    bits = ((stall_l >= MOTION_STALL_TICKS) ? APP_STALL_LEFT : 0u)
           | ((stall_r >= MOTION_STALL_TICKS) ? APP_STALL_RIGHT : 0u);

    if (0u != bits)
    {
        motion_cancel_active();
        motion_reset_control();
        motion_set_state(APP_MOT_STALLED);
        fault_set(APP_FAULT_MOTOR_STALL);
        (void) mission_post_event(APP_EVT_STALL, bits); /* see zero */
        stall_l = 0u;
        stall_r = 0u;
    }

    motion_b_set(PRV_MOT_B_STALL_L_POS, PRV_MOT_B_STALL_L_BITS, stall_l);
    motion_b_set(PRV_MOT_B_STALL_R_POS, PRV_MOT_B_STALL_R_BITS, stall_r);
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
    if ((0u != motion_a(PRV_MOT_A_ENC_OK_POS, PRV_MOT_A_ENC_OK_BITS))
        && motion_is_moving(motion_state()))
    {
        drv_motor_set(motion_duty(MOTION_WHEEL_L),
                      motion_duty(MOTION_WHEEL_R));
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
 * @param   p_sense  this cycle's sensed values
 * @return  none
 * @context motion_task
 */
static void motion_publish(motion_sense_t const * p_sense)
{
    status_board_motion_t part = {0, 0, 0u, 0, 0, 0u};

    part.speed_l_cpw  = p_sense->speed_l_cpw;
    part.speed_r_cpw  = p_sense->speed_r_cpw;
    part.motion_state = motion_state();
    part.pos_l_counts = motion_pos(MOTION_WHEEL_L);
    part.pos_r_counts = motion_pos(MOTION_WHEEL_R);
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
    g_pi[MOTION_WHEEL_L] = MOTION_PI_RESET;
    g_pi[MOTION_WHEEL_R] = MOTION_PI_RESET;
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

/**
 * motion_a
 * @brief   Reads one field of PRV_MOT_A.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @return  the stored value
 * @context motion_task
 */
static uint32_t motion_a(uint32_t pos, uint32_t width)
{
    return app_bits_get(g_mot_a, pos, width);
}

/**
 * motion_a_set
 * @brief   Writes one field of PRV_MOT_A.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @param   value  value within the field's range
 * @return  none
 * @context motion_task
 */
static void motion_a_set(uint32_t pos, uint32_t width, uint32_t value)
{
    g_mot_a = app_bits_put(g_mot_a, pos, width, value);
}

/**
 * motion_b
 * @brief   Reads one field of PRV_MOT_B.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @return  the stored value
 * @context motion_task
 */
static uint32_t motion_b(uint32_t pos, uint32_t width)
{
    return app_bits_get(g_mot_b, pos, width);
}

/**
 * motion_b_set
 * @brief   Writes one field of PRV_MOT_B.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @param   value  value within the field's range
 * @return  none
 * @context motion_task
 */
static void motion_b_set(uint32_t pos, uint32_t width, uint32_t value)
{
    g_mot_b = app_bits_put(g_mot_b, pos, width, value);
}

/**
 * motion_c
 * @brief   Reads one field of PRV_MOT_C.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @return  the stored value
 * @context motion_task
 */
static uint32_t motion_c(uint32_t pos, uint32_t width)
{
    return app_bits_get(g_mot_c, pos, width);
}

/**
 * motion_c_set
 * @brief   Writes one field of PRV_MOT_C.
 * @param   pos    field position (layout _POS macro)
 * @param   width  field width (layout _BITS macro)
 * @param   value  value within the field's range
 * @return  none
 * @context motion_task
 */
static void motion_c_set(uint32_t pos, uint32_t width, uint32_t value)
{
    g_mot_c = app_bits_put(g_mot_c, pos, width, value);
}

/**
 * motion_state
 * @brief   The executor state (PRV_MOT_A STATE).
 * @return  APP_MOT_*
 * @context motion_task
 */
static uint32_t motion_state(void)
{
    return motion_a(PRV_MOT_A_STATE_POS, PRV_MOT_A_STATE_BITS);
}

/**
 * motion_set_state
 * @brief   Sets the executor state (PRV_MOT_A STATE).
 * @param   state  APP_MOT_*
 * @return  none
 * @context motion_task
 */
static void motion_set_state(uint32_t state)
{
    motion_a_set(PRV_MOT_A_STATE_POS, PRV_MOT_A_STATE_BITS, state);
}

/**
 * motion_base_cpw
 * @brief   Base set-point of the active command (PRV_MOT_B BASE_CPW).
 * @return  cpw, negative for MOVE_BACK
 * @context motion_task
 */
static int32_t motion_base_cpw(void)
{
    return app_bits_decode(motion_b(PRV_MOT_B_BASE_CPW_POS,
                                    PRV_MOT_B_BASE_CPW_BITS),
                           PRV_MOT_B_BASE_CPW_OFFSET);
}

/**
 * motion_set_base_cpw
 * @brief   Stores the base set-point. RNG: the largest validated command
 *          (FOLLOW 300 mm/s) is 46 cpw; a clamp sets RANGE_CLAMP.
 * @param   base_cpw  cpw, negative for MOVE_BACK
 * @return  none
 * @context motion_task
 */
static void motion_set_base_cpw(int32_t base_cpw)
{
    bool b_was_clamped = false;

    motion_b_set(PRV_MOT_B_BASE_CPW_POS, PRV_MOT_B_BASE_CPW_BITS,
                 app_bits_encode(base_cpw, PRV_MOT_B_BASE_CPW_MIN,
                                 PRV_MOT_B_BASE_CPW_MAX,
                                 PRV_MOT_B_BASE_CPW_OFFSET, &b_was_clamped));
    if (b_was_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
}

/**
 * motion_creep_cpw
 * @brief   Creep set-point of the active MOVE or TURN (PRV_MOT_B
 *          CREEP_CPW).
 * @return  cpw, with the sign of the base set-point
 * @context motion_task
 */
static int32_t motion_creep_cpw(void)
{
    return app_bits_decode(motion_b(PRV_MOT_B_CREEP_CPW_POS,
                                    PRV_MOT_B_CREEP_CPW_BITS),
                           PRV_MOT_B_CREEP_CPW_OFFSET);
}

/**
 * motion_set_creep_cpw
 * @brief   Stores the creep set-point. RNG: at most MOTION_CREEP_MM_S
 *          (80 mm/s = 12 cpw); a clamp sets RANGE_CLAMP.
 * @param   creep_cpw  cpw, with the sign of the base set-point
 * @return  none
 * @context motion_task
 */
static void motion_set_creep_cpw(int32_t creep_cpw)
{
    bool b_was_clamped = false;

    motion_b_set(PRV_MOT_B_CREEP_CPW_POS, PRV_MOT_B_CREEP_CPW_BITS,
                 app_bits_encode(creep_cpw, PRV_MOT_B_CREEP_CPW_MIN,
                                 PRV_MOT_B_CREEP_CPW_MAX,
                                 PRV_MOT_B_CREEP_CPW_OFFSET, &b_was_clamped));
    if (b_was_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
}

/**
 * motion_duty
 * @brief   Duty applied to one wheel in the last control step (PRV_MOT_PI
 *          DUTY).
 * @param   wheel  MOTION_WHEEL_L or MOTION_WHEEL_R
 * @return  permille, -1000..+1000
 * @context motion_task
 */
static int32_t motion_duty(uint32_t wheel)
{
    return app_bits_decode(app_bits_get(g_pi[wheel], PRV_MOT_PI_DUTY_POS,
                                        PRV_MOT_PI_DUTY_BITS),
                           PRV_MOT_PI_DUTY_OFFSET);
}

/**
 * motion_pos
 * @brief   Position of one wheel: counts since the last ZERO, from the
 *          last trusted image (modular, so it needs no storage of its
 *          own).
 * @param   wheel  MOTION_WHEEL_L or MOTION_WHEEL_R
 * @return  counts (+ = forward)
 * @context motion_task
 */
static int32_t motion_pos(uint32_t wheel)
{
    /* cast: modular difference of E3 images, reinterpreted as signed */
    return (int32_t) (g_trust_raw[wheel] - g_zero_raw[wheel]);
}

/*** end of file ***/
