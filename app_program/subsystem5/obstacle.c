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
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "cfg_obstacle.h"
#include "obs_profile.h"
#include "obs_edge.h"
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
#if (OBS_WIDTH_4US_MAX != MSG_ECHO_WIDTH_MAX) \
    || (OBS_SEQ_LOW_MASK != MSG_ECHO_SEQ_MAX)
#error "The echo record limits must match the MSG_ECHO row"
#endif
#define OBS_CLEAR_HYST_MM      (50u)         /* APPROACH -> MONITOR   */
#define OBS_CLEAR_READINGS     (2u)
#define OBS_COARSE_STRIDE      (3u)          /* 15 deg between points */
#define OBS_CENTRE_DEG         (90u)
#define OBS_STEP_CENTRE        (12u)         /* step of 90 deg */
#define OBS_STEP_NONE          (0xFFu)
#define OBS_RANGE_LINES        (9u)          /* 25 steps, 3 per line */
#define OBS_RANGE_PER_LINE     (3u)
#define OBS_MASK_ALL           ((1u << APP_SCAN_STEPS) - 1u)  /* 25 bits */

/* Mode bits remembered for edge detection */
#define OBS_SEEN_RESCAN        (0x1u)
#define OBS_SEEN_STOPPED       (0x2u)
#define OBS_SEEN_RESUME        (0x4u)
#define OBS_DIST_LOG_PINGS     (8u)          /* bring-up print: ~0.5 s */
#define OBS_PART_WORDS         (3u)          /* SB_OBS0..2 */

/* Private fields (index into g_obs_f) */
#define OBS_F_MEASURED         (0u)
#define OBS_F_STATE            (1u)
#define OBS_F_SETTLE           (2u)
#define OBS_F_ON_RAMP          (3u)
#define OBS_F_INFERRED         (4u)
#define OBS_F_STEP             (5u)
#define OBS_F_SAMPLE_CNT       (6u)
#define OBS_F_PREV             (7u)          /* 4 mm code */
#define OBS_F_ECHO             (8u)          /* 4 mm code */
#define OBS_F_WARMUP           (9u)
#define OBS_F_SILENT           (10u)
#define OBS_F_LOST             (11u)
#define OBS_F_FAR              (12u)
#define OBS_F_PASS_MARK        (13u)
#define OBS_F_RAMP_HOLD        (14u)
#define OBS_F_MODE_SEEN        (15u)
#define OBS_F_DIST_LOG         (16u)
#define OBS_F_RAMP_LOGGED      (17u)
#define OBS_F_CREEP_LOGGED     (18u)
#define OBS_F_RAMP_MARK        (19u)
#define OBS_F_EDGE_PHASE       (20u)
#define OBS_F_EDGE_SUB         (21u)
#define OBS_F_EDGE_LO          (22u)
#define OBS_F_EDGE_HI          (23u)
#define OBS_F_EDGE_LIMIT       (24u)
#define OBS_F_EDGE_LO_D        (25u)         /* code below EDGE_LIMIT */
#define OBS_F_EDGE_HI_D        (26u)         /* code below EDGE_LIMIT */
#define OBS_F_EDGE_OK_LO       (27u)
#define OBS_F_EDGE_OK_HI       (28u)
#define OBS_F_EDGE_GRACE       (29u)
#define OBS_F_COUNT            (30u)


/* Every PRV_OBS_* row must hold the full range its code uses. */
#if (APP_SCAN_STEPS != PRV_OBS_M_MEASURED_BITS)                         \
    || (APP_SCAN_STEPS != PRV_OBS_I_INFERRED_BITS)                      \
    || ((APP_SCAN_STEPS - 1u) != PRV_OBS_I_STEP_MAX)
#error "PRV_OBS_M / PRV_OBS_I must have one bit / code per scan step"
#endif
#if (OBS_WATCH_SETTLE_PINGS > PRV_OBS_M_SETTLE_MAX)                     \
    || (OBS_SETTLE_PINGS > PRV_OBS_M_SETTLE_MAX)                        \
    || ((OBS_SCAN_SAMPLES - 1u) != PRV_OBS_I_SAMPLE_CNT_MAX)            \
    || (OBS_WARMUP_PINGS != PRV_OBS_A_WARMUP_MAX)                       \
    || (OBS_NO_ECHO_PINGS > PRV_OBS_A_SILENT_MAX)                       \
    || (OBS_LOST_PINGS >= PRV_OBS_A_LOST_MAX)                           \
    || (OBS_CLEAR_READINGS >= PRV_OBS_A_FAR_MAX)                        \
    || (OBS_RAMP_HOLD_PINGS != PRV_OBS_B_RAMP_HOLD_MAX)                 \
    || ((OBS_DIST_LOG_PINGS - 1u) != PRV_OBS_B_DIST_LOG_MAX)
#error "A PRV_OBS_* row no longer covers its cfg limit"
#endif
#if ((APP_DIST_NONE_MM / OBS_MM_STEP) != PRV_OBS_A_PREV_MAX)            \
    || ((APP_DIST_NONE_MM / OBS_MM_STEP) != PRV_OBS_A_ECHO_MAX)         \
    || ((APP_PROFILE_CLEAR_MM / OBS_MM_STEP) != PRV_OBS_SCAN_MM_MAX)    \
    || ((APP_PROFILE_CLEAR_MM / OBS_MM_STEP) != PRV_OBS_SMP_MM_MAX)     \
    || ((APP_DIST_NONE_MM % OBS_MM_STEP) != 0u)                         \
    || ((APP_PROFILE_CLEAR_MM % OBS_MM_STEP) != 0u)
#error "Distance codes must cover the sentinels in 4 mm steps"
#endif
#if (OBS_EDGE_SUB_MAX != PRV_OBS_E_EDGE_SUB_MAX)                         \
    || (OBS_SCAN_LAST_DEG != PRV_OBS_E_EDGE_LO_MAX)                     \
    || (OBS_SCAN_LAST_DEG != PRV_OBS_E_EDGE_HI_MAX)                     \
    || (OBS_EDGE_HI_SIDE != PRV_OBS_E_EDGE_PHASE_MAX)                   \
    || ((APP_PROFILE_CLEAR_MM / OBS_MM_STEP) != PRV_OBS_E_EDGE_LIMIT_MAX) \
    || ((OBS_CORE_BAND_MM / OBS_MM_STEP) != PRV_OBS_C_EDGE_LO_D_MAX)      \
    || ((OBS_CORE_BAND_MM / OBS_MM_STEP) != PRV_OBS_C_EDGE_HI_D_MAX)      \
    || ((OBS_CORE_BAND_MM % OBS_MM_STEP) != 0u)
#error "PRV_OBS_E must hold the edge pass (S5-U83)"
#endif
#if (SB_MOT3_PATH_MAX != PRV_OBS_B_PASS_MARK_MAX)                       \
    || (SB_MOT3_PATH_MAX != PRV_OBS_C_RAMP_MARK_MAX)
#error "Path marks must hold the whole wrapping path"
#endif

/* Echo record as obstacle_task uses it, decoded from one load of the
   packed MSG_ECHO word that obstacle_echo_isr writes (local copies
   only; nothing of this type is stored between calls).              */
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

/* Written only by the ISR (B5.2.2). MSG_ECHO: WIDTH 12, SEQ 2, VALID 1,
   PAD 1; the ISR stores the whole 16-bit word at once, so one task load
   is always a consistent record.                                     */
static volatile uint16_t        g_echo_word     = 0u;
static volatile uint8_t         g_echo_rise_cnt = 0u;  /* accepted rises */
static volatile uint8_t         g_echo_done_cnt = 0u;  /* accepted falls */
/* ISR-private. g_rise_us is E5: a stuck-high echo can exceed 65 ms. */
static volatile uint32_t        g_rise_us        = 0u;
static volatile uint8_t         g_rise_seq       = 0u;
static volatile bool            gb_is_rise_armed = false;
/* Written only by obstacle_task; the ISR only reads them. */
static volatile uint8_t         g_ping_seq         = 0u;  /* 8 bits, wraps */
static volatile uint8_t         g_echo_window_open = 0u;

/* Everything below is private to obstacle_task, packed in the
   PRV_OBS_* rows of tools/layout.csv. ECHO (S5-U38) is the last real
   echo distance in APPROACH: lost pings mean "too close to measure"
   only when that echo was already inside the stop distance.          */
static uint32_t g_obs_prv_m =   /* MEASURED, STATE = MONITOR, SETTLE, ON_RAMP */
    ((uint32_t) APP_SCAN_MONITOR << PRV_OBS_M_STATE_POS);
static uint32_t g_obs_prv_i = 0u; /* INFERRED, STEP, SAMPLE_CNT */
static uint32_t g_obs_prv_a =   /* PREV, ECHO = nothing; WARMUP (S5-U30) */
    ((uint32_t) PRV_OBS_A_PREV_MAX << PRV_OBS_A_PREV_POS)
    | ((uint32_t) PRV_OBS_A_ECHO_MAX << PRV_OBS_A_ECHO_POS)
    | ((uint32_t) OBS_WARMUP_PINGS << PRV_OBS_A_WARMUP_POS);
static uint32_t g_obs_prv_b = 0u; /* PASS_MARK, RAMP_HOLD, MODE_SEEN,
                                     DIST_LOG, RAMP_LOGGED, CREEP_LOGGED */
static uint32_t g_obs_prv_c = 0u; /* RAMP_MARK, EDGE_LO_D, EDGE_HI_D */
static uint32_t g_obs_prv_e = 0u; /* EDGE_PHASE, EDGE_SUB, EDGE_LO, EDGE_HI,
                                     EDGE_LIMIT, EDGE_OK_LO/HI, EDGE_GRACE
                                     (S5-U83) */
static uint32_t g_scan_start_us = 0u; /* E_OBS_US (E5): scan-time log */
/* Scan table and the pings at one step, 4 mm codes (PRV_OBS_SCAN,
   PRV_OBS_SMP); 255 = clear (APP_PROFILE_CLEAR_MM).                  */
static uint8_t  g_obs_scan[APP_SCAN_STEPS];
static uint8_t  g_samples[OBS_SCAN_SAMPLES]; /* S5-U47 */
/* The obstacle part of the board as last written, in the board's own
   SB_OBS0..2 layout (obstacle_task is its only writer).              */
static uint32_t g_part_w[OBS_PART_WORDS] =
{
    ((uint32_t) SB_OBS0_DIST_MAX << SB_OBS0_DIST_POS),
    ((uint32_t) SB_OBS1_WIDTH_MAX << SB_OBS1_WIDTH_POS)
    | ((uint32_t) SB_OBS1_NEAREST_MAX << SB_OBS1_NEAREST_POS)
    | ((uint32_t) SB_OBS1_CLEAR_L_MAX << SB_OBS1_CLEAR_L_POS)
    | ((uint32_t) SB_OBS1_CLEAR_R_MAX << SB_OBS1_CLEAR_R_POS),
    0u
};

/* One private field: the word that holds it, position and width. */
typedef struct
{
    uint32_t * p_word;
    uint32_t   pos;
    uint32_t   width;
} obstacle_field_t;

static obstacle_field_t const g_obs_f[OBS_F_COUNT] =
{
    {&g_obs_prv_m, PRV_OBS_M_MEASURED_POS, PRV_OBS_M_MEASURED_BITS},
    {&g_obs_prv_m, PRV_OBS_M_STATE_POS, PRV_OBS_M_STATE_BITS},
    {&g_obs_prv_m, PRV_OBS_M_SETTLE_POS, PRV_OBS_M_SETTLE_BITS},
    {&g_obs_prv_m, PRV_OBS_M_ON_RAMP_POS, PRV_OBS_M_ON_RAMP_BITS},
    {&g_obs_prv_i, PRV_OBS_I_INFERRED_POS, PRV_OBS_I_INFERRED_BITS},
    {&g_obs_prv_i, PRV_OBS_I_STEP_POS, PRV_OBS_I_STEP_BITS},
    {&g_obs_prv_i, PRV_OBS_I_SAMPLE_CNT_POS, PRV_OBS_I_SAMPLE_CNT_BITS},
    {&g_obs_prv_a, PRV_OBS_A_PREV_POS, PRV_OBS_A_PREV_BITS},
    {&g_obs_prv_a, PRV_OBS_A_ECHO_POS, PRV_OBS_A_ECHO_BITS},
    {&g_obs_prv_a, PRV_OBS_A_WARMUP_POS, PRV_OBS_A_WARMUP_BITS},
    {&g_obs_prv_a, PRV_OBS_A_SILENT_POS, PRV_OBS_A_SILENT_BITS},
    {&g_obs_prv_a, PRV_OBS_A_LOST_POS, PRV_OBS_A_LOST_BITS},
    {&g_obs_prv_a, PRV_OBS_A_FAR_POS, PRV_OBS_A_FAR_BITS},
    {&g_obs_prv_b, PRV_OBS_B_PASS_MARK_POS, PRV_OBS_B_PASS_MARK_BITS},
    {&g_obs_prv_b, PRV_OBS_B_RAMP_HOLD_POS, PRV_OBS_B_RAMP_HOLD_BITS},
    {&g_obs_prv_b, PRV_OBS_B_MODE_SEEN_POS, PRV_OBS_B_MODE_SEEN_BITS},
    {&g_obs_prv_b, PRV_OBS_B_DIST_LOG_POS, PRV_OBS_B_DIST_LOG_BITS},
    {&g_obs_prv_b, PRV_OBS_B_RAMP_LOGGED_POS, PRV_OBS_B_RAMP_LOGGED_BITS},
    {&g_obs_prv_b, PRV_OBS_B_CREEP_LOGGED_POS, PRV_OBS_B_CREEP_LOGGED_BITS},
    {&g_obs_prv_c, PRV_OBS_C_RAMP_MARK_POS, PRV_OBS_C_RAMP_MARK_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_PHASE_POS, PRV_OBS_E_EDGE_PHASE_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_SUB_POS, PRV_OBS_E_EDGE_SUB_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_LO_POS, PRV_OBS_E_EDGE_LO_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_HI_POS, PRV_OBS_E_EDGE_HI_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_LIMIT_POS, PRV_OBS_E_EDGE_LIMIT_BITS},
    {&g_obs_prv_c, PRV_OBS_C_EDGE_LO_D_POS, PRV_OBS_C_EDGE_LO_D_BITS},
    {&g_obs_prv_c, PRV_OBS_C_EDGE_HI_D_POS, PRV_OBS_C_EDGE_HI_D_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_OK_LO_POS, PRV_OBS_E_EDGE_OK_LO_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_OK_HI_POS, PRV_OBS_E_EDGE_OK_HI_BITS},
    {&g_obs_prv_e, PRV_OBS_E_EDGE_GRACE_POS, PRV_OBS_E_EDGE_GRACE_BITS},
};

/* ---- private function prototypes ------------------------------------- */

static void     obstacle_task(INT stacd, void * p_exinf);
static void     obstacle_cycle(void);
static void     obstacle_echo_isr(uint32_t events, uint32_t now_us);
static void     obstacle_ping(obstacle_reading_t * p_rd);
static bool     obstacle_wait_echo(uint8_t done0);
static void     obstacle_read_record(obstacle_echo_t * p_rec);
static void     obstacle_echo_store(uint32_t width_4us, uint32_t seq,
                                    bool b_is_valid);
static void     obstacle_scan_log(void);
static void     obstacle_ranges_log(void);
static void     obstacle_block_log(bool b_is_left, uint32_t step,
                                   uint32_t gap_mm);
static uint32_t obstacle_median3(uint8_t const * p_val);
static uint32_t obstacle_get(uint32_t field);
static void     obstacle_set(uint32_t field, uint32_t value);
static uint32_t obstacle_scan_mm(uint32_t step);
static uint8_t  obstacle_mm_code(uint32_t distance_mm);
static void     obstacle_part_get(status_board_obs_t * p_part);
static void     obstacle_part_put(status_board_obs_t const * p_part);
static uint32_t obstacle_part_mm(uint32_t distance_mm, uint32_t max);
static uint32_t obstacle_width_to_mm(uint32_t width_4us);
static uint32_t obstacle_round_mm(uint32_t distance_mm);
static void     obstacle_modes(uint32_t * p_rising);
static void     obstacle_update_ramp(bool b_is_armed);
static void     obstacle_enter(uint32_t state);
static void     obstacle_step_monitor(obstacle_reading_t const * p_rd);
static void     obstacle_step_approach(obstacle_reading_t const * p_rd);
static void     obstacle_step_scan(obstacle_reading_t const * p_rd);
static void     obstacle_step_creep(obstacle_reading_t const * p_rd);
static void     obstacle_step_settle(void);
static void     obstacle_step_passing(void);
static void     obstacle_step_watch(obstacle_reading_t const * p_rd);
static void     obstacle_scan_begin(void);
static bool     obstacle_next_fine(void);
static void     obstacle_scan_finish(void);
static void     obstacle_fill_gaps(void);
static void     obstacle_set_step(uint32_t step);
static bool     obstacle_edge_begin(void);
static bool     obstacle_edge_step(uint32_t code);
static void     obstacle_edge_load(obs_edge_t * p_edge);
static void     obstacle_edge_store(obs_edge_t const * p_edge);
static void     obstacle_edge_probe(obs_edge_probe_t const * p_next);
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
        g_obs_scan[idx] = obstacle_mm_code(APP_PROFILE_CLEAR_MM);
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
    uint32_t           log_cnt = 0u;  /* bring-up print divider */
    status_board_obs_t part;

    bool               b_is_armed = false;

    obstacle_ping(&reading);
    obstacle_modes(&rising);

    /* The stop is armed only while the car drives (APP_MODE_MOTOR_EN)
       outside the CAL_IR sweep, i.e. in RUN. Before that there is nothing
       to stop, and an ESTOP latched in BOOT stayed set until RESET, so it
       blocked the CAL_IR sweep and the car went to FAULT without moving
       (on-car logs 2026-10-28/29, test S5-U31). The sweep only spins on
       the spot, so it never approaches what the sensor sees, but turning
       swings the beam across nearby things: a near echo then a lost one
       latched ESTOP and stopped the sweep mid-turn (on-car logs
       2026-11-03, test S5-U34). Pings, NO_ECHO and the board distance
       stay live while unarmed.                                          */
    b_is_armed = mission_mode_is_set(APP_MODE_MOTOR_EN)
                 && (!mission_mode_is_set(APP_MODE_CAL_IR));
    obstacle_update_ramp(b_is_armed);

    if (0u != (rising & OBS_SEEN_RESUME))
    {
        obstacle_enter(APP_SCAN_MONITOR);
    }
    else if (!b_is_armed)
    {
        if (APP_SCAN_MONITOR != obstacle_get(OBS_F_STATE))
        {
            obstacle_enter(APP_SCAN_MONITOR);
        }

        /* Bring-up diagnosis: what the sensor sees while unarmed, every
           ~0.5 s. cast: distance 0..2047 mm, lost flag 0..1 fit int32_t */
        log_cnt = obstacle_get(OBS_F_DIST_LOG) + 1u;
        obstacle_set(OBS_F_DIST_LOG,
                     (log_cnt >= OBS_DIST_LOG_PINGS) ? 0u : log_cnt);
        if (log_cnt >= OBS_DIST_LOG_PINGS)
        {
            app_log("US dist %d mm, lost %d\n",
                    (int32_t) reading.distance_mm,
                    reading.b_is_lost ? 1 : 0, 0);
        }
    }
    else
    {
        switch (obstacle_get(OBS_F_STATE))
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
                else
                {
                    obstacle_step_creep(&reading);          /* S5-U55 */
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

    obstacle_part_get(&part);
    part.distance_mm   = reading.distance_mm;
    part.scanner_state = obstacle_get(OBS_F_STATE);
    obstacle_part_put(&part);
    obstacle_part_get(&part);       /* as stored: the board's 4 mm codes */
    status_board_write_obstacle(&part);
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
            /* cast: widen the 8-bit ping number; started, not finished */
            obstacle_echo_store(OBS_WIDTH_4US_MAX,
                                (uint32_t) g_rise_seq & OBS_SEQ_LOW_MASK,
                                false);
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
            /* cast: widen the 8-bit ping number */
            obstacle_echo_store(width,
                                (uint32_t) g_rise_seq & OBS_SEQ_LOW_MASK,
                                true);
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
        obstacle_set(OBS_F_SILENT, 0u);             /* echo complete */
        obstacle_set(OBS_F_WARMUP, 0u);             /* sensor is up */
        p_rd->b_is_lost   = false;
        p_rd->distance_mm = obstacle_width_to_mm(rec.width_4us);
    }
    else if (b_rise)
    {
        obstacle_set(OBS_F_SILENT, 0u); /* still high: nothing in range */
        obstacle_set(OBS_F_WARMUP, 0u); /* an edge: sensor is up */
    }
    else if (0u != obstacle_get(OBS_F_WARMUP))
    {
        /* power-up grace (S5-U30) */
        obstacle_set(OBS_F_WARMUP, obstacle_get(OBS_F_WARMUP) - 1u);
    }
    else
    {
        /* no edge at all (or stuck) */
        if (obstacle_get(OBS_F_SILENT) < OBS_NO_ECHO_PINGS)
        {
            obstacle_set(OBS_F_SILENT, obstacle_get(OBS_F_SILENT) + 1u);
        }
        if (obstacle_get(OBS_F_SILENT) >= OBS_NO_ECHO_PINGS)
        {
            fault_set(APP_FAULT_NO_ECHO);   /* again every silent ping */
        }
    }

    diff = (p_rd->distance_mm > (obstacle_get(OBS_F_PREV) * OBS_MM_STEP))
           ? (p_rd->distance_mm - (obstacle_get(OBS_F_PREV) * OBS_MM_STEP))
           : ((obstacle_get(OBS_F_PREV) * OBS_MM_STEP) - p_rd->distance_mm);
    p_rd->b_is_confirmed = (diff <= OBS_AGREE_MM);
    obstacle_set(OBS_F_PREV, p_rd->distance_mm / OBS_MM_STEP);
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
 * @brief   Coherent read of the ISR's record: one 16-bit load of the
 *          MSG_ECHO word (the ISR writes the whole word at once),
 *          decoded into the task's local copy.
 * @param   p_rec copy out
 * @context obstacle_task
 */
static void obstacle_read_record(obstacle_echo_t * p_rec)
{
    uint32_t const word = (uint32_t) g_echo_word;  /* one load */

    p_rec->width_4us  = app_bits_get(word, MSG_ECHO_WIDTH_POS,
                                     MSG_ECHO_WIDTH_BITS);
    p_rec->seq        = app_bits_get(word, MSG_ECHO_SEQ_POS,
                                     MSG_ECHO_SEQ_BITS);
    p_rec->b_is_valid = (0u != app_bits_get(word, MSG_ECHO_VALID_POS,
                                            MSG_ECHO_VALID_BITS));
}

/**
 * obstacle_echo_store
 * @brief   Packs one echo record into MSG_ECHO and stores the whole word
 *          with one 16-bit store (PAD stays 0).
 * @param   width_4us   0..OBS_WIDTH_4US_MAX (already saturated)
 * @param   seq         low 2 bits of the ping number
 * @param   b_is_valid  a complete pulse was measured
 * @context ISR (obstacle_echo_isr)
 */
static void obstacle_echo_store(uint32_t width_4us, uint32_t seq,
                                bool b_is_valid)
{
    uint32_t word = 0u;

    word = app_bits_put(word, MSG_ECHO_WIDTH_POS, MSG_ECHO_WIDTH_BITS,
                        width_4us);
    word = app_bits_put(word, MSG_ECHO_SEQ_POS, MSG_ECHO_SEQ_BITS, seq);
    word = app_bits_put(word, MSG_ECHO_VALID_POS, MSG_ECHO_VALID_BITS,
                        b_is_valid ? 1u : 0u);
    g_echo_word = (uint16_t) word; /* cast: MSG_ECHO is a 16-bit word */
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
    *p_rising   = now & ~obstacle_get(OBS_F_MODE_SEEN);
    obstacle_set(OBS_F_MODE_SEEN, now);
}

/**
 * obstacle_update_ramp
 * @brief   Tracks whether the car is climbing, descending, or exiting a
 *          ramp/hump (S5-U42). Holds ON_RAMP for OBS_RAMP_HOLD_PINGS
 *          (and OBS_RAMP_CLEAR_MM when odometry moves) after levelling off
 *          so the ramp crest and bottom exit do not trigger a false stop.
 * @param   b_is_armed true while obstacle detection is armed
 * @context obstacle_task
 */
static void obstacle_update_ramp(bool b_is_armed)
{
    status_board_t snap      = {0};
    uint32_t       mclass    = 0u;
    int32_t        pitch     = 0;
    uint32_t       path_now  = 0u;
    uint32_t       done_mm   = 0u;
    bool           b_pitched = false;

    if (!b_is_armed)
    {
        obstacle_set(OBS_F_ON_RAMP, 0u);
        obstacle_set(OBS_F_RAMP_HOLD, 0u);
        return;
    }

    status_board_read(&snap);
    mclass   = status_board_motion_class(&snap);
    pitch    = status_board_pitch(&snap);
    path_now = status_board_path(&snap);

    b_pitched = (APP_MCLASS_CLIMBING == mclass)
                || (APP_MCLASS_DESCENDING == mclass)
                || (pitch >= OBS_RAMP_PITCH_DDEG)
                || (pitch <= -OBS_RAMP_PITCH_DDEG)
                || (status_board_hump_now(&snap) > 0u);

    if (b_pitched)
    {
        obstacle_set(OBS_F_ON_RAMP, 1u);
        obstacle_set(OBS_F_RAMP_HOLD, OBS_RAMP_HOLD_PINGS);
        obstacle_set(OBS_F_RAMP_MARK, path_now);
    }
    else if ((0u != obstacle_get(OBS_F_ON_RAMP)))
    {
        if (obstacle_get(OBS_F_RAMP_HOLD) > 0u)
        {
            obstacle_set(OBS_F_RAMP_HOLD, obstacle_get(OBS_F_RAMP_HOLD) - 1u);
        }
        done_mm = odometry_counts_to_mm(
            odometry_path_since(path_now, obstacle_get(OBS_F_RAMP_MARK)));
        if ((0u == obstacle_get(OBS_F_RAMP_HOLD))
            && ((done_mm >= OBS_RAMP_CLEAR_MM)
                || (path_now == obstacle_get(OBS_F_RAMP_MARK))))
        {
            obstacle_set(OBS_F_ON_RAMP, 0u);
        }
    }
    else
    {
        /* flat ground */
    }
}

/**
 * obstacle_enter
 * @brief   Changes scanner state and does its entry action.
 * @param   state APP_SCAN_*
 * @context obstacle_task
 */
static void obstacle_enter(uint32_t state)
{
    status_board_obs_t part;

    obstacle_set(OBS_F_STATE, state);
    obstacle_set(OBS_F_FAR, 0u);
    obstacle_set(OBS_F_LOST, 0u);
    obstacle_set(OBS_F_ECHO, APP_DIST_NONE_MM / OBS_MM_STEP);
    obstacle_set(OBS_F_RAMP_LOGGED, 0u);
    obstacle_set(OBS_F_CREEP_LOGGED, 0u);

    if ((APP_SCAN_MONITOR == state) || (APP_SCAN_PASSING == state))
    {
        drv_servo_set_deg(OBS_CENTRE_DEG);
    }
    if (APP_SCAN_PASSING == state)
    {
        obstacle_set(OBS_F_PASS_MARK, obstacle_path_counts());
    }
    if (APP_SCAN_WATCH_SETTLE == state)
    {
        obstacle_set(OBS_F_SETTLE, OBS_WATCH_SETTLE_PINGS);
        obstacle_part_get(&part);
        if (APP_SIDE_LEFT == part.plan_side)
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
        obstacle_set(OBS_F_ECHO, p_rd->distance_mm / OBS_MM_STEP);
        /* Bring-up diagnosis: what started the slow-down.
           cast: distance 0..2047 mm fits int32_t                       */
        app_log("OBSTACLE AHEAD: distance %d mm, slowing down\n",
                (int32_t) p_rd->distance_mm, 0, 0);
    }
}

/**
 * obstacle_step_approach
 * @brief   APPROACH: confirmed under OBS_STOP_MM, or two lost echoes in a
 *          row after an echo already under OBS_STOP_MM (too close to
 *          measure) -> ESTOP first, then OBSTACLE. Two far readings ->
 *          CONTINUE; a lost echo after a farther echo counts as far.
 * @note    On an open track every ping is lost (the echo pulse outlasts
 *          the 40 ms window), so treating every lost ping as "too close"
 *          stopped the car on a clear track after one stray echo under
 *          OBS_SLOW_MM (on-car 2026-11-04, S5-U38). In 2 pings the car
 *          moves <= 36 mm, so from an echo >= OBS_STOP_MM it cannot have
 *          reached the sensor's blind zone.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_approach(obstacle_reading_t const * p_rd)
{
    bool     b_is_far = false;
    uint32_t stop_mm  = OBS_STOP_MM;

    /* Pitched on a ramp (S5-U42): the beam sees the ramp face or the
       floor, so only the early-stop margin is dropped. The computed NFR6
       minimum still stops the car for a real obstacle.                 */
    if ((0u != obstacle_get(OBS_F_ON_RAMP)))
    {
        stop_mm = OBS_RAMP_STOP_MM;
        if ((!(0u != obstacle_get(OBS_F_RAMP_LOGGED))) && p_rd->b_is_confirmed
            && (p_rd->distance_mm >= OBS_RAMP_STOP_MM)
            && (p_rd->distance_mm < OBS_STOP_MM))
        {
            obstacle_set(OBS_F_RAMP_LOGGED, 1u);
            /* casts: distances 0..2047 mm fit int32_t */
            app_log("RAMP: pitched, echo %d mm ignored, stop below %d mm\n",
                    (int32_t) p_rd->distance_mm,
                    (int32_t) OBS_RAMP_STOP_MM, 0);
        }
    }

    if (!p_rd->b_is_lost)
    {
        obstacle_set(OBS_F_ECHO, p_rd->distance_mm / OBS_MM_STEP);
        obstacle_set(OBS_F_LOST, 0u);
        b_is_far  = (p_rd->distance_mm > (OBS_SLOW_MM + OBS_CLEAR_HYST_MM));
    }
    else if ((obstacle_get(OBS_F_ECHO) * OBS_MM_STEP) < stop_mm)
    {
        /* too close to measure */
        obstacle_set(OBS_F_LOST, obstacle_get(OBS_F_LOST) + 1u);
    }
    else
    {
        obstacle_set(OBS_F_LOST, 0u);
        b_is_far = true;                  /* nothing in range */
    }
    obstacle_set(OBS_F_FAR, b_is_far ? (obstacle_get(OBS_F_FAR) + 1u) : 0u);

    if ((p_rd->b_is_confirmed && (p_rd->distance_mm < stop_mm))
        || (obstacle_get(OBS_F_LOST) >= OBS_LOST_PINGS))
    {
        safety_set(SAFETY_ESTOP);                 /* brake first (NFR6) */
        (void) mission_post_event(APP_EVT_OBSTACLE, 0u);   /* counted */
        /* Bring-up diagnosis (after the brake). casts: distance 0..2047
           and lost count 0..3 fit int32_t                              */
        app_log("OBSTACLE STOP (ESTOP): distance %d mm, lost echoes %d\n",
                (int32_t) p_rd->distance_mm,
                (int32_t) obstacle_get(OBS_F_LOST), 0);
        obstacle_enter(APP_SCAN_STOPPED);
    }
    else if (obstacle_get(OBS_F_FAR) >= OBS_CLEAR_READINGS)
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
 *          then take OBS_SCAN_SAMPLES readings and store their median
 *          for the current step (4 mm steps, >= 1020 = clear; S5-U47)
 *          and mark it measured.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_scan(obstacle_reading_t const * p_rd)
{
    bool           b_is_more  = true;
    uint32_t       med        = 0u;
    uint32_t const settle     = obstacle_get(OBS_F_SETTLE);
    uint32_t const sample_cnt = obstacle_get(OBS_F_SAMPLE_CNT);
    uint32_t const step       = obstacle_get(OBS_F_STEP);

    if (settle > 0u)
    {
        obstacle_set(OBS_F_SETTLE, settle - 1u);  /* servo still moving */
    }
    else if (sample_cnt < (OBS_SCAN_SAMPLES - 1u))
    {
        g_samples[sample_cnt] = obstacle_mm_code(
            (p_rd->distance_mm < APP_PROFILE_CLEAR_MM)
            ? p_rd->distance_mm : APP_PROFILE_CLEAR_MM);
        obstacle_set(OBS_F_SAMPLE_CNT, sample_cnt + 1u); /* more pings */
    }
    else
    {
        g_samples[sample_cnt] = obstacle_mm_code(
            (p_rd->distance_mm < APP_PROFILE_CLEAR_MM)
            ? p_rd->distance_mm : APP_PROFILE_CLEAR_MM);
        med = obstacle_median3(g_samples);
        if (OBS_EDGE_OFF != obstacle_get(OBS_F_EDGE_PHASE))
        {
            b_is_more = obstacle_edge_step(med);             /* S5-U83 */
        }
        else
        {
            /* cast: median of three codes <= 255 */
            g_obs_scan[step] = (uint8_t) med;
            obstacle_set(OBS_F_MEASURED,
                         obstacle_get(OBS_F_MEASURED) | (1u << step));
            if (APP_SCAN_COARSE == obstacle_get(OBS_F_STATE))
            {
                if ((step + OBS_COARSE_STRIDE) < APP_SCAN_STEPS)
                {
                    obstacle_set_step(step + OBS_COARSE_STRIDE);
                }
                else
                {
                    obstacle_set(OBS_F_STATE, APP_SCAN_FINE);
                    b_is_more = obstacle_next_fine();
                }
            }
            else
            {
                b_is_more = obstacle_next_fine();
            }
            if (!b_is_more)
            {
                b_is_more = obstacle_edge_begin();   /* same scan: edges */
            }
        }
    }

    if (!b_is_more)
    {
        obstacle_scan_finish();
    }
}

/**
 * obstacle_step_creep
 * @brief   STOPPED, not scanning (S5-U55): while the mission creeps the car
 *          forward to OBS_SCAN_MM, brake (ESTOP) on a confirmed reading at
 *          or below OBS_SCAN_MM, a lost echo, or a reading that disagrees
 *          with the previous one. Standing still or reversing: nothing.
 * @param   p_rd this cycle's reading
 * @context obstacle_task
 */
static void obstacle_step_creep(obstacle_reading_t const * p_rd)
{
    status_board_t snap = {0};

    status_board_read(&snap);
    if ((status_board_speed_l(&snap) > 0) && (status_board_speed_r(&snap) > 0)
        && (p_rd->b_is_lost || (!p_rd->b_is_confirmed)
            || (p_rd->distance_mm <= OBS_SCAN_MM)))
    {
        safety_set(SAFETY_ESTOP);
        if (!(0u != obstacle_get(OBS_F_CREEP_LOGGED)))
        {
            obstacle_set(OBS_F_CREEP_LOGGED, 1u);
            /* casts: 0..2047 mm and flags fit int32_t */
            app_log("CREEP stop at %d mm (lost %d, agrees %d)\n",
                    (int32_t) p_rd->distance_mm, p_rd->b_is_lost ? 1 : 0,
                    p_rd->b_is_confirmed ? 1 : 0);
        }
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
    status_board_obs_t part;

    if (obstacle_get(OBS_F_SETTLE) > 0u)
    {
        obstacle_set(OBS_F_SETTLE, obstacle_get(OBS_F_SETTLE) - 1u);
    }
    if (0u == obstacle_get(OBS_F_SETTLE))
    {
        obstacle_part_get(&part);
        (void) mission_post_event(APP_EVT_PLAN, part.plan_side); /* cnt */
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
    status_board_obs_t part;
    uint32_t           need_mm = 0u;
    uint32_t           done_mm = 0u;

    obstacle_part_get(&part);
    need_mm = part.nearest_mm + OBS_DEPTH_ASSUMED_MM + OBS_ROBOT_LEN_MM;
    done_mm = odometry_counts_to_mm(
        odometry_path_since(obstacle_path_counts(),
                            obstacle_get(OBS_F_PASS_MARK)));

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
    status_board_obs_t part;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        g_obs_scan[idx] = obstacle_mm_code(APP_PROFILE_CLEAR_MM);
    }
    obstacle_set(OBS_F_MEASURED, 0u);
    obstacle_set(OBS_F_INFERRED, 0u);
    g_obs_prv_e = 0u;          /* no edge pass yet (EDGE_PHASE = OFF) */
    obstacle_set(OBS_F_EDGE_LO_D, 0u);
    obstacle_set(OBS_F_EDGE_HI_D, 0u);
    obstacle_part_get(&part);
    part.b_is_profile_valid = false;
    obstacle_part_put(&part);
    g_scan_start_us           = sys_tick_now_us();
    obstacle_set(OBS_F_STATE, APP_SCAN_COARSE);
    obstacle_set(OBS_F_STEP, OBS_STEP_CENTRE);   /* servo at 90 deg */
    obstacle_set_step(0u);
}

/**
 * obstacle_next_fine
 * @brief   Moves to the next unmeasured 5-degree step between two coarse
 *          points of which at least one is near (S5-U50: every step
 *          beside an object is measured, none inferred; was: only where
 *          the near/far class differs).
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
            && (0u == (obstacle_get(OBS_F_MEASURED) & (1u << idx))))
        {
            b_near  = (obstacle_scan_mm(coarse) < OBS_NEAR_MM);
            b_near2 = (obstacle_scan_mm(coarse + OBS_COARSE_STRIDE)
                       < OBS_NEAR_MM);
            if (b_near || b_near2)
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
    obs_profile_t      prof = {0};
    obs_plan_t         plan = {OBS_OUT_NO_PATH, APP_SIDE_LEFT, 0u, 0u, false};
    obs_edge_t         edge;
    status_board_obs_t part;
    uint16_t           scan_mm[APP_SCAN_STEPS];
    uint32_t           idx  = 0u;
    uint32_t           est  = 0u;
    uint32_t           fine = 0u;
    uint32_t           pm_mm = 0u;

    obstacle_fill_gaps();
    for (idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        /* cast: <= 1020 mm */
        scan_mm[idx] = (uint16_t) obstacle_scan_mm(idx);
    }
    obs_profile_build(scan_mm, obstacle_get(OBS_F_INFERRED), &prof);
    obs_profile_plan(&prof, &plan);
    /* S5-U83: the reported width is the 1-degree edge measurement; the
       plan above still comes from the 5-degree estimate.             */
    est  = prof.width_mm;
    obstacle_edge_load(&edge);
    fine = obs_edge_width(&edge, &pm_mm);    /* 0: a corner not seen */
    /* Reported width: the corner measurement or unavailable (1020 =
       clear on the board, "width unavailable" on the dashboard); never
       the 5-degree estimate under the 1-degree label.               */
    prof.width_mm = ((fine > 0u) && (fine < APP_PROFILE_CLEAR_MM))
                    ? fine : APP_PROFILE_CLEAR_MM;
    obstacle_set(OBS_F_EDGE_PHASE, OBS_EDGE_OFF);
    /* S5-U42: a scan that saw only a narrow normal-incidence glint
       (< OBS_MIN_WIDTH_MM) off a sloped ramp face is a ramp: drive on
       (PASSING). The pitch flag alone never overrides the plan, so a
       real obstacle (>= 80 mm, A29) still gets PLAN or NO_PATH (reverse
       and rescan), also when the car stopped on the ramp.              */
    if ((OBS_OUT_CONTINUE != plan.outcome)
        && prof.b_is_valid && (prof.raw_width_mm < OBS_MIN_WIDTH_MM))
    {
        /* casts: width 0..1020 mm, flag 0..1 fit int32_t */
        app_log("RAMP/GLINT in scan (width %d mm, on_ramp %d): continuing\n",
                (int32_t) prof.raw_width_mm,
                (0u != obstacle_get(OBS_F_ON_RAMP)) ? 1 : 0, 0);
        plan.outcome       = OBS_OUT_CONTINUE;
        plan.b_was_clamped = false;
    }
    if (plan.b_was_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    /* stored in 4 mm steps (same information as the packed build);
       legs round up so a rounded leg is never shorter than planned */
    obstacle_part_get(&part);
    part.b_is_profile_valid = prof.b_is_valid;
    part.angle_step         = prof.angle_step;
    part.width_mm           = obstacle_round_mm(prof.width_mm);
    part.nearest_mm         = obstacle_round_mm(prof.nearest_mm);
    part.clear_left_mm      = obstacle_round_mm(prof.clear_left_mm);
    part.clear_right_mm     = obstacle_round_mm(prof.clear_right_mm);
    part.plan_side          = plan.side;
    part.leg_lateral_mm     = ((plan.leg_lateral_mm + OBS_MM_STEP - 1u)
                               / OBS_MM_STEP) * OBS_MM_STEP;
    part.leg_forward_mm     = ((plan.leg_forward_mm + OBS_MM_STEP - 1u)
                               / OBS_MM_STEP) * OBS_MM_STEP;
    part.scanner_state      = obstacle_get(OBS_F_STATE);
    obstacle_part_put(&part);
    obstacle_part_get(&part);       /* as stored: the board's 4 mm codes */
    status_board_write_obstacle(&part);           /* board before event */

    /* casts: scan ms, outcome and the 25-bit mask all fit int32_t */
    app_log("SCAN %d ms out %d inferred 0x%x\n",
            (int32_t) ((sys_tick_now_us() - g_scan_start_us) / OBS_US_PER_MS),
            (int32_t) plan.outcome, (int32_t) obstacle_get(OBS_F_INFERRED));
    obstacle_scan_log();
    obstacle_ranges_log();
    if (prof.b_is_valid)
    {
        /* S5-U45: estimate and the measured span (its uncertainty).
           casts: <= 1020 mm fit int32_t                               */
        app_log("SCAN width estimate %d mm (measured span %d mm, kept "
                "within %d..200 mm)\n",
                (int32_t) est, (int32_t) prof.raw_width_mm,
                (int32_t) OBS_WIDTH_MIN_MM);
        /* S5-U83 (calibration: S5-I09). casts: <= 1500 fit int32_t */
        app_log("EDGE low %d deg at %d mm, high %d deg\n",
                (int32_t) edge.lo_deg,
                (int32_t) (edge.lo_code * OBS_MM_STEP),
                (int32_t) edge.hi_deg);
        if (APP_PROFILE_CLEAR_MM != prof.width_mm)
        {
            app_log("WIDTH %d mm +-%d mm (high edge at %d mm)\n",
                    (int32_t) prof.width_mm, (int32_t) pm_mm,
                    (int32_t) (edge.hi_code * OBS_MM_STEP));
        }
        else
        {
            /* casts: flags 0..1 */
            app_log("WIDTH unavailable (edge seen: low %d, high %d)\n",
                    edge.b_ok_lo ? 1 : 0, edge.b_ok_hi ? 1 : 0, 0);
        }
        obstacle_block_log(true, prof.block_step_left, prof.clear_left_mm);
        obstacle_block_log(false, prof.block_step_right,
                           prof.clear_right_mm);
    }

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
 * obstacle_scan_log
 * @brief   Bring-up diagnosis: the nearest measured scan point, its servo
 *          angle and which steps counted as near. A constant short echo
 *          at 30 or 150 deg on a clear track means the sensor sees part
 *          of the car there.
 * @context obstacle_task (once per scan)
 */
static void obstacle_scan_log(void)
{
    uint32_t nearest = APP_PROFILE_CLEAR_MM;
    uint32_t at_step = 0u;
    uint32_t near_mask = 0u;

    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        if (0u != (obstacle_get(OBS_F_MEASURED) & (1u << idx)))
        {
            if (obstacle_scan_mm(idx) < nearest)
            {
                nearest = obstacle_scan_mm(idx);
                at_step = idx;
            }
            if (obstacle_scan_mm(idx) < OBS_NEAR_MM)
            {
                near_mask |= (1u << idx);
            }
        }
    }

    /* casts: <= 1020 mm, 30..150 deg and a 25-bit mask fit int32_t */
    app_log("SCAN nearest %d mm at %d deg, near steps 0x%x (step n = "
            "30 + 5n deg)\n",
            (int32_t) nearest,
            (int32_t) (APP_SCAN_ANGLE_MIN_DEG
                       + (APP_SCAN_ANGLE_STEP_DEG * at_step)),
            (int32_t) near_mask);
}

/**
 * obstacle_ranges_log
 * @brief   Bring-up diagnosis (S5-U47): every step's stored range, three
 *          steps per line (inferred steps show their filled value).
 * @context obstacle_task (once per scan)
 */
static void obstacle_ranges_log(void)
{
    static char const * const p_fmt[OBS_RANGE_LINES] =
    {
        "SCAN mm 30/35/40 deg: %d %d %d\n",
        "SCAN mm 45/50/55 deg: %d %d %d\n",
        "SCAN mm 60/65/70 deg: %d %d %d\n",
        "SCAN mm 75/80/85 deg: %d %d %d\n",
        "SCAN mm 90/95/100 deg: %d %d %d\n",
        "SCAN mm 105/110/115 deg: %d %d %d\n",
        "SCAN mm 120/125/130 deg: %d %d %d\n",
        "SCAN mm 135/140/145 deg: %d %d %d\n",
        "SCAN mm 150 deg: %d\n"
    };
    uint32_t step = 0u;

    for (uint32_t line = 0u; line < OBS_RANGE_LINES; line++)
    {
        step = line * OBS_RANGE_PER_LINE;
        if ((step + OBS_RANGE_PER_LINE) <= APP_SCAN_STEPS)
        {
            /* casts: <= 1020 mm fit int32_t */
            app_log(p_fmt[line], (int32_t) obstacle_scan_mm(step),
                    (int32_t) obstacle_scan_mm(step + 1u),
                    (int32_t) obstacle_scan_mm(step + 2u));
        }
        else
        {
            /* cast: <= 1020 mm fits int32_t */
            app_log(p_fmt[line], (int32_t) obstacle_scan_mm(step), 0, 0);
        }
    }
}

/**
 * obstacle_block_log
 * @brief   Bring-up diagnosis (S5-U49): which return limits the free gap
 *          on one side (none: nothing printed).
 * @param   b_is_left true = left side, false = right side
 * @param   step      blocking step, APP_SCAN_STEPS = none
 * @param   gap_mm    free gap on that side
 * @context obstacle_task (once per scan)
 */
static void obstacle_block_log(bool b_is_left, uint32_t step,
                               uint32_t gap_mm)
{
    /* casts: <= 1020 mm and 30..150 deg fit int32_t */
    int32_t const deg = (int32_t) (APP_SCAN_ANGLE_MIN_DEG
                                   + (APP_SCAN_ANGLE_STEP_DEG * step));

    if (step < APP_SCAN_STEPS)
    {
        if (b_is_left)
        {
            app_log("SCAN left gap %d mm, set by the return at %d deg"
                    " (%d mm)\n", (int32_t) gap_mm, deg,
                    (int32_t) obstacle_scan_mm(step));
        }
        else
        {
            app_log("SCAN right gap %d mm, set by the return at %d deg"
                    " (%d mm)\n", (int32_t) gap_mm, deg,
                    (int32_t) obstacle_scan_mm(step));
        }
    }
}

/**
 * obstacle_get
 * @brief   Reads one private field (PRV_OBS_* rows).
 * @param   field OBS_F_*
 * @return  field value (4 mm code for OBS_F_PREV / OBS_F_ECHO)
 * @context obstacle_task
 */
static uint32_t obstacle_get(uint32_t field)
{
    obstacle_field_t const * const p_f = &g_obs_f[field];

    return app_bits_get(*p_f->p_word, p_f->pos, p_f->width);
}

/**
 * obstacle_set
 * @brief   Writes one private field. Callers keep every value inside its
 *          row range (build checks above tie the rows to the cfg limits).
 * @param   field OBS_F_*
 * @param   value new value
 * @context obstacle_task
 */
static void obstacle_set(uint32_t field, uint32_t value)
{
    obstacle_field_t const * const p_f = &g_obs_f[field];

    *p_f->p_word = app_bits_put(*p_f->p_word, p_f->pos, p_f->width, value);
}

/**
 * obstacle_mm_code
 * @brief   A scan distance (a multiple of 4 mm, <= APP_PROFILE_CLEAR_MM)
 *          as its 4 mm code (PRV_OBS_SCAN / PRV_OBS_SMP).
 * @param   distance_mm 0..1020, a multiple of 4
 * @return  code 0..255
 * @context obstacle_task, usermain (init)
 */
static uint8_t obstacle_mm_code(uint32_t distance_mm)
{
    return (uint8_t) (distance_mm / OBS_MM_STEP); /* cast: <= 255 */
}

/**
 * obstacle_scan_mm
 * @brief   One scan table point in mm.
 * @param   step 0..APP_SCAN_STEPS - 1
 * @return  0..1020 mm (1020 = clear)
 * @context obstacle_task
 */
static uint32_t obstacle_scan_mm(uint32_t step)
{
    return ((uint32_t) g_obs_scan[step] * OBS_MM_STEP);
}

/**
 * obstacle_part_mm
 * @brief   A distance as the board stores it: rounded to 4 mm and
 *          saturated at the field maximum (same rule as sb_put_mm).
 * @param   distance_mm mm
 * @param   max         field maximum (code)
 * @return  code
 * @context obstacle_task
 */
static uint32_t obstacle_part_mm(uint32_t distance_mm, uint32_t max)
{
    return app_bits_clamp_u((distance_mm + (OBS_MM_STEP / 2u))
                            / OBS_MM_STEP, max, NULL);
}

/**
 * obstacle_part_put
 * @brief   Stores the obstacle board part in g_part_w, in the SB_OBS0..2
 *          layout with the board's rounding, so reading it back gives
 *          exactly what the board holds.
 * @param   p_part part in natural units
 * @context obstacle_task
 */
static void obstacle_part_put(status_board_obs_t const * p_part)
{
    uint32_t w0 = 0u;
    uint32_t w1 = 0u;
    uint32_t w2 = 0u;

    w0 = app_bits_put(w0, SB_OBS0_DIST_POS, SB_OBS0_DIST_BITS,
                      obstacle_part_mm(p_part->distance_mm,
                                       SB_OBS0_DIST_MAX));
    w0 = app_bits_put(w0, SB_OBS0_SCAN_STATE_POS, SB_OBS0_SCAN_STATE_BITS,
                      p_part->scanner_state);
    w0 = app_bits_put(w0, SB_OBS0_ANGLE_STEP_POS, SB_OBS0_ANGLE_STEP_BITS,
                      p_part->angle_step);
    w0 = app_bits_put(w0, SB_OBS0_PLAN_SIDE_POS, SB_OBS0_PLAN_SIDE_BITS,
                      p_part->plan_side);
    w0 = app_bits_put(w0, SB_OBS0_PROFILE_VALID_POS,
                      SB_OBS0_PROFILE_VALID_BITS,
                      p_part->b_is_profile_valid ? 1u : 0u);
    w1 = app_bits_put(w1, SB_OBS1_WIDTH_POS, SB_OBS1_WIDTH_BITS,
                      obstacle_part_mm(p_part->width_mm, SB_OBS1_WIDTH_MAX));
    w1 = app_bits_put(w1, SB_OBS1_NEAREST_POS, SB_OBS1_NEAREST_BITS,
                      obstacle_part_mm(p_part->nearest_mm,
                                       SB_OBS1_NEAREST_MAX));
    w1 = app_bits_put(w1, SB_OBS1_CLEAR_L_POS, SB_OBS1_CLEAR_L_BITS,
                      obstacle_part_mm(p_part->clear_left_mm,
                                       SB_OBS1_CLEAR_L_MAX));
    w1 = app_bits_put(w1, SB_OBS1_CLEAR_R_POS, SB_OBS1_CLEAR_R_BITS,
                      obstacle_part_mm(p_part->clear_right_mm,
                                       SB_OBS1_CLEAR_R_MAX));
    w2 = app_bits_put(w2, SB_OBS2_LEG_LAT_POS, SB_OBS2_LEG_LAT_BITS,
                      obstacle_part_mm(p_part->leg_lateral_mm,
                                       SB_OBS2_LEG_LAT_MAX));
    w2 = app_bits_put(w2, SB_OBS2_LEG_FWD_POS, SB_OBS2_LEG_FWD_BITS,
                      obstacle_part_mm(p_part->leg_forward_mm,
                                       SB_OBS2_LEG_FWD_MAX));
    g_part_w[0] = w0;
    g_part_w[1] = w1;
    g_part_w[2] = w2;
}

/**
 * obstacle_part_get
 * @brief   The obstacle board part as stored in g_part_w, natural units.
 * @param   p_part part out
 * @context obstacle_task
 */
static void obstacle_part_get(status_board_obs_t * p_part)
{
    uint32_t const w0 = g_part_w[0];
    uint32_t const w1 = g_part_w[1];
    uint32_t const w2 = g_part_w[2];

    p_part->distance_mm = app_bits_get(w0, SB_OBS0_DIST_POS,
                                       SB_OBS0_DIST_BITS) * OBS_MM_STEP;
    p_part->scanner_state = app_bits_get(w0, SB_OBS0_SCAN_STATE_POS,
                                         SB_OBS0_SCAN_STATE_BITS);
    p_part->angle_step = app_bits_get(w0, SB_OBS0_ANGLE_STEP_POS,
                                      SB_OBS0_ANGLE_STEP_BITS);
    p_part->plan_side = app_bits_get(w0, SB_OBS0_PLAN_SIDE_POS,
                                     SB_OBS0_PLAN_SIDE_BITS);
    p_part->b_is_profile_valid =
        (0u != app_bits_get(w0, SB_OBS0_PROFILE_VALID_POS,
                            SB_OBS0_PROFILE_VALID_BITS));
    p_part->width_mm = app_bits_get(w1, SB_OBS1_WIDTH_POS,
                                    SB_OBS1_WIDTH_BITS) * OBS_MM_STEP;
    p_part->nearest_mm = app_bits_get(w1, SB_OBS1_NEAREST_POS,
                                      SB_OBS1_NEAREST_BITS) * OBS_MM_STEP;
    p_part->clear_left_mm = app_bits_get(w1, SB_OBS1_CLEAR_L_POS,
                                         SB_OBS1_CLEAR_L_BITS) * OBS_MM_STEP;
    p_part->clear_right_mm = app_bits_get(w1, SB_OBS1_CLEAR_R_POS,
                                          SB_OBS1_CLEAR_R_BITS)
                             * OBS_MM_STEP;
    p_part->leg_lateral_mm = app_bits_get(w2, SB_OBS2_LEG_LAT_POS,
                                          SB_OBS2_LEG_LAT_BITS)
                             * OBS_MM_STEP;
    p_part->leg_forward_mm = app_bits_get(w2, SB_OBS2_LEG_FWD_POS,
                                          SB_OBS2_LEG_FWD_BITS)
                             * OBS_MM_STEP;
}

/**
 * obstacle_median3
 * @brief   Median of three readings (S5-U47).
 * @param   p_val three values
 * @return  the middle one
 * @context any
 */
static uint32_t obstacle_median3(uint8_t const * p_val)
{
    uint32_t const val_a = p_val[0];
    uint32_t const val_b = p_val[1];
    uint32_t const val_c = p_val[2];
    uint32_t       mid   = val_a;

    if (((val_b >= val_a) && (val_b <= val_c))
        || ((val_b <= val_a) && (val_b >= val_c)))
    {
        mid = val_b;
    }
    else if (((val_c >= val_a) && (val_c <= val_b))
             || ((val_c <= val_a) && (val_c >= val_b)))
    {
        mid = val_c;
    }
    else
    {
        /* val_a is the middle value */
    }

    return mid;
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
        if (0u == (obstacle_get(OBS_F_MEASURED) & (1u << idx)))
        {
            left  = idx;
            right = idx;
            while ((left > 0u)
                   && (0u == (obstacle_get(OBS_F_MEASURED) & (1u << left))))
            {
                left--;
            }
            while ((right < (APP_SCAN_STEPS - 1u))
                   && (0u == (obstacle_get(OBS_F_MEASURED) & (1u << right))))
            {
                right++;
            }
            g_obs_scan[idx] = (g_obs_scan[left] < g_obs_scan[right])
                              ? g_obs_scan[left] : g_obs_scan[right];
            /* codes keep the order of the distances they stand for */
            obstacle_set(OBS_F_INFERRED,
                         obstacle_get(OBS_F_INFERRED) | (1u << idx));
        }
    }
    obstacle_set(OBS_F_INFERRED, obstacle_get(OBS_F_INFERRED) & OBS_MASK_ALL);
}

/**
 * obstacle_set_step
 * @brief   Points the servo at a scan step and starts its settle time.
 * @param   step 0..24 (angle 30 + 5 x step)
 * @context obstacle_task (robot stopped)
 */
static void obstacle_set_step(uint32_t step)
{
    uint32_t const now   = obstacle_get(OBS_F_STEP);
    uint32_t const moved = (step > now) ? (step - now) : (now - step);

    /* S5-U52: short moves (<= one coarse stride) settle in one ping */
    obstacle_set(OBS_F_SETTLE, (moved <= OBS_COARSE_STRIDE)
                               ? OBS_SETTLE_SMALL_PINGS : OBS_SETTLE_PINGS);
    obstacle_set(OBS_F_STEP, step);
    obstacle_set(OBS_F_SAMPLE_CNT, 0u);
    drv_servo_set_deg(APP_SCAN_ANGLE_MIN_DEG
                      + (APP_SCAN_ANGLE_STEP_DEG * step));
}

/**
 * obstacle_edge_begin
 * @brief   End of the 5-degree points (S5-U83): builds the profile the
 *          way obstacle_scan_finish() will, and starts the 1-degree edge
 *          pass (obs_edge.c) from its core.
 * @return  true if an edge pass was started, false if the scan is done
 * @context obstacle_task
 */
static bool obstacle_edge_begin(void)
{
    obs_profile_t    prof = {0};
    obs_edge_t       edge;
    obs_edge_probe_t next = {0u, false};
    uint16_t         scan_mm[APP_SCAN_STEPS];
    bool             b_is_more = false;

    obstacle_fill_gaps();
    for (uint32_t idx = 0u; idx < APP_SCAN_STEPS; idx++)
    {
        /* cast: <= 1020 mm */
        scan_mm[idx] = (uint16_t) obstacle_scan_mm(idx);
    }
    obs_profile_build(scan_mm, obstacle_get(OBS_F_INFERRED), &prof);

    b_is_more = obs_edge_begin(&edge, g_obs_scan, &prof, &next);
    obstacle_edge_store(&edge);
    if (b_is_more)
    {
        obstacle_edge_probe(&next);
    }

    return b_is_more;
}

/**
 * obstacle_edge_step
 * @brief   One edge-pass probe done: hands its median to obs_edge.c and
 *          points the servo where it asks next.
 * @param   code probe median, 4 mm code
 * @return  true while probes remain, false when the edge pass is done
 * @context obstacle_task
 */
static bool obstacle_edge_step(uint32_t code)
{
    obs_edge_t       edge;
    obs_edge_probe_t next = {0u, false};
    bool             b_is_more = false;

    obstacle_edge_load(&edge);
    b_is_more = obs_edge_sample(&edge, code, g_obs_scan, &next);
    obstacle_edge_store(&edge);
    if (b_is_more)
    {
        obstacle_edge_probe(&next);
    }

    return b_is_more;
}

/**
 * obstacle_edge_load
 * @brief   Unpacks the edge-pass state (PRV_OBS_E, PRV_OBS_C). The corner
 *          echoes are stored as their depth below the band limit.
 * @param   p_edge out: working copy
 * @context obstacle_task
 */
static void obstacle_edge_load(obs_edge_t * p_edge)
{
    uint32_t const limit = obstacle_get(OBS_F_EDGE_LIMIT);
    uint32_t const lo_d  = obstacle_get(OBS_F_EDGE_LO_D);
    uint32_t const hi_d  = obstacle_get(OBS_F_EDGE_HI_D);

    p_edge->phase      = obstacle_get(OBS_F_EDGE_PHASE);
    p_edge->sub        = obstacle_get(OBS_F_EDGE_SUB);
    p_edge->lo_deg     = obstacle_get(OBS_F_EDGE_LO);
    p_edge->hi_deg     = obstacle_get(OBS_F_EDGE_HI);
    p_edge->limit_code = limit;
    p_edge->lo_code    = (lo_d <= limit) ? (limit - lo_d) : 0u;
    p_edge->hi_code    = (hi_d <= limit) ? (limit - hi_d) : 0u;
    p_edge->b_ok_lo    = (0u != obstacle_get(OBS_F_EDGE_OK_LO));
    p_edge->b_ok_hi    = (0u != obstacle_get(OBS_F_EDGE_OK_HI));
    p_edge->b_grace    = (0u != obstacle_get(OBS_F_EDGE_GRACE));
}

/**
 * obstacle_edge_store
 * @brief   Packs the edge-pass state (PRV_OBS_E, PRV_OBS_C). Corner
 *          echoes are stored as their depth below EDGE_LIMIT, clamped to
 *          the field maximum.
 * @param   p_edge working copy
 * @context obstacle_task
 */
static void obstacle_edge_store(obs_edge_t const * p_edge)
{
    uint32_t const limit = p_edge->limit_code;
    uint32_t       lo_d  = (p_edge->lo_code <= limit)
                           ? (limit - p_edge->lo_code) : 0u;
    uint32_t       hi_d  = (p_edge->hi_code <= limit)
                           ? (limit - p_edge->hi_code) : 0u;

    lo_d = (lo_d < PRV_OBS_C_EDGE_LO_D_MAX) ? lo_d : PRV_OBS_C_EDGE_LO_D_MAX;
    hi_d = (hi_d < PRV_OBS_C_EDGE_HI_D_MAX) ? hi_d : PRV_OBS_C_EDGE_HI_D_MAX;
    obstacle_set(OBS_F_EDGE_PHASE, p_edge->phase);
    obstacle_set(OBS_F_EDGE_SUB, p_edge->sub);
    obstacle_set(OBS_F_EDGE_LO, p_edge->lo_deg);
    obstacle_set(OBS_F_EDGE_HI, p_edge->hi_deg);
    obstacle_set(OBS_F_EDGE_LIMIT, limit);
    obstacle_set(OBS_F_EDGE_LO_D, lo_d);
    obstacle_set(OBS_F_EDGE_HI_D, hi_d);
    obstacle_set(OBS_F_EDGE_OK_LO, p_edge->b_ok_lo ? 1u : 0u);
    obstacle_set(OBS_F_EDGE_OK_HI, p_edge->b_ok_hi ? 1u : 0u);
    obstacle_set(OBS_F_EDGE_GRACE, p_edge->b_grace ? 1u : 0u);
}

/**
 * obstacle_edge_probe
 * @brief   Points the servo at the next edge-pass angle (the 5-degree
 *          STEP is unchanged). The first probe of a side follows a long
 *          swing and waits OBS_WATCH_SETTLE_PINGS, the others one ping.
 * @param   p_next probe from obs_edge.c
 * @context obstacle_task
 */
static void obstacle_edge_probe(obs_edge_probe_t const * p_next)
{
    obstacle_set(OBS_F_SETTLE, p_next->b_is_far ? OBS_WATCH_SETTLE_PINGS
                                                : OBS_SETTLE_SMALL_PINGS);
    obstacle_set(OBS_F_SAMPLE_CNT, 0u);
    drv_servo_set_deg(p_next->deg);
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
