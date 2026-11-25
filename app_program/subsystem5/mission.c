/**
 * @file    mission.c
 * @brief   Mission queue K4, mode flags K10 and mission_task: the system
 *          statechart (Figure 5a). RUN substates live in mission_run.c.
 *          Packed state between passes (PRV_MIS_* words, E5 timers).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR11 (buttons), FR13, FR14, FR16, FR17, NFR10b, min-bit item 7
 */

#include "mission.h"           /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem1/telem.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/odometry.h"
#include "../subsystem3/barcode.h"
#include "cfg_mission.h"
#include "mission_run.h"
#include "status_board.h"
#include "sys_health.h"
#include "sys_tick.h"
#include "hmi.h"

/* One packer serves BARCODE and MAP events: their fields must match. */
#if (EVT_MAP_CHAR_POS != EVT_BARCODE_CHAR_POS)                          \
    || (EVT_MAP_CHAR_BITS != EVT_BARCODE_CHAR_BITS)                     \
    || (EVT_MAP_CMD_POS != EVT_BARCODE_CMD_POS)                         \
    || (EVT_MAP_CMD_BITS != EVT_BARCODE_CMD_BITS)
#error "EVT_MAP and EVT_BARCODE layouts differ; split mission_evt_barcode_value"
#endif

/* ---- private constants and types ------------------------------------ */

/* K4 message: one MSG_EVT 16-bit word (Tribunal 1 min-bit change,
   gap 3): 11-bit value, 5-bit type. Kernel cost per entry: 4-byte
   header + 2 B rounded up to 4 = 8 B (was 16 B with the 12-byte
   natural-type struct), so 16 entries take 128 B (was 256 B).        */
#define MISSION_K4_ENTRIES     (16u)
#define MISSION_EVT_MSG_BYTES  (2u)    /* one MSG_EVT word */
#define MISSION_EVT_SLOT_BYTES (4u)    /* 2 B rounded up to 4 */
#define MISSION_MBF_HDR_BYTES  (4u)    /* kernel header per message */
#define MISSION_MBF_BYTES      (MISSION_K4_ENTRIES                     \
                                * (MISSION_MBF_HDR_BYTES               \
                                   + MISSION_EVT_SLOT_BYTES))

#if (MSG_EVT_USED_BITS != 16u) \
    || (APP_EVT_COUNT > (1u << MSG_EVT_TYPE_BITS))
#error "K4 event no longer fits the 16-bit MSG_EVT word"
#endif
#if (APP_EVT_COUNT > (MSG_EVT_TYPE_MAX + 1))
#error "tools/layout.csv MSG_EVT TYPE max is below the last APP_EVT_* code"
#endif
#define MISSION_WAKE_TMO_MS    (20)    /* backup if an R2 wake is lost  */
#define MISSION_EVTS_PER_WAKE  (8u)    /* bounds one wake's work        */
#define MISSION_STACK_WORDS    (APP_STACK_MISSION_BYTES / 8u)

#define MISSION_US_PER_MS      (1000u)
#define MISSION_US_PER_S       (1000000u)
#define MISSION_HMI_PERIOD_US  (10000u)  /* buttons polled every 10 ms */
#define MISSION_ODO_MAX_MM     (32767u)  /* K7_SUMM ODO_MM (A23)       */
#define MISSION_RUN_S_MAX      (1023u)   /* K7_SUMM RUN_S (A28)        */
/* Run odometer cap: the first count whose mm reach MISSION_ODO_MAX_MM. */
#define MISSION_ODO_MAX_COUNTS (((MISSION_ODO_MAX_MM * 1000u)              \
                                 / APP_UM_PER_COUNT) + 1u)
#define MISSION_STATE_RUN_SHIFT (3u)     /* STATE_CHANGE: sys | run<<3 */

/* Packed word indices in g_mis_w[] (matching PRV_MIS_* layout groups). */
#define MIS_W_A                (0u)    /* PRV_MIS_A */
#define MIS_W_B                (1u)    /* PRV_MIS_B */
#define MIS_W_C                (2u)    /* PRV_MIS_C */
#define MIS_W_D                (3u)    /* PRV_MIS_D */
#define MIS_W_E                (4u)    /* PRV_MIS_E */
#define MIS_W_F                (5u)    /* PRV_MIS_F */
#define MIS_W_H                (6u)    /* PRV_MIS_H (G belongs to hmi.c) */
#define MIS_W_J                (7u)    /* PRV_MIS_J */
#define MIS_W_K                (8u)    /* PRV_MIS_K */
#define MIS_W_L                (9u)    /* PRV_MIS_L */
#define MIS_W_M                (10u)   /* PRV_MIS_M */
#define MIS_W_N                (11u)   /* PRV_MIS_N */
#define MIS_W_P                (12u)   /* PRV_MIS_P */
#define MIS_W_Q                (13u)   /* PRV_MIS_Q */
#define MIS_W_R                (14u)   /* PRV_MIS_R */
#define MISSION_CTX_WORDS      (15u)

/* E5 32-bit timer indices in g_mis_us[]. */
#define MIS_US_BARCODE         (0u)
#define MIS_US_CAL             (1u)
#define MIS_US_SWEEP           (2u)
#define MIS_US_WARN            (3u)
#define MIS_US_SEARCH          (4u)
#define MIS_US_FR9             (5u)
#define MIS_US_RESUME          (6u)
#define MIS_US_RUN_START       (7u)
#define MIS_US_RUN_END         (8u)
#define MIS_US_MOVE            (9u)
#define MIS_US_RUN_SET         (10u)
#define MIS_US_RECOVER         (11u)
#define MISSION_CTX_TIMERS     (12u)
/* stored code of MISSION_REQ_NONE */
#define MISSION_REQ_CODE_NONE  (APP_SYS_FAULT + 1u)

#if (MISSION_REQ_CODE_NONE != PRV_MIS_C_SYS_REQUEST_MAX)                \
    || (MISSION_ODO_MAX_COUNTS > PRV_MIS_N_RUN_ODO_MAX)                  \
    || (MISSION_BARCODE_REWIND_MM != PRV_MIS_C_BAR_BACK_LEFT_MAX)        \
    || (MISSION_BARCODE_RETRIES != PRV_MIS_J_BAR_TRIES_MAX)              \
    || (MISSION_CAL_CORR_MAX != PRV_MIS_J_CAL_CORR_CNT_MAX)              \
    || (APP_FAULT_ALL != PRV_MIS_B_LAST_FAULTS_MAX)                      \
    || (SB_MOT3_PATH_MAX != PRV_MIS_D_BAR_MARK_MAX)                      \
    || (MISSION_MV_ZERO != PRV_MIS_H_RETRY_KIND_MAX)                     \
    || ((SB_OBS2_LEG_LAT_MAX * 4) != PRV_MIS_E_LEG_LAT_MAX)              \
    || ((SB_OBS2_LEG_FWD_MAX * 4) != PRV_MIS_F_LEG_FWD_MAX)              \
    || (((PRV_MIS_E_LEG_LAT_MAX * 1414) / 1000) != PRV_MIS_A_REJOIN_E_MAX)
#error "A PRV_MIS_* row no longer covers the value it stores"
#endif
#if ((((MOTION_MOVE_MAX_MM * 1000) / MOTION_MOVE_SPEED_MM_S) * 3) / 2   \
     + MISSION_MOVE_MARGIN_MS) != PRV_MIS_D_MOVE_LIMIT_MS_MAX
#error "PRV_MIS_D MOVE_LIMIT must hold the longest move deadline"
#endif

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
/* Mission state between passes, packed (PRV_MIS_A..R rows) plus the E5
   timer images (E_MIS_RUN). Each pass unpacks it into mission_task's
   local mission_ctx_t, runs, and packs it back: the natural-type struct
   exists only during a pass (like the status board part structs).    */
static uint32_t g_mis_w[MISSION_CTX_WORDS];
static uint32_t g_mis_us[MISSION_CTX_TIMERS];

/* Console names of the APP_FAULT_IDX_* bits (bring-up diagnosis: the
   fault mask otherwise only leaves the car over MQTT). No % in them,
   because each is passed to app_log() as the format.                 */
static char const * const g_fault_names[APP_FAULT_COUNT] =
{
    "FAULT bit 0 MOTOR_STALL (wheel not turning)\n",
    "FAULT bit 1 ENCODER (impossible encoder read)\n",
    "FAULT bit 2 IMU (sensor communication or calibration failed)\n",
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

/* CAL_IR substeps after the three sweep legs (0, 1, 2) */
#define MISSION_CAL_SUB_DONE    (3u)  /* sweep over, or stopped on time */
#define MISSION_CAL_SUB_SETTLE  (4u)  /* a turn ended: wait for 0 speed */
#define MISSION_CAL_SUB_CORRECT (5u)  /* heading return turn running */

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
static void     mission_ctx_save(mission_ctx_t const * p_ctx);
static void     mission_ctx_load(mission_ctx_t * p_ctx);
static void     mission_save_words_a_f(mission_ctx_t const * p_ctx,
                                       uint32_t * p_w);
static void     mission_save_words_h_l(mission_ctx_t const * p_ctx,
                                       uint32_t * p_w);
static void     mission_save_words_m_r(mission_ctx_t const * p_ctx,
                                       uint32_t * p_w);
static void     mission_save_timers(mission_ctx_t const * p_ctx);
static void     mission_load_words_a_f(mission_ctx_t * p_ctx);
static void     mission_load_words_h_l(mission_ctx_t * p_ctx);
static void     mission_load_words_m_r(mission_ctx_t * p_ctx);
static void     mission_load_timers(mission_ctx_t * p_ctx);
static uint32_t mission_put_u(uint32_t word, uint32_t pos, uint32_t width,
                              int32_t max, uint32_t value);
static uint32_t mission_put_b(uint32_t word, uint32_t pos, uint32_t width,
                              bool b_val);
static uint32_t mission_put_s(uint32_t word, uint32_t pos, uint32_t width,
                              int32_t min, int32_t max, int32_t offset,
                              int32_t value);
static uint32_t mission_get_u(uint32_t word, uint32_t pos, uint32_t width);
static bool     mission_get_b(uint32_t word, uint32_t pos, uint32_t width);
static int32_t  mission_get_s(uint32_t word, uint32_t pos, uint32_t width,
                              int32_t offset);
static void mission_cal_settle(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap,
                               uint32_t now_us);
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
static void mission_forward(mission_ctx_t const * p_ctx,
                            mission_evt_in_t const * p_evt);
static void mission_snapshot(mission_ctx_t const * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us);
static void mission_publish(mission_ctx_t const * p_ctx,
                            status_board_mission_t const * p_before);
static void mission_part(mission_ctx_t const * p_ctx,
                         status_board_mission_t * p_part);
static void mission_summary(mission_ctx_t const * p_ctx,
                            status_board_t const * p_snap, uint32_t now_us);
static void mission_odo_update(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap);
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
    uint32_t evt_value = value;
    uint32_t word      = 0u;
    uint16_t msg       = 0u;           /* one MSG_EVT word */
    bool     b_is_sent = false;

    if (type < APP_EVT_COUNT)
    {
        /* Every producer sends < 2048 (D.13); a larger value is clamped,
           never wrapped, and flagged (Tribunal 1: every write clamps). */
        if (evt_value > (uint32_t) MSG_EVT_VALUE_MAX)
        {
            evt_value = (uint32_t) MSG_EVT_VALUE_MAX; /* cast: 2047 */
            fault_set(APP_FAULT_RANGE_CLAMP);
        }
        else
        {
            /* fits the 11-bit field */
        }
        word = app_bits_put(0u, MSG_EVT_VALUE_POS, MSG_EVT_VALUE_BITS,
                            evt_value);
        word = app_bits_put(word, MSG_EVT_TYPE_POS, MSG_EVT_TYPE_BITS, type);
        msg  = (uint16_t) word;    /* cast: MSG_EVT_USED_BITS == 16 */
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
 * @brief   Packs a BARCODE / MAP event value with the generated
 *          EVT_BARCODE layout (EVT_MAP has the same fields, checked at
 *          compile time above).
 * @param   cmd        APP_BAR_CMD_* (EVT_BARCODE CMD)
 * @param   char_index Code 39 index 0..42 (EVT_BARCODE CHAR)
 * @return  the 8-bit event value
 * @context any task
 */
uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    uint32_t value = 0u;

    value = app_bits_put(value, EVT_BARCODE_CHAR_POS, EVT_BARCODE_CHAR_BITS,
                         char_index);
    value = app_bits_put(value, EVT_BARCODE_CMD_POS, EVT_BARCODE_CMD_BITS,
                         cmd);

    return value;
}

/**
 * mission_evt_barcode_unpack
 * @brief   Unpacks a BARCODE / MAP event value with the generated
 *          EVT_BARCODE layout.
 * @param   value        packed 8-bit event value
 * @param   p_cmd        out: APP_BAR_CMD_* (may be NULL)
 * @param   p_char_index out: Code 39 index 0..42 (may be NULL)
 * @context any task
 */
void mission_evt_barcode_unpack(uint32_t value, uint32_t * p_cmd,
                                uint32_t * p_char_index)
{
    if (NULL != p_char_index)
    {
        *p_char_index = app_bits_get(value, EVT_BARCODE_CHAR_POS,
                                     EVT_BARCODE_CHAR_BITS);
    }
    if (NULL != p_cmd)
    {
        *p_cmd = app_bits_get(value, EVT_BARCODE_CMD_POS,
                              EVT_BARCODE_CMD_BITS);
    }
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
 * @return  bytes (each queued event costs 8 B, one 16-bit MSG_EVT)
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
    uint16_t         msg      = 0u;    /* last MSG_EVT word received */
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
    {
        mission_ctx_t ctx;

        mission_ctx_reset(&ctx);
        status_board_read(&snap);
        now_us = sys_tick_now_us();
        hmi_us = now_us;
        mission_enter(&ctx, APP_SYS_BOOT, &snap, now_us);
        mission_ctx_save(&ctx);
    }
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
            mission_ctx_t ctx;   /* this pass only; stored packed */

            mission_ctx_load(&ctx);
            size            = tk_rcv_mbf(gh_mission_mbf, &msg, TMO_POL);
            n_pass++;
            evt.b_has_event = (size > 0);
            evt.type        = app_bits_get((uint32_t) msg, /* cast: widen */
                                           MSG_EVT_TYPE_POS,
                                           MSG_EVT_TYPE_BITS);
            evt.value       = app_bits_get((uint32_t) msg, /* cast: widen */
                                           MSG_EVT_VALUE_POS,
                                           MSG_EVT_VALUE_BITS);

            /* 2. buttons every 10 ms */
            now_us          = sys_tick_now_us();
            btn.b_has_event = false;
            if ((now_us - hmi_us) >= MISSION_HMI_PERIOD_US)
            {
                hmi_us = now_us;
                press  = hmi_poll();
                mission_button_event(&ctx, press, &btn);
            }

            status_board_read(&snap);
            mission_pass(&ctx, &evt, &btn, &snap, now_us);

            /* 7. LED, buzzer and watchdog */
            hmi_show(ctx.sys_state, now_us);
            if (!b_is_rgb_ok)
            {
                fault_set(APP_FAULT_PIO_LOAD); /* every pass (D.13) */
            }
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
                if (APP_SYS_RUN == ctx.sys_state)
                {
                    /* Bring-up diagnosis: barcode sensor activity while
                       driving. casts: count modulo 2^31, level 0..1 */
                    app_log("BAR edges %d so far (DO now %d)\n",
                            (int32_t) (barcode_edge_count() & 0x7FFFFFFFu),
                            (int32_t) barcode_do_level(), 0);
                }
            }
#endif
            mission_ctx_save(&ctx);
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
 * mission_put_u / mission_put_b / mission_put_s
 * @brief   Typed field packers for PRV_MIS_* words: unsigned (clamped to
 *          [0, max]), boolean (0/1), and signed (clamped + offset).
 * @param   word   packed word
 * @param   pos    field position (_POS)
 * @param   width  field width (_BITS)
 * @param   max    row maximum (_MAX)
 * @param   value  field value
 * @return  updated word
 * @context mission_task, host test
 */
static uint32_t mission_put_u(uint32_t word, uint32_t pos, uint32_t width,
                              int32_t max, uint32_t value)
{
    /* cast: unsigned field maxima are non-negative */
    uint32_t const limit = (max > 0) ? (uint32_t) max : 0u;

    return app_bits_put(word, pos, width,
                        app_bits_clamp_u(value, limit, NULL));
}

static uint32_t mission_put_b(uint32_t word, uint32_t pos, uint32_t width,
                              bool b_val)
{
    return app_bits_put(word, pos, width, b_val ? 1u : 0u);
}

static uint32_t mission_put_s(uint32_t word, uint32_t pos, uint32_t width,
                              int32_t min, int32_t max, int32_t offset,
                              int32_t value)
{
    return app_bits_put(word, pos, width,
                        app_bits_encode(value, min, max, offset, NULL));
}

/**
 * mission_get_u / mission_get_b / mission_get_s
 * @brief   Typed field unpackers matching mission_put_u/b/s.
 * @param   word   packed word
 * @param   pos    field position (_POS)
 * @param   width  field width (_BITS)
 * @return  unpacked field value
 * @context mission_task, host test
 */
static uint32_t mission_get_u(uint32_t word, uint32_t pos, uint32_t width)
{
    return app_bits_get(word, pos, width);
}

static bool mission_get_b(uint32_t word, uint32_t pos, uint32_t width)
{
    return (0u != app_bits_get(word, pos, width));
}

static int32_t mission_get_s(uint32_t word, uint32_t pos, uint32_t width,
                             int32_t offset)
{
    return app_bits_decode(app_bits_get(word, pos, width), offset);
}

/**
 * mission_ctx_save
 * @brief   Packs the mission state into g_mis_w (PRV_MIS_A..R rows) and
 *          g_mis_us (E5 timers). Every row covers the full range its
 *          field can take (build checks above), so nothing is clamped.
 * @param   p_ctx  state after a pass
 * @context mission_task (end of every pass), host test
 */
static void mission_ctx_save(mission_ctx_t const * p_ctx)
{
    uint32_t w[MISSION_CTX_WORDS] = {0u};
    uint32_t idx = 0u;

    mission_save_words_a_f(p_ctx, w);
    mission_save_words_h_l(p_ctx, w);
    mission_save_words_m_r(p_ctx, w);
    mission_save_timers(p_ctx);

    for (idx = 0u; idx < MISSION_CTX_WORDS; idx++)
    {
        g_mis_w[idx] = w[idx];
    }
}

/**
 * mission_save_words_a_f
 * @brief   Packs PRV_MIS_A through PRV_MIS_F into p_w[MIS_W_A..MIS_W_F].
 * @param   p_ctx  source context
 * @param   p_w    destination word array
 * @context mission_task, host test
 */
static void mission_save_words_a_f(mission_ctx_t const * p_ctx,
                                   uint32_t * p_w)
{
    uint32_t const req_code = (MISSION_REQ_NONE == p_ctx->sys_request)
                              ? MISSION_REQ_CODE_NONE : p_ctx->sys_request;

    /* PRV_MIS_A */
    p_w[MIS_W_A] = mission_put_u(p_w[MIS_W_A], PRV_MIS_A_END_REASON_POS,
                                 PRV_MIS_A_END_REASON_BITS,
                                 PRV_MIS_A_END_REASON_MAX, p_ctx->end_reason);
    p_w[MIS_W_A] = mission_put_s(p_w[MIS_W_A], PRV_MIS_A_DIFF_AT_STOP_POS,
                                 PRV_MIS_A_DIFF_AT_STOP_BITS,
                                 PRV_MIS_A_DIFF_AT_STOP_MIN,
                                 PRV_MIS_A_DIFF_AT_STOP_MAX,
                                 PRV_MIS_A_DIFF_AT_STOP_OFFSET,
                                 p_ctx->diff_at_stop);
    p_w[MIS_W_A] = mission_put_u(p_w[MIS_W_A], PRV_MIS_A_REJOIN_E_POS,
                                 PRV_MIS_A_REJOIN_E_BITS,
                                 PRV_MIS_A_REJOIN_E_MAX, p_ctx->rejoin_e_mm);

    /* PRV_MIS_B */
    p_w[MIS_W_B] = mission_put_u(p_w[MIS_W_B], PRV_MIS_B_SYS_STATE_POS,
                                 PRV_MIS_B_SYS_STATE_BITS,
                                 PRV_MIS_B_SYS_STATE_MAX, p_ctx->sys_state);
    p_w[MIS_W_B] = mission_put_u(p_w[MIS_W_B], PRV_MIS_B_LAST_FAULTS_POS,
                                 PRV_MIS_B_LAST_FAULTS_BITS,
                                 PRV_MIS_B_LAST_FAULTS_MAX,
                                 p_ctx->last_faults);
    p_w[MIS_W_B] = mission_put_s(p_w[MIS_W_B], PRV_MIS_B_CAL_DIFF0_POS,
                                 PRV_MIS_B_CAL_DIFF0_BITS,
                                 PRV_MIS_B_CAL_DIFF0_MIN,
                                 PRV_MIS_B_CAL_DIFF0_MAX,
                                 PRV_MIS_B_CAL_DIFF0_OFFSET,
                                 p_ctx->cal_diff0);

    /* PRV_MIS_C */
    p_w[MIS_W_C] = mission_put_u(p_w[MIS_W_C], PRV_MIS_C_SYS_REQUEST_POS,
                                 PRV_MIS_C_SYS_REQUEST_BITS,
                                 PRV_MIS_C_SYS_REQUEST_MAX, req_code);
    p_w[MIS_W_C] = mission_put_u(p_w[MIS_W_C], PRV_MIS_C_BAR_BACK_LEFT_POS,
                                 PRV_MIS_C_BAR_BACK_LEFT_BITS,
                                 PRV_MIS_C_BAR_BACK_LEFT_MAX,
                                 p_ctx->barcode_back_left);
    p_w[MIS_W_C] = mission_put_s(p_w[MIS_W_C], PRV_MIS_C_RETRY_PARAM_POS,
                                 PRV_MIS_C_RETRY_PARAM_BITS,
                                 PRV_MIS_C_RETRY_PARAM_MIN,
                                 PRV_MIS_C_RETRY_PARAM_MAX,
                                 PRV_MIS_C_RETRY_PARAM_OFFSET,
                                 p_ctx->retry_param);

    /* PRV_MIS_D */
    p_w[MIS_W_D] = mission_put_u(p_w[MIS_W_D], PRV_MIS_D_BAR_MARK_POS,
                                 PRV_MIS_D_BAR_MARK_BITS,
                                 PRV_MIS_D_BAR_MARK_MAX, p_ctx->barcode_mark);
    p_w[MIS_W_D] = mission_put_u(p_w[MIS_W_D], PRV_MIS_D_MOVE_LIMIT_MS_POS,
                                 PRV_MIS_D_MOVE_LIMIT_MS_BITS,
                                 PRV_MIS_D_MOVE_LIMIT_MS_MAX,
                                 p_ctx->move_limit_ms);

    /* PRV_MIS_E */
    p_w[MIS_W_E] = mission_put_u(p_w[MIS_W_E], PRV_MIS_E_RUN_STATE_POS,
                                 PRV_MIS_E_RUN_STATE_BITS,
                                 PRV_MIS_E_RUN_STATE_MAX, p_ctx->run_state);
    p_w[MIS_W_E] = mission_put_u(p_w[MIS_W_E], PRV_MIS_E_ARMED_CMD_POS,
                                 PRV_MIS_E_ARMED_CMD_BITS,
                                 PRV_MIS_E_ARMED_CMD_MAX, p_ctx->armed_cmd);
    p_w[MIS_W_E] = mission_put_u(p_w[MIS_W_E], PRV_MIS_E_BAR_PROBE_MARK_POS,
                                 PRV_MIS_E_BAR_PROBE_MARK_BITS,
                                 PRV_MIS_E_BAR_PROBE_MARK_MAX,
                                 p_ctx->barcode_probe_mark);
    p_w[MIS_W_E] = mission_put_u(p_w[MIS_W_E], PRV_MIS_E_LEG_LAT_POS,
                                 PRV_MIS_E_LEG_LAT_BITS,
                                 PRV_MIS_E_LEG_LAT_MAX, p_ctx->leg_lat_mm);

    /* PRV_MIS_F */
    p_w[MIS_W_F] = mission_put_u(p_w[MIS_W_F], PRV_MIS_F_AVOID_STEP_POS,
                                 PRV_MIS_F_AVOID_STEP_BITS,
                                 PRV_MIS_F_AVOID_STEP_MAX, p_ctx->avoid_step);
    p_w[MIS_W_F] = mission_put_u(p_w[MIS_W_F], PRV_MIS_F_NO_PATH_CNT_POS,
                                 PRV_MIS_F_NO_PATH_CNT_BITS,
                                 PRV_MIS_F_NO_PATH_CNT_MAX,
                                 p_ctx->no_path_cnt);
    p_w[MIS_W_F] = mission_put_u(p_w[MIS_W_F], PRV_MIS_F_BAR_REARM_PATH_POS,
                                 PRV_MIS_F_BAR_REARM_PATH_BITS,
                                 PRV_MIS_F_BAR_REARM_PATH_MAX,
                                 p_ctx->barcode_rearm_path);
    p_w[MIS_W_F] = mission_put_u(p_w[MIS_W_F], PRV_MIS_F_LEG_FWD_POS,
                                 PRV_MIS_F_LEG_FWD_BITS,
                                 PRV_MIS_F_LEG_FWD_MAX, p_ctx->leg_fwd_mm);
}

/**
 * mission_save_words_h_l
 * @brief   Packs PRV_MIS_H, J, K and L into p_w[MIS_W_H..MIS_W_L].
 * @param   p_ctx  source context
 * @param   p_w    destination word array
 * @context mission_task, host test
 */
static void mission_save_words_h_l(mission_ctx_t const * p_ctx,
                                   uint32_t * p_w)
{
    uint32_t w_h = 0u;
    uint32_t w_j = 0u;
    uint32_t w_k = 0u;
    uint32_t w_l = 0u;

    /* PRV_MIS_H */
    w_h = mission_put_u(w_h, PRV_MIS_H_SUBSTEP_POS, PRV_MIS_H_SUBSTEP_BITS,
                        PRV_MIS_H_SUBSTEP_MAX, p_ctx->substep);
    w_h = mission_put_u(w_h, PRV_MIS_H_ARMED_CHAR_POS,
                        PRV_MIS_H_ARMED_CHAR_BITS, PRV_MIS_H_ARMED_CHAR_MAX,
                        p_ctx->armed_char);
    w_h = mission_put_u(w_h, PRV_MIS_H_BAR_PHASE_POS,
                        PRV_MIS_H_BAR_PHASE_BITS, PRV_MIS_H_BAR_PHASE_MAX,
                        p_ctx->barcode_phase);
    w_h = mission_put_u(w_h, PRV_MIS_H_BAR_FOLLOW_POS,
                        PRV_MIS_H_BAR_FOLLOW_BITS, PRV_MIS_H_BAR_FOLLOW_MAX,
                        p_ctx->barcode_follow_path);
    w_h = mission_put_u(w_h, PRV_MIS_H_RETRY_KIND_POS,
                        PRV_MIS_H_RETRY_KIND_BITS, PRV_MIS_H_RETRY_KIND_MAX,
                        p_ctx->retry_kind);

    /* PRV_MIS_J */
    w_j = mission_put_b(w_j, PRV_MIS_J_ARMED_POS, PRV_MIS_J_ARMED_BITS,
                        p_ctx->b_is_armed);
    w_j = mission_put_b(w_j, PRV_MIS_J_PROBE_PENDING_POS,
                        PRV_MIS_J_PROBE_PENDING_BITS, p_ctx->b_probe_pending);
    w_j = mission_put_b(w_j, PRV_MIS_J_REARM_WAIT_POS,
                        PRV_MIS_J_REARM_WAIT_BITS, p_ctx->b_rearm_wait);
    w_j = mission_put_u(w_j, PRV_MIS_J_BAR_BACK_PATH_POS,
                        PRV_MIS_J_BAR_BACK_PATH_BITS,
                        PRV_MIS_J_BAR_BACK_PATH_MAX,
                        p_ctx->barcode_back_path);
    w_j = mission_put_u(w_j, PRV_MIS_J_BAR_TRIES_POS,
                        PRV_MIS_J_BAR_TRIES_BITS, PRV_MIS_J_BAR_TRIES_MAX,
                        p_ctx->barcode_tries);
    w_j = mission_put_u(w_j, PRV_MIS_J_SIDE_POS, PRV_MIS_J_SIDE_BITS,
                        PRV_MIS_J_SIDE_MAX, p_ctx->side);
    w_j = mission_put_u(w_j, PRV_MIS_J_SEARCH_FIRST_POS,
                        PRV_MIS_J_SEARCH_FIRST_BITS,
                        PRV_MIS_J_SEARCH_FIRST_MAX, p_ctx->search_first);
    w_j = mission_put_u(w_j, PRV_MIS_J_CAL_CORR_CNT_POS,
                        PRV_MIS_J_CAL_CORR_CNT_BITS,
                        PRV_MIS_J_CAL_CORR_CNT_MAX, p_ctx->cal_corr_cnt);
    w_j = mission_put_u(w_j, PRV_MIS_J_WAIT_TAG_POS, PRV_MIS_J_WAIT_TAG_BITS,
                        PRV_MIS_J_WAIT_TAG_MAX, p_ctx->wait_tag);
    w_j = mission_put_b(w_j, PRV_MIS_J_WARNING_POS, PRV_MIS_J_WARNING_BITS,
                        p_ctx->b_is_warning);
    w_j = mission_put_b(w_j, PRV_MIS_J_RESUME_PENDING_POS,
                        PRV_MIS_J_RESUME_PENDING_BITS,
                        p_ctx->b_resume_pending);
    w_j = mission_put_b(w_j, PRV_MIS_J_INHIBIT_PEND_POS,
                        PRV_MIS_J_INHIBIT_PEND_BITS,
                        p_ctx->b_inhibit_pending);

    /* PRV_MIS_K */
    w_k = mission_put_u(w_k, PRV_MIS_K_BAR_EDGE_PATH_POS,
                        PRV_MIS_K_BAR_EDGE_PATH_BITS,
                        PRV_MIS_K_BAR_EDGE_PATH_MAX,
                        p_ctx->barcode_edge_path);
    w_k = mission_put_b(w_k, PRV_MIS_K_FR9_RUNNING_POS,
                        PRV_MIS_K_FR9_RUNNING_BITS, p_ctx->b_fr9_running);
    w_k = mission_put_b(w_k, PRV_MIS_K_WAIT_STILL_POS,
                        PRV_MIS_K_WAIT_STILL_BITS, p_ctx->b_wait_standstill);
    w_k = mission_put_b(w_k, PRV_MIS_K_CREEPING_POS, PRV_MIS_K_CREEPING_BITS,
                        p_ctx->b_creeping);
    w_k = mission_put_b(w_k, PRV_MIS_K_CREPT_POS, PRV_MIS_K_CREPT_BITS,
                        p_ctx->b_crept);
    w_k = mission_put_b(w_k, PRV_MIS_K_HAS_RUN_POS, PRV_MIS_K_HAS_RUN_BITS,
                        p_ctx->b_has_run);
    w_k = mission_put_b(w_k, PRV_MIS_K_POST_AVOID_POS,
                        PRV_MIS_K_POST_AVOID_BITS, p_ctx->b_post_avoid_slow);
    w_k = mission_put_b(w_k, PRV_MIS_K_QUIET_POS, PRV_MIS_K_QUIET_BITS,
                        p_ctx->b_is_quiet);
    w_k = mission_put_b(w_k, PRV_MIS_K_MOVE_PENDING_POS,
                        PRV_MIS_K_MOVE_PENDING_BITS, p_ctx->b_move_pending);
    w_k = mission_put_b(w_k, PRV_MIS_K_RETRY_PENDING_POS,
                        PRV_MIS_K_RETRY_PENDING_BITS, p_ctx->b_retry_pending);
    w_k = mission_put_b(w_k, PRV_MIS_K_MOTION_FAILED_POS,
                        PRV_MIS_K_MOTION_FAILED_BITS, p_ctx->b_motion_failed);
    w_k = mission_put_b(w_k, PRV_MIS_K_IMU_VALID_POS,
                        PRV_MIS_K_IMU_VALID_BITS, p_ctx->b_imu_valid);
    w_k = mission_put_b(w_k, PRV_MIS_K_IR_VALID_POS, PRV_MIS_K_IR_VALID_BITS,
                        p_ctx->b_ir_valid);
    w_k = mission_put_b(w_k, PRV_MIS_K_WDT_ENABLED_POS,
                        PRV_MIS_K_WDT_ENABLED_BITS, p_ctx->b_wdt_enabled);
    w_k = mission_put_b(w_k, PRV_MIS_K_RUN_STARTING_POS,
                        PRV_MIS_K_RUN_STARTING_BITS, p_ctx->b_run_starting);
    w_k = mission_put_b(w_k, PRV_MIS_K_ZERO_DONE_POS,
                        PRV_MIS_K_ZERO_DONE_BITS, p_ctx->b_zero_done);

    /* PRV_MIS_L */
    w_l = mission_put_u(w_l, PRV_MIS_L_PATH_AT_STOP_POS,
                        PRV_MIS_L_PATH_AT_STOP_BITS,
                        PRV_MIS_L_PATH_AT_STOP_MAX, p_ctx->path_at_stop);
    w_l = mission_put_b(w_l, PRV_MIS_L_RECOVERING_POS,
                        PRV_MIS_L_RECOVERING_BITS, p_ctx->b_recovering);

    p_w[MIS_W_H] = w_h;
    p_w[MIS_W_J] = w_j;
    p_w[MIS_W_K] = w_k;
    p_w[MIS_W_L] = w_l;
}

/**
 * mission_save_words_m_r
 * @brief   Packs PRV_MIS_M, N, P, Q and R into p_w[MIS_W_M..MIS_W_R].
 * @param   p_ctx  source context
 * @param   p_w    destination word array
 * @context mission_task, host test
 */
static void mission_save_words_m_r(mission_ctx_t const * p_ctx,
                                   uint32_t * p_w)
{
    p_w[MIS_W_M] = mission_put_u(0u, PRV_MIS_M_RUN_ODO_MARK_POS,
                                 PRV_MIS_M_RUN_ODO_MARK_BITS,
                                 PRV_MIS_M_RUN_ODO_MARK_MAX,
                                 p_ctx->run_odo_mark);
    p_w[MIS_W_N] = mission_put_u(0u, PRV_MIS_N_RUN_ODO_POS,
                                 PRV_MIS_N_RUN_ODO_BITS,
                                 PRV_MIS_N_RUN_ODO_MAX,
                                 p_ctx->run_odo_counts);
    p_w[MIS_W_P] = mission_put_u(0u, PRV_MIS_P_REJOIN_START_POS,
                                 PRV_MIS_P_REJOIN_START_BITS,
                                 PRV_MIS_P_REJOIN_START_MAX,
                                 p_ctx->rejoin_start);
    p_w[MIS_W_Q] = mission_put_u(0u, PRV_MIS_Q_INHIBIT_MARK_POS,
                                 PRV_MIS_Q_INHIBIT_MARK_BITS,
                                 PRV_MIS_Q_INHIBIT_MARK_MAX,
                                 p_ctx->inhibit_mark);
    p_w[MIS_W_R] = mission_put_u(0u, PRV_MIS_R_BAR_EDGES_POS,
                                 PRV_MIS_R_BAR_EDGES_BITS,
                                 PRV_MIS_R_BAR_EDGES_MAX,
                                 p_ctx->barcode_edges);
}

/**
 * mission_save_timers
 * @brief   Stores the 12 E5 timer images into g_mis_us[MIS_US_*].
 * @param   p_ctx  source context
 * @context mission_task, host test
 */
static void mission_save_timers(mission_ctx_t const * p_ctx)
{
    g_mis_us[MIS_US_BARCODE]   = p_ctx->barcode_since_us;
    g_mis_us[MIS_US_CAL]       = p_ctx->cal_start_us;
    g_mis_us[MIS_US_SWEEP]     = p_ctx->sweep_start_us;
    g_mis_us[MIS_US_WARN]      = p_ctx->warn_start_us;
    g_mis_us[MIS_US_SEARCH]    = p_ctx->search_start_us;
    g_mis_us[MIS_US_FR9]       = p_ctx->fr9_start_us;
    g_mis_us[MIS_US_RESUME]    = p_ctx->resume_start_us;
    g_mis_us[MIS_US_RUN_START] = p_ctx->run_start_us;
    g_mis_us[MIS_US_RUN_END]   = p_ctx->run_end_us;
    g_mis_us[MIS_US_MOVE]      = p_ctx->move_start_us;
    g_mis_us[MIS_US_RUN_SET]   = p_ctx->run_start_set_us;
    g_mis_us[MIS_US_RECOVER]   = p_ctx->recover_start_us;
}

/**
 * mission_ctx_load
 * @brief   Unpacks g_mis_w / g_mis_us into a working copy for one pass.
 * @param   p_ctx  working copy out
 * @context mission_task (start of every pass), host test
 */
static void mission_ctx_load(mission_ctx_t * p_ctx)
{
    mission_load_words_a_f(p_ctx);
    mission_load_words_h_l(p_ctx);
    mission_load_words_m_r(p_ctx);
    mission_load_timers(p_ctx);
}

/**
 * mission_load_words_a_f
 * @brief   Unpacks PRV_MIS_A through PRV_MIS_F from g_mis_w[MIS_W_A..F].
 * @param   p_ctx  destination context
 * @context mission_task, host test
 */
static void mission_load_words_a_f(mission_ctx_t * p_ctx)
{
    uint32_t const w_a = g_mis_w[MIS_W_A];
    uint32_t const w_b = g_mis_w[MIS_W_B];
    uint32_t const w_c = g_mis_w[MIS_W_C];
    uint32_t const w_d = g_mis_w[MIS_W_D];
    uint32_t const w_e = g_mis_w[MIS_W_E];
    uint32_t const w_f = g_mis_w[MIS_W_F];
    uint32_t const req = mission_get_u(w_c, PRV_MIS_C_SYS_REQUEST_POS,
                                       PRV_MIS_C_SYS_REQUEST_BITS);

    /* PRV_MIS_A */
    p_ctx->end_reason   = mission_get_u(w_a, PRV_MIS_A_END_REASON_POS,
                                        PRV_MIS_A_END_REASON_BITS);
    p_ctx->diff_at_stop = mission_get_s(w_a, PRV_MIS_A_DIFF_AT_STOP_POS,
                                        PRV_MIS_A_DIFF_AT_STOP_BITS,
                                        PRV_MIS_A_DIFF_AT_STOP_OFFSET);
    p_ctx->rejoin_e_mm  = mission_get_u(w_a, PRV_MIS_A_REJOIN_E_POS,
                                        PRV_MIS_A_REJOIN_E_BITS);

    /* PRV_MIS_B */
    p_ctx->sys_state   = mission_get_u(w_b, PRV_MIS_B_SYS_STATE_POS,
                                       PRV_MIS_B_SYS_STATE_BITS);
    p_ctx->last_faults = mission_get_u(w_b, PRV_MIS_B_LAST_FAULTS_POS,
                                       PRV_MIS_B_LAST_FAULTS_BITS);
    p_ctx->cal_diff0   = mission_get_s(w_b, PRV_MIS_B_CAL_DIFF0_POS,
                                       PRV_MIS_B_CAL_DIFF0_BITS,
                                       PRV_MIS_B_CAL_DIFF0_OFFSET);

    /* PRV_MIS_C */
    p_ctx->sys_request       = (MISSION_REQ_CODE_NONE == req)
                               ? MISSION_REQ_NONE : req;
    p_ctx->barcode_back_left = mission_get_u(w_c, PRV_MIS_C_BAR_BACK_LEFT_POS,
                                             PRV_MIS_C_BAR_BACK_LEFT_BITS);
    p_ctx->retry_param       = mission_get_s(w_c, PRV_MIS_C_RETRY_PARAM_POS,
                                             PRV_MIS_C_RETRY_PARAM_BITS,
                                             PRV_MIS_C_RETRY_PARAM_OFFSET);

    /* PRV_MIS_D */
    p_ctx->barcode_mark  = mission_get_u(w_d, PRV_MIS_D_BAR_MARK_POS,
                                         PRV_MIS_D_BAR_MARK_BITS);
    p_ctx->move_limit_ms = mission_get_u(w_d, PRV_MIS_D_MOVE_LIMIT_MS_POS,
                                         PRV_MIS_D_MOVE_LIMIT_MS_BITS);

    /* PRV_MIS_E */
    p_ctx->run_state          = mission_get_u(w_e, PRV_MIS_E_RUN_STATE_POS,
                                              PRV_MIS_E_RUN_STATE_BITS);
    p_ctx->armed_cmd          = mission_get_u(w_e, PRV_MIS_E_ARMED_CMD_POS,
                                              PRV_MIS_E_ARMED_CMD_BITS);
    p_ctx->barcode_probe_mark = mission_get_u(w_e,
                                              PRV_MIS_E_BAR_PROBE_MARK_POS,
                                              PRV_MIS_E_BAR_PROBE_MARK_BITS);
    p_ctx->leg_lat_mm         = mission_get_u(w_e, PRV_MIS_E_LEG_LAT_POS,
                                              PRV_MIS_E_LEG_LAT_BITS);

    /* PRV_MIS_F */
    p_ctx->avoid_step         = mission_get_u(w_f, PRV_MIS_F_AVOID_STEP_POS,
                                              PRV_MIS_F_AVOID_STEP_BITS);
    p_ctx->no_path_cnt        = mission_get_u(w_f, PRV_MIS_F_NO_PATH_CNT_POS,
                                              PRV_MIS_F_NO_PATH_CNT_BITS);
    p_ctx->barcode_rearm_path = mission_get_u(w_f,
                                              PRV_MIS_F_BAR_REARM_PATH_POS,
                                              PRV_MIS_F_BAR_REARM_PATH_BITS);
    p_ctx->leg_fwd_mm         = mission_get_u(w_f, PRV_MIS_F_LEG_FWD_POS,
                                              PRV_MIS_F_LEG_FWD_BITS);
}

/**
 * mission_load_words_h_l
 * @brief   Unpacks PRV_MIS_H, J, K and L from g_mis_w[MIS_W_H..MIS_W_L].
 * @param   p_ctx  destination context
 * @context mission_task, host test
 */
static void mission_load_words_h_l(mission_ctx_t * p_ctx)
{
    uint32_t const w_h = g_mis_w[MIS_W_H];
    uint32_t const w_j = g_mis_w[MIS_W_J];
    uint32_t const w_k = g_mis_w[MIS_W_K];
    uint32_t const w_l = g_mis_w[MIS_W_L];

    /* PRV_MIS_H */
    p_ctx->substep             = mission_get_u(w_h, PRV_MIS_H_SUBSTEP_POS,
                                               PRV_MIS_H_SUBSTEP_BITS);
    p_ctx->armed_char          = mission_get_u(w_h, PRV_MIS_H_ARMED_CHAR_POS,
                                               PRV_MIS_H_ARMED_CHAR_BITS);
    p_ctx->barcode_phase       = mission_get_u(w_h, PRV_MIS_H_BAR_PHASE_POS,
                                               PRV_MIS_H_BAR_PHASE_BITS);
    p_ctx->barcode_follow_path = mission_get_u(w_h, PRV_MIS_H_BAR_FOLLOW_POS,
                                               PRV_MIS_H_BAR_FOLLOW_BITS);
    p_ctx->retry_kind          = mission_get_u(w_h, PRV_MIS_H_RETRY_KIND_POS,
                                               PRV_MIS_H_RETRY_KIND_BITS);

    /* PRV_MIS_J */
    p_ctx->b_is_armed        = mission_get_b(w_j, PRV_MIS_J_ARMED_POS,
                                             PRV_MIS_J_ARMED_BITS);
    p_ctx->b_probe_pending   = mission_get_b(w_j, PRV_MIS_J_PROBE_PENDING_POS,
                                             PRV_MIS_J_PROBE_PENDING_BITS);
    p_ctx->b_rearm_wait      = mission_get_b(w_j, PRV_MIS_J_REARM_WAIT_POS,
                                             PRV_MIS_J_REARM_WAIT_BITS);
    p_ctx->barcode_back_path = mission_get_u(w_j, PRV_MIS_J_BAR_BACK_PATH_POS,
                                             PRV_MIS_J_BAR_BACK_PATH_BITS);
    p_ctx->barcode_tries     = mission_get_u(w_j, PRV_MIS_J_BAR_TRIES_POS,
                                             PRV_MIS_J_BAR_TRIES_BITS);
    p_ctx->side              = mission_get_u(w_j, PRV_MIS_J_SIDE_POS,
                                             PRV_MIS_J_SIDE_BITS);
    p_ctx->search_first      = mission_get_u(w_j, PRV_MIS_J_SEARCH_FIRST_POS,
                                             PRV_MIS_J_SEARCH_FIRST_BITS);
    p_ctx->cal_corr_cnt      = mission_get_u(w_j, PRV_MIS_J_CAL_CORR_CNT_POS,
                                             PRV_MIS_J_CAL_CORR_CNT_BITS);
    p_ctx->wait_tag          = mission_get_u(w_j, PRV_MIS_J_WAIT_TAG_POS,
                                             PRV_MIS_J_WAIT_TAG_BITS);
    p_ctx->b_is_warning      = mission_get_b(w_j, PRV_MIS_J_WARNING_POS,
                                             PRV_MIS_J_WARNING_BITS);
    p_ctx->b_resume_pending  = mission_get_b(w_j,
                                             PRV_MIS_J_RESUME_PENDING_POS,
                                             PRV_MIS_J_RESUME_PENDING_BITS);
    p_ctx->b_inhibit_pending = mission_get_b(w_j, PRV_MIS_J_INHIBIT_PEND_POS,
                                             PRV_MIS_J_INHIBIT_PEND_BITS);

    /* PRV_MIS_K */
    p_ctx->barcode_edge_path = mission_get_u(w_k, PRV_MIS_K_BAR_EDGE_PATH_POS,
                                             PRV_MIS_K_BAR_EDGE_PATH_BITS);
    p_ctx->b_fr9_running     = mission_get_b(w_k, PRV_MIS_K_FR9_RUNNING_POS,
                                             PRV_MIS_K_FR9_RUNNING_BITS);
    p_ctx->b_wait_standstill = mission_get_b(w_k, PRV_MIS_K_WAIT_STILL_POS,
                                             PRV_MIS_K_WAIT_STILL_BITS);
    p_ctx->b_creeping        = mission_get_b(w_k, PRV_MIS_K_CREEPING_POS,
                                             PRV_MIS_K_CREEPING_BITS);
    p_ctx->b_crept           = mission_get_b(w_k, PRV_MIS_K_CREPT_POS,
                                             PRV_MIS_K_CREPT_BITS);
    p_ctx->b_has_run         = mission_get_b(w_k, PRV_MIS_K_HAS_RUN_POS,
                                             PRV_MIS_K_HAS_RUN_BITS);
    p_ctx->b_post_avoid_slow = mission_get_b(w_k, PRV_MIS_K_POST_AVOID_POS,
                                             PRV_MIS_K_POST_AVOID_BITS);
    p_ctx->b_is_quiet        = mission_get_b(w_k, PRV_MIS_K_QUIET_POS,
                                             PRV_MIS_K_QUIET_BITS);
    p_ctx->b_move_pending    = mission_get_b(w_k, PRV_MIS_K_MOVE_PENDING_POS,
                                             PRV_MIS_K_MOVE_PENDING_BITS);
    p_ctx->b_retry_pending   = mission_get_b(w_k, PRV_MIS_K_RETRY_PENDING_POS,
                                             PRV_MIS_K_RETRY_PENDING_BITS);
    p_ctx->b_motion_failed   = mission_get_b(w_k, PRV_MIS_K_MOTION_FAILED_POS,
                                             PRV_MIS_K_MOTION_FAILED_BITS);
    p_ctx->b_imu_valid       = mission_get_b(w_k, PRV_MIS_K_IMU_VALID_POS,
                                             PRV_MIS_K_IMU_VALID_BITS);
    p_ctx->b_ir_valid        = mission_get_b(w_k, PRV_MIS_K_IR_VALID_POS,
                                             PRV_MIS_K_IR_VALID_BITS);
    p_ctx->b_wdt_enabled     = mission_get_b(w_k, PRV_MIS_K_WDT_ENABLED_POS,
                                             PRV_MIS_K_WDT_ENABLED_BITS);
    p_ctx->b_run_starting    = mission_get_b(w_k, PRV_MIS_K_RUN_STARTING_POS,
                                             PRV_MIS_K_RUN_STARTING_BITS);
    p_ctx->b_zero_done       = mission_get_b(w_k, PRV_MIS_K_ZERO_DONE_POS,
                                             PRV_MIS_K_ZERO_DONE_BITS);

    /* PRV_MIS_L */
    p_ctx->path_at_stop = mission_get_u(w_l, PRV_MIS_L_PATH_AT_STOP_POS,
                                        PRV_MIS_L_PATH_AT_STOP_BITS);
    p_ctx->b_recovering = mission_get_b(w_l, PRV_MIS_L_RECOVERING_POS,
                                        PRV_MIS_L_RECOVERING_BITS);
}

/**
 * mission_load_words_m_r
 * @brief   Unpacks PRV_MIS_M, N, P, Q and R from g_mis_w[MIS_W_M..MIS_W_R].
 * @param   p_ctx  destination context
 * @context mission_task, host test
 */
static void mission_load_words_m_r(mission_ctx_t * p_ctx)
{
    p_ctx->run_odo_mark   = mission_get_u(g_mis_w[MIS_W_M],
                                          PRV_MIS_M_RUN_ODO_MARK_POS,
                                          PRV_MIS_M_RUN_ODO_MARK_BITS);
    p_ctx->run_odo_counts = mission_get_u(g_mis_w[MIS_W_N],
                                          PRV_MIS_N_RUN_ODO_POS,
                                          PRV_MIS_N_RUN_ODO_BITS);
    p_ctx->rejoin_start   = mission_get_u(g_mis_w[MIS_W_P],
                                          PRV_MIS_P_REJOIN_START_POS,
                                          PRV_MIS_P_REJOIN_START_BITS);
    p_ctx->inhibit_mark   = mission_get_u(g_mis_w[MIS_W_Q],
                                          PRV_MIS_Q_INHIBIT_MARK_POS,
                                          PRV_MIS_Q_INHIBIT_MARK_BITS);
    p_ctx->barcode_edges  = mission_get_u(g_mis_w[MIS_W_R],
                                          PRV_MIS_R_BAR_EDGES_POS,
                                          PRV_MIS_R_BAR_EDGES_BITS);
}

/**
 * mission_load_timers
 * @brief   Restores the 12 E5 timer images from g_mis_us[MIS_US_*].
 * @param   p_ctx  destination context
 * @context mission_task, host test
 */
static void mission_load_timers(mission_ctx_t * p_ctx)
{
    p_ctx->barcode_since_us = g_mis_us[MIS_US_BARCODE];
    p_ctx->cal_start_us     = g_mis_us[MIS_US_CAL];
    p_ctx->sweep_start_us   = g_mis_us[MIS_US_SWEEP];
    p_ctx->warn_start_us    = g_mis_us[MIS_US_WARN];
    p_ctx->search_start_us  = g_mis_us[MIS_US_SEARCH];
    p_ctx->fr9_start_us     = g_mis_us[MIS_US_FR9];
    p_ctx->resume_start_us  = g_mis_us[MIS_US_RESUME];
    p_ctx->run_start_us     = g_mis_us[MIS_US_RUN_START];
    p_ctx->run_end_us       = g_mis_us[MIS_US_RUN_END];
    p_ctx->move_start_us    = g_mis_us[MIS_US_MOVE];
    p_ctx->run_start_set_us = g_mis_us[MIS_US_RUN_SET];
    p_ctx->recover_start_us = g_mis_us[MIS_US_RECOVER];
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

    if (APP_SYS_RUN == p_ctx->sys_state)
    {
        mission_odo_update(p_ctx, p_snap);
    }
    else
    {
        /* Odometer frozen outside RUN. */
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
                " (1 = ESTOP, 2 = LIFT)\n",
                (int32_t) status_board_motion_state(p_snap),
                (int32_t) safety_get(), 0);
        mission_enter(p_ctx, APP_SYS_FAULT, p_snap, now_us);
    }
    p_ctx->b_motion_failed = false;
    mission_publish(p_ctx, &before);                    /* 5 */
    mission_forward(p_ctx, p_evt);                      /* 6 */
    mission_snapshot(p_ctx, p_evt, p_snap, now_us);
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
    if ((APP_SYS_RUN == p_ctx->sys_state) && (APP_SYS_RUN != sys_state))
    {
        p_ctx->run_end_us   = now_us;          /* frozen for snapshots */
        mission_odo_update(p_ctx, p_snap);     /* odometer stops here  */
    }
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
            p_ctx->cal_diff0      = status_board_pos_l(p_snap)
                                    - status_board_pos_r(p_snap);
            p_ctx->cal_corr_cnt   = 0u;
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
            mission_mode_clear(APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD);
            mission_summary(p_ctx, p_snap, now_us);
            break;
        case APP_SYS_FAULT:
            p_ctx->end_reason = APP_END_FAULT;
            mission_stop_motion(p_ctx);
            mission_mode_clear(APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD);
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
 *          leg starts on the tagged MOVE_DONE), turn back to the start
 *          heading (mission_cal_settle), then clear CAL_IR and
 *          MOTOR_EN; CAL_DONE(IR) -> READY. Sweep over 4 s -> stop;
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
            /* last leg or a return turn done (or a stale one after) */
        }
        if (p_ctx->substep < 2u)
        {
            p_ctx->substep++;
        }
        else if ((2u == p_ctx->substep)
                 || (MISSION_CAL_SUB_CORRECT == p_ctx->substep))
        {
            p_ctx->substep = MISSION_CAL_SUB_SETTLE;
        }
        else
        {
            /* DONE or SETTLE: nothing to start */
        }
    }
    else
    {
        /* waiting for the sweep or the result */
    }

    if ((APP_SYS_CAL_IR == p_ctx->sys_state)
        && (MISSION_CAL_SUB_SETTLE == p_ctx->substep))
    {
        mission_cal_settle(p_ctx, p_snap, now_us);
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
    else if ((MISSION_CAL_SUB_DONE != p_ctx->substep)
             && ((now_us - p_ctx->sweep_start_us) >= sweep_us))
    {
        mission_stop_motion(p_ctx);               /* sweep took > 4 s */
        mission_mode_clear(cal_ir);
        p_ctx->substep = MISSION_CAL_SUB_DONE;
    }
    else
    {
        /* in time */
    }
}

/**
 * mission_cal_settle
 * @brief   CAL_IR after a turn: once both wheels stand still, compare the
 *          encoder heading with the one at CAL_IR entry. Within
 *          MISSION_CAL_HEADING_TOL_CDEG (or after MISSION_CAL_CORR_MAX
 *          return turns): sweep done, clear CAL_IR and MOTOR_EN. Else
 *          turn back by the error (+ = turned right, so turn left).
 * @context mission_task
 */
static void mission_cal_settle(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap,
                               uint32_t now_us)
{
    int32_t const diff_now = status_board_pos_l(p_snap)
                             - status_board_pos_r(p_snap);
    int32_t const err_cdeg = odometry_heading_cdeg(diff_now, 0)
                             - odometry_heading_cdeg(p_ctx->cal_diff0, 0);
    int32_t const mag_cdeg = (err_cdeg < 0) ? -err_cdeg : err_cdeg;
    int32_t       turn_deg = 0;

    if ((0 != status_board_speed_l(p_snap))
        || (0 != status_board_speed_r(p_snap)))
    {
        /* still coasting from the last turn */
    }
    else if ((mag_cdeg <= MISSION_CAL_HEADING_TOL_CDEG)
             || (p_ctx->cal_corr_cnt >= MISSION_CAL_CORR_MAX))
    {
        if (!p_ctx->b_is_quiet)
        {
            /* cast: return turn count 0..2 */
            app_log("CAL_IR sweep done: heading %d cdeg from the start"
                    " (+ = right) after %d return turns\n",
                    err_cdeg, (int32_t) p_ctx->cal_corr_cnt, 0);
        }
        mission_mode_clear(APP_MODE_CAL_IR | APP_MODE_MOTOR_EN);
        p_ctx->substep = MISSION_CAL_SUB_DONE;
    }
    else
    {
        /* event time: division allowed */
        turn_deg = (mag_cdeg + (MISSION_CDEG_PER_DEG / 2))
                   / MISSION_CDEG_PER_DEG;
        if (!p_ctx->b_is_quiet)
        {
            app_log("CAL_IR heading %d cdeg from the start (+ = right):"
                    " turning back %d deg\n", err_cdeg, turn_deg, 0);
        }
        (void) mission_send_move(p_ctx,
                                 (err_cdeg > 0) ? MISSION_MV_LEFT
                                                : MISSION_MV_RIGHT,
                                 turn_deg, now_us);
        p_ctx->cal_corr_cnt++;
        p_ctx->substep = MISSION_CAL_SUB_CORRECT;
    }
}

/**
 * mission_st_ready
 * @brief   READY: START -> 1 s warning (buzzer + LED) -> run start: set
 *          clear a stale LIFT, set RUN_START, ZERO the odometry and wait
 *          for its tagged MOVE_DONE (deadline 200 ms -> FAULT; a LIFT
 *          from here on -> FAULT LIFT), hold RUN_START >= 30 ms so line,
 *          barcode and IMU reset their per-run state, then clear it, set
 *          MOTOR_EN and enter RUN with the run odometer at 0.
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
        /* Before the ZERO: with LIFT set, motion_task sits in ESTOP and
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
            p_ctx->run_odo_mark   = 0u;           /* odometry rebased */
            p_ctx->run_odo_counts = 0u;
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
 *          stop fault. Lift-stop (Change Requirement 2): SAFETY_LIFT has
 *          already braked the motors; in RUN it ends the run normally
 *          (FINISHED, end reason STOP: lifting the car is the end-of-run
 *          signal in the demonstration), and in the CAL_IR sweep or the
 *          run start it is a FAULT, as the calibration or the start did
 *          not complete. In the run start the LIFT was already cleared
 *          once (mission_clear_stale_safety), so a new one is real.
 * @context mission_task
 */
static void mission_faults(mission_ctx_t * p_ctx,
                           status_board_t const * p_snap, uint32_t now_us)
{
    uint32_t const faults  = fault_get();
    uint32_t const fresh   = faults & ~p_ctx->last_faults;
    bool const     b_lift  = (0u != (safety_get() & SAFETY_LIFT));
    bool const     b_start = (APP_SYS_CAL_IR == p_ctx->sys_state)
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
    else if (b_lift && (APP_SYS_RUN == p_ctx->sys_state))
    {
        app_log("RUN END: robot lifted (lift-stop)\n", 0, 0, 0);
        p_ctx->end_reason      = APP_END_STOP;
        p_ctx->b_motion_failed = false; /* a move cut short by the brake */
        mission_enter(p_ctx, APP_SYS_FINISHED, p_snap, now_us);
    }
    else if (b_lift && b_start)
    {
        app_log("FAULT LIFT (lifted during the sweep or the run start)\n",
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
 * @brief   Clears a LIFT left set while the motors were not enabled,
 *          before the motors are used: on CAL_IR entry (before MOTOR_EN)
 *          and at the end of the READY warning (before the run-start ZERO,
 *          which motion_task only latches while it is in ESTOP). imu_task
 *          only arms the lift-stop while MOTOR_EN is set, so this is a
 *          guard, not a normal path (tests S5-U32 and S5-U33). A LIFT
 *          after this point, in CAL_IR, the run start or RUN, is acted on
 *          by mission_faults.
 *          Only the LIFT bit is cleared. ESTOP is NOT cleared:
 *          obstacle_task does not set it while MOTOR_EN is clear, and an
 *          ESTOP that is set means a hazard that only obstacle_task or a
 *          RESET may release. mission_task stays the only task that
 *          clears safety bits.
 * @context mission_task
 */
static void mission_clear_stale_safety(void)
{
    if (0u != (safety_get() & SAFETY_LIFT))
    {
        /* Bring-up diagnosis */
        app_log("SAFETY LIFT latched before the motors were enabled:"
                " cleared\n", 0, 0, 0);
        safety_clear(SAFETY_LIFT);
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
 * @brief   Step 6: copies selected events to telemetry and confirms MAP
 *          updates after they are stored. A MAP is stored only outside
 *          RUN (Tribunal 1, Version C gap 7); in RUN the table is left
 *          unchanged and no confirmation is sent.
 * @param   p_ctx mission context (system state after this pass)
 * @param   p_evt event taken this pass
 * @context mission_task
 */
static void mission_forward(mission_ctx_t const * p_ctx,
                            mission_evt_in_t const * p_evt)
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
            case APP_EVT_JUNCTION:
                (void) telem_post_event(APP_EVT_JUNCTION, p_evt->value);
                if (APP_JCT_CROSS == p_evt->value)
                {
                    app_log("CROSS-JUNCTION detected\n", 0, 0, 0);
                }
                else
                {
                    /* cast: APP_JCT_SIDE_L / APP_JCT_SIDE_R are 2..3 */
                    app_log("SIDE MARK detected (%d: 2 = left, 3 = right)\n",
                            (int32_t) p_evt->value, 0, 0);
                }
                break;
            case APP_EVT_MAP:
            {
                uint32_t ch  = 0u;
                uint32_t cmd = 0u;

                mission_evt_barcode_unpack(p_evt->value, &cmd, &ch);
                if (APP_SYS_RUN == p_ctx->sys_state)
                {
                    /* casts: Code 39 index and command fit int32_t */
                    app_log("MAP refused in RUN: char index %d, command %d"
                            " (send it before START)\n",
                            (int32_t) ch, (int32_t) cmd, 0);
                }
                else if (barcode_map_set(ch, cmd))
                {
                    (void) telem_post_event(APP_EVT_MAP, p_evt->value);
                    /* casts: Code 39 index and command fit int32_t */
                    app_log("MAP stored char index %d, command %d\n",
                            (int32_t) ch, (int32_t) cmd, 0);
                }
                break;
            }
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
 * mission_odo_update
 * @brief   Adds the path travelled since the last pass to the run
 *          odometer. The board path wraps at 2^17 counts (about 42.5 m),
 *          so the run distance is summed here, pass by pass, and stops
 *          at the summary field's maximum instead of wrapping.
 * @param   p_ctx  mission context
 * @param   p_snap status board snapshot of this pass
 * @context mission_task
 */
static void mission_odo_update(mission_ctx_t * p_ctx,
                               status_board_t const * p_snap)
{
    uint32_t const path = status_board_path(p_snap);
    uint32_t const step = odometry_path_since(path, p_ctx->run_odo_mark);

    p_ctx->run_odo_mark   = path;
    p_ctx->run_odo_counts =
        ((MISSION_ODO_MAX_COUNTS - p_ctx->run_odo_counts) < step)
            ? MISSION_ODO_MAX_COUNTS : (p_ctx->run_odo_counts + step);
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
        odo  = odometry_counts_to_mm(p_ctx->run_odo_counts);
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
 * mission_snapshot
 * @brief   Reply to a remote "summary now" (APP_EVT_SNAPSHOT from the
 *          MQTT command CMD_SNAP): state, peak hump, faults, and the
 *          distance and time of the run so far. Once the run has ended
 *          they stay at the values from the moment it ended; before the
 *          first run (or after RESET) they are 0. Changes no state.
 * @context mission_task (event time: division allowed)
 */
static void mission_snapshot(mission_ctx_t const * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us)
{
    telem_bin_snapshot_t reply  = {0u, 0u, 0u, 0u, 0u, 0u};
    uint32_t             end_us = now_us;
    uint32_t             odo    = 0u;
    uint32_t             secs   = 0u;

    if (p_evt->b_has_event && (APP_EVT_SNAPSHOT == p_evt->type))
    {
        if (p_ctx->b_has_run)
        {
            if (APP_SYS_RUN != p_ctx->sys_state)
            {
                end_us = p_ctx->run_end_us;
            }
            odo  = odometry_counts_to_mm(p_ctx->run_odo_counts);
            secs = (end_us - p_ctx->run_start_us) / MISSION_US_PER_S;
        }
        reply.sys_state    = p_ctx->sys_state;
        reply.run_state    = p_ctx->run_state;
        reply.hump_peak_mm = status_board_hump_peak(p_snap);
        reply.odometry_mm  = (odo > MISSION_ODO_MAX_MM) ? MISSION_ODO_MAX_MM
                                                        : odo;
        reply.faults       = fault_get();
        reply.run_time_s   = (secs > MISSION_RUN_S_MAX) ? MISSION_RUN_S_MAX
                                                        : secs;
        (void) telem_post_snapshot(&reply);  /* a full K7 is counted there */
    }
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

/*** end of file ***/
