/**
 * @file    line.c
 * @brief   10 ms line sensing task, IR calibration, and junction events.
 * @owner   Buddy 3 (line, barcode)
 * @kind    task
 * @reqs    FR1, FR14, FR19, NFR1, NFR4
 */

#include "line.h"              /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/sys_health.h"
#include "../subsystem5/sys_tick.h"
#include "barcode.h"
#include "cfg_line.h"
#include "drv_ir_adc.h"
#include "line_track.h"

/* ---- private constants and types ------------------------------------ */

#define LINE_STACK_WORDS      (APP_STACK_LINE_BYTES / 8u)
#define LINE_REJOIN_WHITE_MIN (2u)
#define LINE_SEQ_MASK         (0xFFu)
#define LINE_RAW_LOG_CYCLES   (50u)        /* bring-up raw print: every 0.5 s */
#define LINE_LOG_CNT_MASK     (0x7FFFFFFFu)

/* ---- private data ---------------------------------------------------- */

static uint64_t           g_line_stack[LINE_STACK_WORDS];
static line_track_steer_t g_steer         = {0, 0, 0, 0, true,
                                              LINE_RECOVER_STEER, 0u, 0};

/* Packed private state words (Phase 4, B3.4). */
static uint32_t           g_prv_cal_l     = 0u; /* PRV_CAL_L: WHITE, BLACK */
static uint32_t           g_prv_cal_r     = 0u; /* PRV_CAL_R: WHITE, BLACK */
static uint32_t           g_recip_l_q16   = 0u; /* E_LINE E8 reciprocal left */
static uint32_t           g_recip_r_q16   = 0u; /* E_LINE E8 reciprocal right */
static uint32_t           g_line_prv_a    = 0u; /* PRV_LINE_A flags/cnt/JCT */
static uint32_t           g_line_prv_b    = 0u; /* PRV_LINE_B: JCT_MARK */
static uint32_t           g_line_prv_c    = 0u; /* PRV_LINE_C: RUN_MARK */
static uint32_t           g_line_prv_d    = 0u; /* PRV_LINE_D: SEQ, PATH */
/* SH_IR_RAW: last raw ADC pair for telemetry; line_task only writes. */
static volatile uint32_t  g_sh_ir_raw     = 0u;

/* Per-cycle caches for status board, rejoin latch, and bring-up logging. */
static bool               gb_rejoin_white = false;
/* Path where the current both-white stretch began (S3-U19, plain:
   add to the Week 13 packing list).                                 */
static uint32_t           g_white_mark    = 0u;
static uint32_t           g_cross_straight_until = 0u; /* path counts */
static bool               gb_branch_seen  = false;
static uint8_t            g_last_jct      = (uint8_t) APP_JCT_NONE;
static uint32_t           g_raw_log_cnt   = 0u;

/* ---- private function prototypes ------------------------------------- */

static void     line_task(INT stacd, void * p_exinf);
static void     line_cycle(void);
static void     line_raw_publish(uint32_t raw_l, uint32_t raw_r);
static void     line_log_bringup(status_board_t const * p_snap,
                                 uint32_t raw_l, uint32_t raw_r);
static void     line_reset_run_state(uint32_t path);
static bool     line_handle_modes(uint32_t raw_l, uint32_t raw_r,
                                  uint32_t path, uint32_t * p_status);
static void     line_cal_sample(uint32_t raw_l, uint32_t raw_r);
static void     line_cal_complete(void);
static void     line_update_loss_and_found(uint32_t norm_l, uint32_t norm_r,
                                           uint32_t path, bool b_inhibit_jct,
                                           uint32_t * p_status);
static void     line_handle_rejoin(bool b_is_black, uint32_t lost_cnt,
                                   uint32_t path);
static bool     line_step_junction(uint32_t norm_l, uint32_t norm_r,
                                   uint32_t path, bool b_inhibit_jct,
                                   uint32_t * p_status);
static void     line_jct_unpack(line_track_jct_t * p_jct);
static void     line_jct_pack(line_track_jct_t const * p_jct);
static uint32_t line_clamp_u(uint32_t val, uint32_t max_val);

/* ---- public functions ------------------------------------------------- */

/**
 * line_init
 * @brief   Initialises the line subsystem state before tasks start.
 * @return  E_OK
 * @context usermain
 */
ER line_init(void)
{
    line_track_steer_reset(&g_steer);
    g_prv_cal_l     = 0u;
    g_prv_cal_r     = 0u;
    g_recip_l_q16   = 0u;
    g_recip_r_q16   = 0u;
    g_line_prv_a    = 0u;
    g_line_prv_b    = 0u;
    g_line_prv_c    = 0u;
    g_line_prv_d    = 0u;
    gb_rejoin_white = false;
    gb_branch_seen  = false;
    g_cross_straight_until = 0u;
    g_last_jct      = (uint8_t) APP_JCT_NONE;
    g_raw_log_cnt   = 0u;

    return E_OK;
}

/**
 * line_start
 * @brief   Creates and starts line_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER line_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) line_task, /* cast: kernel entry type */
                             APP_PRI_LINE, g_line_stack,
                             APP_STACK_LINE_BYTES, "line_task");

    if (E_OK > tid)
    {
        ercd = tid;
    }
    else
    {
        /* Task started cleanly. */
    }

    return ercd;
}

/**
 * line_raw_word
 * @brief   Last raw IR ADC pair (SH_IR_RAW layout) for debug telemetry.
 * @return  SH_IR_RAW word: L in bits 0-11, R in bits 12-23
 * @context any task (one aligned load of a word written by line_task)
 */
uint32_t line_raw_word(void)
{
    return g_sh_ir_raw;
}

/* ---- private functions ------------------------------------------------ */

/**
 * line_clamp_u
 * @brief   Clamps an unsigned value to [0, max_val] and raises
 *          APP_FAULT_RANGE_CLAMP if clamped.
 * @param   val      Input unsigned value.
 * @param   max_val  Maximum representable value for the RNG field.
 * @return  Clamped unsigned value in 0..max_val.
 * @context line_task
 */
static uint32_t line_clamp_u(uint32_t val, uint32_t max_val)
{
    bool           b_clamped = false;
    uint32_t const clamped   = app_bits_clamp_u(val, max_val, &b_clamped);

    if (b_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
    else
    {
        /* Within representable RNG range. */
    }

    return clamped;
}

/**
 * line_jct_unpack
 * @brief   Unpacks line_track_jct_t from g_line_prv_a and g_line_prv_b.
 * @param   p_jct  Output junction state structure.
 * @return  none
 * @context line_task
 */
static void line_jct_unpack(line_track_jct_t * p_jct)
{
    uint32_t const word_a = g_line_prv_a;

    p_jct->state = app_bits_get(word_a, PRV_LINE_A_JCT_STATE_POS,
                                PRV_LINE_A_JCT_STATE_BITS);
    p_jct->b_sat_l = (0u != app_bits_get(word_a, PRV_LINE_A_SAT_L_POS,
                                         PRV_LINE_A_SAT_L_BITS));
    p_jct->b_sat_r = (0u != app_bits_get(word_a, PRV_LINE_A_SAT_R_POS,
                                         PRV_LINE_A_SAT_R_BITS));
    p_jct->seq0 = app_bits_get(word_a, PRV_LINE_A_SEQ0_POS,
                               PRV_LINE_A_SEQ0_BITS);
    p_jct->mark_counts = app_bits_get(g_line_prv_b, PRV_LINE_B_JCT_MARK_POS,
                                      PRV_LINE_B_JCT_MARK_BITS);
}

/**
 * line_jct_pack
 * @brief   Packs line_track_jct_t into g_line_prv_a and g_line_prv_b.
 * @param   p_jct  Input junction state structure.
 * @return  none
 * @context line_task
 */
static void line_jct_pack(line_track_jct_t const * p_jct)
{
    uint32_t       word_a   = g_line_prv_a;
    uint32_t const mark_enc = line_clamp_u(
        p_jct->mark_counts, (uint32_t) PRV_LINE_B_JCT_MARK_MAX);

    word_a = app_bits_put(word_a, PRV_LINE_A_JCT_STATE_POS,
                          PRV_LINE_A_JCT_STATE_BITS, p_jct->state);
    word_a = app_bits_put(word_a, PRV_LINE_A_SAT_L_POS,
                          PRV_LINE_A_SAT_L_BITS, p_jct->b_sat_l ? 1u : 0u);
    word_a = app_bits_put(word_a, PRV_LINE_A_SAT_R_POS,
                          PRV_LINE_A_SAT_R_BITS, p_jct->b_sat_r ? 1u : 0u);
    word_a = app_bits_put(word_a, PRV_LINE_A_SEQ0_POS,
                          PRV_LINE_A_SEQ0_BITS,
                          p_jct->seq0 & LINE_SEQ_MASK);
    g_line_prv_a = word_a;
    g_line_prv_b = app_bits_put(g_line_prv_b, PRV_LINE_B_JCT_MARK_POS,
                                PRV_LINE_B_JCT_MARK_BITS, mark_enc);
}

/**
 * line_task
 * @brief   10 ms periodic line sensing task woken by R2 (sys_tick).
 * @param   stacd    Start code from tk_sta_tsk (unused).
 * @param   p_exinf  Extended information (unused).
 * @return  never returns
 * @context line_task
 */
static void line_task(INT stacd, void * p_exinf)
{
    (void) stacd;   /* unused: kernel start code */
    (void) p_exinf; /* unused: no extended info  */

    drv_ir_adc_init();
    (void) sys_tick_register(SYS_TICK_SLOT_LINE, tk_get_tid());

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        sys_tick_cycle_start(SYS_TICK_SLOT_LINE);
        sys_health_check_in(SYS_HEALTH_BIT_LINE);
        line_cycle();
    }
}

/**
 * line_log_bringup
 * @brief   Prints raw IR ADC, wheel positions, and barcode DO diagnostics
 *          every 0.5 s while the motors are disabled.
 * @param   p_snap  Current status board snapshot.
 * @param   raw_l   Raw left ADC reading.
 * @param   raw_r   Raw right ADC reading.
 * @return  none
 * @context line_task
 */
static void line_log_bringup(status_board_t const * p_snap,
                             uint32_t raw_l, uint32_t raw_r)
{
    if (!mission_mode_is_set(APP_MODE_MOTOR_EN))
    {
        g_raw_log_cnt++;
        if (g_raw_log_cnt >= LINE_RAW_LOG_CYCLES)
        {
            g_raw_log_cnt = 0u;
            /* casts: ADC codes 0..4095 and level 0..1 fit int32_t */
            app_log("IR raw L %d R %d\n", (int32_t) raw_l,
                    (int32_t) raw_r, 0);
            app_log("ENC pos L %d R %d\n", status_board_pos_l(p_snap),
                    status_board_pos_r(p_snap), 0);
            app_log("BAR DO %d (1 = black), edges %d\n",
                    (int32_t) barcode_do_level(),
                    (int32_t) (barcode_edge_count() & LINE_LOG_CNT_MASK), 0);
        }
        else
        {
            /* Wait for 0.5 s divider. */
        }
    }
    else
    {
        /* Suppress periodic bring-up log while motors are enabled. */
    }
}

/**
 * line_cycle
 * @brief   Executes one 10 ms pass of line sensing, calibration, loss/rejoin,
 *          junction classification, and status board publishing.
 * @return  none
 * @context line_task
 */
static void line_cycle(void)
{
    uint32_t            raw_l    = 0u;
    uint32_t            raw_r    = 0u;
    status_board_t      snap     = {0};
    status_board_line_t part     = {0};
    uint32_t            path     = 0u;
    int32_t             steering = 0;
    uint32_t            status   = APP_LINE_LOST;
    bool                b_cal_ok = false;

    drv_ir_adc_read(&raw_l, &raw_r);
    line_raw_publish(raw_l, raw_r);
    status_board_read(&snap);
    line_log_bringup(&snap, raw_l, raw_r);
    path = status_board_path(&snap);

    if (line_handle_modes(raw_l, raw_r, path, &status))
    {
        /* Mode active (RUN_START or CAL_IR): skip steps 4-6. */
    }
    else
    {
        b_cal_ok = (0u != app_bits_get(g_line_prv_a, PRV_LINE_A_CALIBRATED_POS,
                                       PRV_LINE_A_CALIBRATED_BITS));
        if (!b_cal_ok)
        {
            steering = 0;
            status   = APP_LINE_LOST;
        }
        else
        {
            uint32_t const white_l = app_bits_get(
                g_prv_cal_l, PRV_CAL_L_WHITE_POS, PRV_CAL_L_WHITE_BITS);
            uint32_t const black_l = app_bits_get(
                g_prv_cal_l, PRV_CAL_L_BLACK_POS, PRV_CAL_L_BLACK_BITS);
            uint32_t const white_r = app_bits_get(
                g_prv_cal_r, PRV_CAL_R_WHITE_POS, PRV_CAL_R_WHITE_BITS);
            uint32_t const black_r = app_bits_get(
                g_prv_cal_r, PRV_CAL_R_BLACK_POS, PRV_CAL_R_BLACK_BITS);
            uint32_t const norm_l = line_track_normalise(
                raw_l, white_l, black_l, g_recip_l_q16);
            uint32_t const norm_r = line_track_normalise(
                raw_r, white_r, black_r, g_recip_r_q16);
            bool const b_inhibit_jct =
                mission_mode_is_set(APP_MODE_JUNCTION_INHIBIT);

            steering = line_track_steer_step(&g_steer, norm_l, norm_r);
            line_update_loss_and_found(norm_l, norm_r, path,
                                       b_inhibit_jct, &status);
            if (line_step_junction(norm_l, norm_r, path, b_inhibit_jct,
                                    &status))
            {
                line_track_steer_reset(&g_steer);
                steering = 0;
            }
        }
    }

    b_cal_ok = (0u != app_bits_get(g_line_prv_a, PRV_LINE_A_CALIBRATED_POS,
                                   PRV_LINE_A_CALIBRATED_BITS));
    part.steering        = steering;
    part.line_status     = status;
    /* cast: widen 8-bit cached junction class to uint32_t */
    part.junction        = (uint32_t) g_last_jct;
    part.b_is_calibrated = b_cal_ok;
    status_board_write_line(&part);
}

/**
 * line_reset_run_state
 * @brief   Resets per-run line and junction tracking while RUN_START is set.
 * @param   path  Current odometry path counts.
 * @return  none
 * @context line_task
 */
static void line_reset_run_state(uint32_t path)
{
    line_track_jct_t jct      = {0u, 0u, false, false, 0u};
    uint32_t const   cur_seq  = barcode_branch_seq() & LINE_SEQ_MASK;
    uint32_t const   path_enc = line_clamp_u(
        path, (uint32_t) PRV_LINE_D_BRANCH_PATH_MAX);
    uint32_t         word_a   = 0u;
    uint32_t         word_d   = 0u;

    line_track_jct_reset(&jct);
    line_jct_pack(&jct);
    line_track_steer_reset(&g_steer);

    word_a = g_line_prv_a;
    word_a = app_bits_put(word_a, PRV_LINE_A_LOST_CNT_POS,
                          PRV_LINE_A_LOST_CNT_BITS, 0u);
    word_a = app_bits_put(word_a, PRV_LINE_A_LOST_SENT_POS,
                          PRV_LINE_A_LOST_SENT_BITS, 0u);
    word_a = app_bits_put(word_a, PRV_LINE_A_RUN_ACTIVE_POS,
                          PRV_LINE_A_RUN_ACTIVE_BITS, 0u);
    g_line_prv_a = word_a;
    g_line_prv_c = 0u;

    word_d = app_bits_put(word_d, PRV_LINE_D_LAST_SEQ_POS,
                          PRV_LINE_D_LAST_SEQ_BITS, cur_seq);
    word_d = app_bits_put(word_d, PRV_LINE_D_BRANCH_PATH_POS,
                          PRV_LINE_D_BRANCH_PATH_BITS, path_enc);
    g_line_prv_d    = word_d;
    gb_rejoin_white = false;
    gb_branch_seen  = false;
    g_cross_straight_until = 0u;
    g_last_jct      = (uint8_t) APP_JCT_NONE;
}

/**
 * line_cal_sample
 * @brief   Accumulates min/max raw ADC into PRV_CAL_L and PRV_CAL_R.
 * @param   raw_l  Raw left ADC code.
 * @param   raw_r  Raw right ADC code.
 * @return  none
 * @context line_task
 */
static void line_cal_sample(uint32_t raw_l, uint32_t raw_r)
{
    uint32_t const cl_l = line_clamp_u(raw_l, (uint32_t) PRV_CAL_L_WHITE_MAX);
    uint32_t const cl_r = line_clamp_u(raw_r, (uint32_t) PRV_CAL_R_WHITE_MAX);
    bool const     b_act = (0u != app_bits_get(
        g_line_prv_a, PRV_LINE_A_CAL_ACTIVE_POS, PRV_LINE_A_CAL_ACTIVE_BITS));
    uint32_t min_l = cl_l;
    uint32_t max_l = cl_l;
    uint32_t min_r = cl_r;
    uint32_t max_r = cl_r;

    if (b_act)
    {
        uint32_t const cur_min_l = app_bits_get(
            g_prv_cal_l, PRV_CAL_L_WHITE_POS, PRV_CAL_L_WHITE_BITS);
        uint32_t const cur_max_l = app_bits_get(
            g_prv_cal_l, PRV_CAL_L_BLACK_POS, PRV_CAL_L_BLACK_BITS);
        uint32_t const cur_min_r = app_bits_get(
            g_prv_cal_r, PRV_CAL_R_WHITE_POS, PRV_CAL_R_WHITE_BITS);
        uint32_t const cur_max_r = app_bits_get(
            g_prv_cal_r, PRV_CAL_R_BLACK_POS, PRV_CAL_R_BLACK_BITS);

        min_l = (cl_l < cur_min_l) ? cl_l : cur_min_l;
        max_l = (cl_l > cur_max_l) ? cl_l : cur_max_l;
        min_r = (cl_r < cur_min_r) ? cl_r : cur_min_r;
        max_r = (cl_r > cur_max_r) ? cl_r : cur_max_r;
    }
    else
    {
        g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_CAL_ACTIVE_POS,
                                    PRV_LINE_A_CAL_ACTIVE_BITS, 1u);
    }

    g_prv_cal_l = app_bits_put(0u, PRV_CAL_L_WHITE_POS,
                               PRV_CAL_L_WHITE_BITS, min_l);
    g_prv_cal_l = app_bits_put(g_prv_cal_l, PRV_CAL_L_BLACK_POS,
                               PRV_CAL_L_BLACK_BITS, max_l);
    g_prv_cal_r = app_bits_put(0u, PRV_CAL_R_WHITE_POS,
                               PRV_CAL_R_WHITE_BITS, min_r);
    g_prv_cal_r = app_bits_put(g_prv_cal_r, PRV_CAL_R_BLACK_POS,
                               PRV_CAL_R_BLACK_BITS, max_r);
}

/**
 * line_cal_complete
 * @brief   Finalises calibration when APP_MODE_CAL_IR clears.
 * @return  none
 * @context line_task
 */
static void line_cal_complete(void)
{
    line_track_cal_t cal   = {0u, 0u, 0u, 0u, 0u, 0u};
    uint32_t const   min_l = app_bits_get(
        g_prv_cal_l, PRV_CAL_L_WHITE_POS, PRV_CAL_L_WHITE_BITS);
    uint32_t const   max_l = app_bits_get(
        g_prv_cal_l, PRV_CAL_L_BLACK_POS, PRV_CAL_L_BLACK_BITS);
    uint32_t const   min_r = app_bits_get(
        g_prv_cal_r, PRV_CAL_R_WHITE_POS, PRV_CAL_R_WHITE_BITS);
    uint32_t const   max_r = app_bits_get(
        g_prv_cal_r, PRV_CAL_R_BLACK_POS, PRV_CAL_R_BLACK_BITS);

    g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_CAL_ACTIVE_POS,
                                PRV_LINE_A_CAL_ACTIVE_BITS, 0u);

    if (line_track_cal_finish(&cal, min_l, max_l, min_r, max_r))
    {
        g_prv_cal_l = app_bits_put(0u, PRV_CAL_L_WHITE_POS,
                                   PRV_CAL_L_WHITE_BITS, cal.white_l);
        g_prv_cal_l = app_bits_put(g_prv_cal_l, PRV_CAL_L_BLACK_POS,
                                   PRV_CAL_L_BLACK_BITS, cal.black_l);
        g_prv_cal_r = app_bits_put(0u, PRV_CAL_R_WHITE_POS,
                                   PRV_CAL_R_WHITE_BITS, cal.white_r);
        g_prv_cal_r = app_bits_put(g_prv_cal_r, PRV_CAL_R_BLACK_POS,
                                   PRV_CAL_R_BLACK_BITS, cal.black_r);
        g_recip_l_q16 = cal.recip_l_q16;
        g_recip_r_q16 = cal.recip_r_q16;
        g_line_prv_a  = app_bits_put(g_line_prv_a, PRV_LINE_A_CALIBRATED_POS,
                                     PRV_LINE_A_CALIBRATED_BITS, 1u);
        /* casts: ADC codes 0..4095 fit int32_t */
        app_log("IR cal OK: span L %d R %d\n",
                (int32_t) (max_l - min_l), (int32_t) (max_r - min_r), 0);
        (void) mission_post_event(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
    }
    else
    {
        g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_CALIBRATED_POS,
                                    PRV_LINE_A_CALIBRATED_BITS, 0u);
        /* casts: ADC codes 0..4095 fit int32_t */
        app_log("IR cal FAILED: span L %d R %d (need %d each);"
                " ensure both sensors cross black and white\n",
                (int32_t) (max_l - min_l), (int32_t) (max_r - min_r),
                (int32_t) LINE_CAL_MIN_SPAN);
    }
}

/**
 * line_handle_modes
 * @brief   Handles APP_MODE_RUN_START and APP_MODE_CAL_IR (step 3).
 * @param   raw_l     Raw left ADC reading.
 * @param   raw_r     Raw right ADC reading.
 * @param   path      Current odometry path counts.
 * @param   p_status  Output line status if skipping steps 4-6.
 * @return  true if steps 4-6 must be skipped this cycle, else false.
 * @context line_task
 */
static bool line_handle_modes(uint32_t raw_l, uint32_t raw_r,
                              uint32_t path, uint32_t * p_status)
{
    bool       b_skip     = false;
    bool const b_run_init = mission_mode_is_set(APP_MODE_RUN_START);
    bool const b_cal_mode = mission_mode_is_set(APP_MODE_CAL_IR);
    bool const b_cal_act  = (0u != app_bits_get(
        g_line_prv_a, PRV_LINE_A_CAL_ACTIVE_POS, PRV_LINE_A_CAL_ACTIVE_BITS));

    if (b_run_init)
    {
        bool const b_cal_ok = (0u != app_bits_get(
            g_line_prv_a, PRV_LINE_A_CALIBRATED_POS,
            PRV_LINE_A_CALIBRATED_BITS));

        line_reset_run_state(path);
        *p_status = b_cal_ok ? APP_LINE_ON : APP_LINE_LOST;
        b_skip    = true;
    }
    else if (b_cal_mode)
    {
        line_cal_sample(raw_l, raw_r);
        *p_status = APP_LINE_CALIBRATING;
        b_skip    = true;
    }
    else if (b_cal_act)
    {
        line_cal_complete();
    }
    else
    {
        /* Normal running cycle. */
    }

    return b_skip;
}

/**
 * line_update_loss_and_found
 * @brief   Updates white-cycle counter, LINE_LOST, and LINE_FOUND events.
 * @param   norm_l         Left normalised reading (0..1000).
 * @param   norm_r         Right normalised reading (0..1000).
 * @param   path           Current odometry path counts.
 * @param   b_inhibit_jct  true when APP_MODE_JUNCTION_INHIBIT is set.
 * @param   p_status       Output line status (APP_LINE_ON or APP_LINE_LOST).
 * @return  none
 * @context line_task
 */
static void line_update_loss_and_found(uint32_t norm_l, uint32_t norm_r,
                                       uint32_t path, bool b_inhibit_jct,
                                       uint32_t * p_status)
{
    bool const b_is_white =
        ((norm_l < LINE_WHITE_THRESHOLD) && (norm_r < LINE_WHITE_THRESHOLD));
    bool const b_is_black =
        ((norm_l >= LINE_BLACK_THRESHOLD) || (norm_r >= LINE_BLACK_THRESHOLD));
    uint32_t lost_cnt   = app_bits_get(
        g_line_prv_a, PRV_LINE_A_LOST_CNT_POS, PRV_LINE_A_LOST_CNT_BITS);
    bool     b_lost_snt = (0u != app_bits_get(
        g_line_prv_a, PRV_LINE_A_LOST_SENT_POS, PRV_LINE_A_LOST_SENT_BITS));
    bool     b_is_between = false;
    bool     b_is_waiting = false;
    uint32_t white_mm     = 0u;

    if (b_is_white)
    {
        if (0u == lost_cnt)
        {
            g_white_mark = path;          /* both white from here */
        }
        if (lost_cnt < (uint32_t) PRV_LINE_A_LOST_CNT_MAX)
        {
            lost_cnt++;
        }
        else
        {
            /* Saturate at 31 (5-bit LOST_CNT). */
        }

        /* Centred (line between the sensors, S3-U19): never lost. The
           track has no line end and the car keeps following; a distance
           limit here also fired after a barcode rewind, whose reverse
           travel counts in the path (on-car 12 Nov, S3-U44). After a
           black exit: lost after LINE_EXIT_LOST_MM (S3-U20).           */
        white_mm     = odometry_counts_to_mm(path - g_white_mark);
        b_is_between = line_track_steer_line_between(&g_steer);
        b_is_waiting = (!line_track_steer_line_between(&g_steer))
                       && (white_mm < LINE_EXIT_LOST_MM);

        if ((lost_cnt >= LINE_LOST_CYCLES) && (!b_lost_snt)
            && (!b_is_between) && (!b_is_waiting))
        {
            (void) mission_post_event(APP_EVT_LINE_LOST, 0u);
            b_lost_snt = true;
        }
        else
        {
            /* Not yet at loss threshold, centred, or already reported. */
        }
        *p_status = (b_is_between && (!b_lost_snt)) ? APP_LINE_ON
                                                    : APP_LINE_LOST;
    }
    else
    {
        lost_cnt  = 0u;
        *p_status = APP_LINE_ON;
    }

    g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_LOST_CNT_POS,
                                PRV_LINE_A_LOST_CNT_BITS, lost_cnt);
    g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_LOST_SENT_POS,
                                PRV_LINE_A_LOST_SENT_BITS,
                                b_lost_snt ? 1u : 0u);

    if (b_inhibit_jct)
    {
        line_handle_rejoin(b_is_black, lost_cnt, path);
    }
    else
    {
        gb_rejoin_white = false;
        g_line_prv_a    = app_bits_put(g_line_prv_a, PRV_LINE_A_RUN_ACTIVE_POS,
                                       PRV_LINE_A_RUN_ACTIVE_BITS, 0u);
        if (b_is_black && b_lost_snt)
        {
            (void) mission_post_event(APP_EVT_LINE_FOUND, 0u);
            g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_LOST_SENT_POS,
                                        PRV_LINE_A_LOST_SENT_BITS, 0u);
        }
        else
        {
            /* Not searching or still on white. */
        }
    }
}

/**
 * line_handle_rejoin
 * @brief   Measures every black run after >= 2 white cycles in rejoin mode
 *          and posts APP_EVT_LINE_FOUND when the run ends or reaches 255 mm.
 *          Grey cycles at a 45-deg line edge do not clear gb_rejoin_white
 *          (on-car log 2026-11-03, S3-U15).
 * @param   b_is_black  true if either channel >= LINE_BLACK_THRESHOLD.
 * @param   lost_cnt    White-cycle count updated this cycle.
 * @param   path        Current odometry path counts.
 * @return  none
 * @context line_task
 */
static void line_handle_rejoin(bool b_is_black, uint32_t lost_cnt,
                               uint32_t path)
{
    bool b_run_act = (0u != app_bits_get(
        g_line_prv_a, PRV_LINE_A_RUN_ACTIVE_POS, PRV_LINE_A_RUN_ACTIVE_BITS));
    uint32_t run_mark = app_bits_get(
        g_line_prv_c, PRV_LINE_C_RUN_MARK_POS, PRV_LINE_C_RUN_MARK_BITS);

    if (lost_cnt >= LINE_REJOIN_WHITE_MIN)
    {
        gb_rejoin_white = true;
    }
    else
    {
        /* Black or grey this cycle, or white too short so far. */
    }

    if (b_is_black)
    {
        if ((!b_run_act) && gb_rejoin_white)
        {
            uint32_t const mark_enc = line_clamp_u(
                path, (uint32_t) PRV_LINE_C_RUN_MARK_MAX);

            b_run_act       = true;
            gb_rejoin_white = false;
            run_mark        = mark_enc;
            g_line_prv_c = app_bits_put(g_line_prv_c, PRV_LINE_C_RUN_MARK_POS,
                                        PRV_LINE_C_RUN_MARK_BITS, mark_enc);
            g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_RUN_ACTIVE_POS,
                                        PRV_LINE_A_RUN_ACTIVE_BITS, 1u);
        }
        else
        {
            /* Either already in a black run or < 2 prior white cycles. */
        }

        if (b_run_act &&
            (odometry_counts_to_mm(path - run_mark) >= LINE_FOUND_MAX_MM))
        {
            (void) mission_post_event(APP_EVT_LINE_FOUND, LINE_FOUND_MAX_MM);
            g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_RUN_ACTIVE_POS,
                                        PRV_LINE_A_RUN_ACTIVE_BITS, 0u);
            g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_LOST_SENT_POS,
                                        PRV_LINE_A_LOST_SENT_BITS, 0u);
        }
        else
        {
            /* Continue measuring active black run or waiting for white. */
        }
    }
    else if (b_run_act)
    {
        uint32_t cross_mm = odometry_counts_to_mm(path - run_mark);

        if (cross_mm > LINE_FOUND_MAX_MM)
        {
            cross_mm = LINE_FOUND_MAX_MM;
        }
        else
        {
            /* Within 0..255 mm. */
        }
        (void) mission_post_event(APP_EVT_LINE_FOUND, cross_mm);
        g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_RUN_ACTIVE_POS,
                                    PRV_LINE_A_RUN_ACTIVE_BITS, 0u);
        g_line_prv_a = app_bits_put(g_line_prv_a, PRV_LINE_A_LOST_SENT_POS,
                                    PRV_LINE_A_LOST_SENT_BITS, 0u);
    }
    else
    {
        /* White/grey with no black run ending. */
    }
}

/**
 * line_step_junction
 * @brief   Tracks barcode branch sequence and steps the junction classifier.
 * @param   norm_l         Left normalised reading (0..1000).
 * @param   norm_r         Right normalised reading (0..1000).
 * @param   path           Current odometry path counts.
 * @param   b_inhibit_jct  true when APP_MODE_JUNCTION_INHIBIT is set.
 * @param   p_status       Updated to APP_LINE_JUNCTION while active.
 * @return  true when the cross stripe should clear the steering bias.
 * @context line_task
 */
static bool line_step_junction(uint32_t norm_l, uint32_t norm_r,
                               uint32_t path, bool b_inhibit_jct,
                               uint32_t * p_status)
{
    uint32_t const cur_seq  = barcode_branch_seq() & LINE_SEQ_MASK;
    uint32_t const last_seq = app_bits_get(
        g_line_prv_d, PRV_LINE_D_LAST_SEQ_POS, PRV_LINE_D_LAST_SEQ_BITS);
    uint32_t branch_path = app_bits_get(
        g_line_prv_d, PRV_LINE_D_BRANCH_PATH_POS, PRV_LINE_D_BRANCH_PATH_BITS);
    line_track_jct_t jct = {0u, 0u, false, false, 0u};
    bool b_reset_steer = false;

    if (cur_seq != last_seq)
    {
        uint32_t const path_enc = line_clamp_u(
            path, (uint32_t) PRV_LINE_D_BRANCH_PATH_MAX);
        uint32_t word_d = 0u;

        word_d = app_bits_put(word_d, PRV_LINE_D_LAST_SEQ_POS,
                              PRV_LINE_D_LAST_SEQ_BITS, cur_seq);
        word_d = app_bits_put(word_d, PRV_LINE_D_BRANCH_PATH_POS,
                              PRV_LINE_D_BRANCH_PATH_BITS, path_enc);
        g_line_prv_d   = word_d;
        branch_path    = path_enc;
        gb_branch_seen = true;
    }
    else
    {
        /* Branch counter unchanged this cycle. */
    }

    line_jct_unpack(&jct);

    if (b_inhibit_jct)
    {
        g_cross_straight_until = 0u;
        line_track_jct_reset(&jct);
        line_jct_pack(&jct);
    }
    else
    {
        line_track_jct_in_t jct_in    = {0};
        uint32_t            jct_class = APP_JCT_NONE;
        uint32_t const      early_cnt =
            odometry_mm_to_counts(LINE_BRANCH_EARLY_MM);
        bool const          b_was_on_band =
            ((LINE_JCT_STATE_CANDIDATE == jct.state)
             && jct.b_sat_l && jct.b_sat_r);

        jct_in.norm_l          = norm_l;
        jct_in.norm_r          = norm_r;
        jct_in.path_counts     = path;
        jct_in.branch_seq      = cur_seq;
        jct_in.min_counts      = odometry_mm_to_counts(LINE_JUNCTION_MIN_MM);
        jct_in.confirm_counts  =
            odometry_mm_to_counts(LINE_JUNCTION_CONFIRM_MM);
        jct_in.b_branch_recent =
            (gb_branch_seen && ((path - branch_path) <= early_cnt));

        jct_class = line_track_jct_step(&jct, &jct_in);
        line_jct_pack(&jct);

        /* A skewed stripe can leave one sensor dark for several cycles.
           Clear steering bias until the stripe clears or travel expires,
           even if the stripe did not satisfy the junction classifier. */
        if ((norm_l >= LINE_BLACK_THRESHOLD)
            && (norm_r >= LINE_BLACK_THRESHOLD))
        {
            g_cross_straight_until =
                path + odometry_mm_to_counts(LINE_JUNCTION_EXIT_MM);
        }
        else if ((norm_l < LINE_WHITE_THRESHOLD)
                 && (norm_r < LINE_WHITE_THRESHOLD))
        {
            g_cross_straight_until = 0u;
        }
        else
        {
            /* One sensor is still on the stripe or a line. */
        }

        /* On the confirm cycle the bias is cleared only while tape is
           still under a sensor; centred and both white, steering is
           left alone.                                               */
        b_reset_steer = (path < g_cross_straight_until)
            || b_was_on_band
            || ((APP_JCT_CROSS == jct_class)
                && ((norm_l >= LINE_WHITE_THRESHOLD)
                    || (norm_r >= LINE_WHITE_THRESHOLD)))
            || ((LINE_JCT_STATE_CANDIDATE == jct.state) && jct.b_sat_l
                && jct.b_sat_r);

        if (APP_JCT_NONE != jct_class)
        {
            (void) mission_post_event(APP_EVT_JUNCTION, jct_class);
            /* cast: APP_JCT_* enum 0..3 fits in uint8_t */
            g_last_jct     = (uint8_t) jct_class;
            gb_branch_seen = false;
        }
        else
        {
            /* No junction confirmed on this cycle. */
        }

        if (LINE_JCT_STATE_IDLE != jct.state)
        {
            *p_status = APP_LINE_JUNCTION;
        }
        else
        {
            /* Keep ON / LOST status. */
        }
    }

    return b_reset_steer;
}

/**
 * line_raw_publish
 * @brief   Packs this cycle's raw ADC pair into SH_IR_RAW and stores the
 *          whole word at once (single writer, so readers never see half
 *          an update).
 * @param   raw_l  left ADC reading, 0..4095
 * @param   raw_r  right ADC reading, 0..4095
 * @context line_task
 */
static void line_raw_publish(uint32_t raw_l, uint32_t raw_r)
{
    uint32_t word = 0u;

    word = app_bits_put(word, SH_IR_RAW_L_POS, SH_IR_RAW_L_BITS,
                        line_clamp_u(raw_l, (uint32_t) SH_IR_RAW_L_MAX));
    word = app_bits_put(word, SH_IR_RAW_R_POS, SH_IR_RAW_R_BITS,
                        line_clamp_u(raw_r, (uint32_t) SH_IR_RAW_R_MAX));
    /* casts above: the layout maxima are the positive constant 4095 */
    g_sh_ir_raw = word;
}

/*** end of file ***/
