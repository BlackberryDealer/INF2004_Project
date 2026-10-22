/**
 * @file    line.c
 * @brief   10 ms line sensing task, IR calibration, and junction events.
 * @owner   Buddy 3 (line, barcode)
 * @kind    task
 * @reqs    FR1, FR14, FR19, NFR1, NFR4
 */

#include "line.h"              /* own header first (BARR-C 4.3.c) */
#include "../app.h"
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
#define LINE_LOST_CNT_MAX     (31u)
#define LINE_REJOIN_WHITE_MIN (2u)
#define LINE_ADC_MAX_INIT     (4095u)

/* ---- private data ---------------------------------------------------- */

static uint64_t         g_line_stack[LINE_STACK_WORDS];
static line_track_cal_t g_cal             = {0u, 0u, 0u, 0u, 0u, 0u};
static line_track_jct_t g_jct             = {0u, 0u, false, false, 0u};
static uint32_t         g_cal_min_l       = LINE_ADC_MAX_INIT;
static uint32_t         g_cal_max_l       = 0u;
static uint32_t         g_cal_min_r       = LINE_ADC_MAX_INIT;
static uint32_t         g_cal_max_r       = 0u;
static uint32_t         g_lost_cnt        = 0u;
static bool             gb_lost_sent      = false;
static bool             gb_run_active     = false;
static bool             gb_cal_active     = false;
static bool             gb_is_calibrated  = false;
static bool             gb_branch_seen    = false;
static uint32_t         g_run_mark        = 0u;
static uint32_t         g_last_branch_seq = 0u;
static uint32_t         g_branch_path     = 0u;
static uint32_t         g_last_jct        = APP_JCT_NONE;
static uint32_t         g_jct_min_counts  = 0u;
static uint32_t         g_jct_conf_counts = 0u;
static uint32_t         g_branch_early_c  = 0u;

/* ---- private function prototypes ------------------------------------- */

static void line_task(INT stacd, void * p_exinf);
static void line_init_thresholds(void);
static void line_cycle(void);
static void line_reset_run_state(uint32_t path);
static bool line_handle_modes(uint32_t raw_l, uint32_t raw_r,
                              uint32_t path, uint32_t * p_status);
static void line_update_loss_and_found(uint32_t norm_l, uint32_t norm_r,
                                       uint32_t path, bool b_inhibit_jct,
                                       uint32_t * p_status);
static void line_handle_rejoin(bool b_is_black, uint32_t prev_lost_cnt,
                               uint32_t path);
static void line_step_junction(uint32_t norm_l, uint32_t norm_r,
                               uint32_t path, bool b_inhibit_jct,
                               uint32_t * p_status);

/* ---- public functions ------------------------------------------------- */

/**
 * line_init
 * @brief   Initialises the line subsystem state before tasks start.
 * @return  E_OK
 * @context usermain
 */
ER line_init(void)
{
    line_track_jct_reset(&g_jct);
    g_cal_min_l       = LINE_ADC_MAX_INIT;
    g_cal_max_l       = 0u;
    g_cal_min_r       = LINE_ADC_MAX_INIT;
    g_cal_max_r       = 0u;
    g_lost_cnt        = 0u;
    gb_lost_sent      = false;
    gb_run_active     = false;
    gb_cal_active     = false;
    gb_is_calibrated  = false;
    gb_branch_seen    = false;
    g_run_mark        = 0u;
    g_last_branch_seq = 0u;
    g_branch_path     = 0u;
    g_last_jct        = APP_JCT_NONE;
    line_init_thresholds();

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

/* ---- private functions ------------------------------------------------ */

/**
 * line_init_thresholds
 * @brief   Converts mm junction thresholds to encoder counts once.
 * @return  none
 * @context usermain / line_task startup
 */
static void line_init_thresholds(void)
{
    g_jct_min_counts  = odometry_mm_to_counts(LINE_JUNCTION_MIN_MM);
    g_jct_conf_counts = odometry_mm_to_counts(LINE_JUNCTION_CONFIRM_MM);
    g_branch_early_c  = odometry_mm_to_counts(LINE_BRANCH_EARLY_MM);
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
    line_init_thresholds();
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

    drv_ir_adc_read(&raw_l, &raw_r);
    status_board_read(&snap);
    path = status_board_path(&snap);

    if (line_handle_modes(raw_l, raw_r, path, &status))
    {
        /* Mode active (RUN_START or CAL_IR): skip steps 4-6. */
    }
    else if (!gb_is_calibrated)
    {
        steering = 0;
        status   = APP_LINE_LOST;
    }
    else
    {
        uint32_t const norm_l = line_track_normalise(
            raw_l, g_cal.white_l, g_cal.black_l, g_cal.recip_l_q16);
        uint32_t const norm_r = line_track_normalise(
            raw_r, g_cal.white_r, g_cal.black_r, g_cal.recip_r_q16);
        bool const b_inhibit_jct =
            mission_mode_is_set(APP_MODE_JUNCTION_INHIBIT);

        steering = line_track_steering(norm_l, norm_r);
        line_update_loss_and_found(norm_l, norm_r, path,
                                   b_inhibit_jct, &status);
        line_step_junction(norm_l, norm_r, path, b_inhibit_jct, &status);
    }

    part.steering        = steering;
    part.line_status     = status;
    part.junction        = g_last_jct;
    part.b_is_calibrated = gb_is_calibrated;
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
    line_track_jct_reset(&g_jct);
    g_lost_cnt        = 0u;
    gb_lost_sent      = false;
    gb_run_active     = false;
    g_run_mark        = 0u;
    g_last_branch_seq = barcode_branch_seq();
    g_branch_path     = path;
    gb_branch_seen    = false;
    g_last_jct        = APP_JCT_NONE;
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

    if (b_run_init)
    {
        line_reset_run_state(path);
        *p_status = gb_is_calibrated ? APP_LINE_ON : APP_LINE_LOST;
        b_skip    = true;
    }
    else if (b_cal_mode)
    {
        if (!gb_cal_active)
        {
            gb_cal_active = true;
            g_cal_min_l   = raw_l;
            g_cal_max_l   = raw_l;
            g_cal_min_r   = raw_r;
            g_cal_max_r   = raw_r;
        }
        else
        {
            g_cal_min_l = (raw_l < g_cal_min_l) ? raw_l : g_cal_min_l;
            g_cal_max_l = (raw_l > g_cal_max_l) ? raw_l : g_cal_max_l;
            g_cal_min_r = (raw_r < g_cal_min_r) ? raw_r : g_cal_min_r;
            g_cal_max_r = (raw_r > g_cal_max_r) ? raw_r : g_cal_max_r;
        }
        *p_status = APP_LINE_CALIBRATING;
        b_skip    = true;
    }
    else if (gb_cal_active)
    {
        gb_cal_active = false;
        if (line_track_cal_finish(&g_cal, g_cal_min_l, g_cal_max_l,
                                  g_cal_min_r, g_cal_max_r))
        {
            gb_is_calibrated = true;
            /* casts: ADC codes 0..4095 fit int32_t */
            app_log("IR cal OK: span L %d R %d\n",
                    (int32_t) (g_cal_max_l - g_cal_min_l),
                    (int32_t) (g_cal_max_r - g_cal_min_r), 0);
            (void) mission_post_event(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
        }
        else
        {
            gb_is_calibrated = false;
            /* Bring-up diagnosis: without this line the car just waits
               in CAL_IR until the 10 s CAL_TIMEOUT fault. casts: ADC
               codes 0..4095 fit int32_t                                */
            app_log("IR cal FAILED: span L %d R %d (need %d each);"
                    " ensure both sensors cross black and white\n",
                    (int32_t) (g_cal_max_l - g_cal_min_l),
                    (int32_t) (g_cal_max_r - g_cal_min_r),
                    (int32_t) LINE_CAL_MIN_SPAN);
        }
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
    uint32_t const prev_lost_cnt = g_lost_cnt;

    if (b_is_white)
    {
        if (g_lost_cnt < LINE_LOST_CNT_MAX)
        {
            g_lost_cnt++;
        }
        else
        {
            /* Saturate at 31 (5-bit LOST_CNT). */
        }

        if ((g_lost_cnt >= LINE_LOST_CYCLES) && (!gb_lost_sent))
        {
            (void) mission_post_event(APP_EVT_LINE_LOST, 0u);
            gb_lost_sent = true;
        }
        else
        {
            /* Not yet at loss threshold or already reported. */
        }
        *p_status = APP_LINE_LOST;
    }
    else
    {
        g_lost_cnt = 0u;
        *p_status  = APP_LINE_ON;
    }

    if (b_inhibit_jct)
    {
        line_handle_rejoin(b_is_black, prev_lost_cnt, path);
    }
    else
    {
        gb_run_active = false;
        if (b_is_black && gb_lost_sent)
        {
            (void) mission_post_event(APP_EVT_LINE_FOUND, 0u);
            gb_lost_sent = false;
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
 * @param   b_is_black     true if either channel >= LINE_BLACK_THRESHOLD.
 * @param   prev_lost_cnt  White cycle count immediately before this cycle.
 * @param   path           Current odometry path counts.
 * @return  none
 * @context line_task
 */
static void line_handle_rejoin(bool b_is_black, uint32_t prev_lost_cnt,
                               uint32_t path)
{
    if (b_is_black)
    {
        if ((!gb_run_active) && (prev_lost_cnt >= LINE_REJOIN_WHITE_MIN))
        {
            gb_run_active = true;
            g_run_mark    = path;
        }
        else
        {
            /* Either already in a black run or < 2 prior white cycles. */
        }

        if (gb_run_active &&
            (odometry_counts_to_mm(path - g_run_mark) >= LINE_FOUND_MAX_MM))
        {
            (void) mission_post_event(APP_EVT_LINE_FOUND, LINE_FOUND_MAX_MM);
            gb_run_active = false;
            gb_lost_sent  = false;
        }
        else
        {
            /* Continue measuring active black run or waiting for white. */
        }
    }
    else if (gb_run_active)
    {
        uint32_t cross_mm = odometry_counts_to_mm(path - g_run_mark);

        if (cross_mm > LINE_FOUND_MAX_MM)
        {
            cross_mm = LINE_FOUND_MAX_MM;
        }
        else
        {
            /* Within 0..255 mm. */
        }
        (void) mission_post_event(APP_EVT_LINE_FOUND, cross_mm);
        gb_run_active = false;
        gb_lost_sent  = false;
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
 * @return  none
 * @context line_task
 */
static void line_step_junction(uint32_t norm_l, uint32_t norm_r,
                               uint32_t path, bool b_inhibit_jct,
                               uint32_t * p_status)
{
    uint32_t const cur_seq = barcode_branch_seq();

    if (cur_seq != g_last_branch_seq)
    {
        g_branch_path     = path;
        gb_branch_seen    = true;
        g_last_branch_seq = cur_seq;
    }
    else
    {
        /* Branch counter unchanged this cycle. */
    }

    if (b_inhibit_jct)
    {
        line_track_jct_reset(&g_jct);
    }
    else
    {
        line_track_jct_in_t jct_in    = {0};
        uint32_t            jct_class = APP_JCT_NONE;

        jct_in.norm_l          = norm_l;
        jct_in.norm_r          = norm_r;
        jct_in.path_counts     = path;
        jct_in.branch_seq      = cur_seq;
        jct_in.min_counts      = g_jct_min_counts;
        jct_in.confirm_counts  = g_jct_conf_counts;
        jct_in.b_branch_recent =
            (gb_branch_seen && ((path - g_branch_path) <= g_branch_early_c));

        jct_class = line_track_jct_step(&g_jct, &jct_in);
        if (APP_JCT_NONE != jct_class)
        {
            (void) mission_post_event(APP_EVT_JUNCTION, jct_class);
            g_last_jct     = jct_class;
            gb_branch_seen = false;
        }
        else
        {
            /* No junction confirmed on this cycle. */
        }

        if (LINE_JCT_STATE_IDLE != g_jct.state)
        {
            *p_status = APP_LINE_JUNCTION;
        }
        else
        {
            /* Keep ON / LOST status. */
        }
    }
}

/*** end of file ***/
