/**
 * @file    imu.c
 * @brief   imu_task (10 ms, R2): accelerometer FIFO + magnetometer on
 *          I2C1, calibration, hump detection, motion classification,
 *          turn rate and the lift-stop (SAFETY_LIFT).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    task
 * @reqs    FR4, FR5, FR13 (lift-stop, Change Requirement 2), FR18, NFR7,
 *          NFR11
 *
 * Change Requirement 2 (lecturer, 11 Nov 2026) removed IMU collision
 * detection: the INT1 impact interrupt, its R1 callback and the IMPACT
 * class are gone. The only replacement is the lift-stop: while the motors
 * are enabled, a car lifted or overturned for IMU_LIFT_CONFIRM_CYCLES
 * cycles sets SAFETY_LIFT, and motion_task brakes at its next cycle. It
 * runs in this task on the calibrated averages already read every cycle,
 * so no ISR shares data with imu_task any more.
 *
 * Cycle order is fixed by B4.2.2: check-in, sensor reads, board read,
 * calibration / run start, forward acceleration, terrain step, turn
 * rate, class, board write. The lift-stop check runs right after the
 * calibration / run-start step.
 *
 * Phase 2 (baseline) representation: plain static variables, all
 * written only by imu_task. Phase 4 packs the statics into the
 * PRV_IMU_* / PRV_HIST_* / PRV_OFS_* layout rows without changing
 * behaviour.
 *
 * WCET budget: 550 us normal, 750 us magnetometer cycle, 800 us backlog
 * (B.3); measured on GP14 in Phase 3 (S4-I09).
 */

#include "imu.h" /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "cfg_imu.h"
#include "terrain.h"
#include "drv_lsm303.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../subsystem2/odometry.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/fault.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/sys_tick.h"
#include "../subsystem5/sys_health.h"

/* ---- private constants and types ------------------------------------ */

#define IMU_STACK_WORDS        (APP_STACK_IMU_BYTES / 8u)
#define IMU_AVG_Q15_ONE        (32768)
#define IMU_MG_NUM             (61) /* 0.122 mg/LSB at +-4 g: 61/500  */
#define IMU_MG_DEN             (500)
#define IMU_MAG_DIV_PERIOD     (IMU_MAG_EVERY_N - 1u)
#define IMU_MAG_BACKLOG_LIMIT  (2u) /* more samples: postpone the mag  */
#define IMU_GOOD_READS_RECOVER (3u)
#define IMU_CLASS_GAP_MAX      (31u)
#define IMU_TURN_WINDOW_CYCLES (5)     /* magnetometer cycle = 50 ms      */
#define IMU_DPS_PER_WINDOW     (2)     /* 0.1 deg / 50 ms -> deg/s        */
#define IMU_OFFSET_LSB_MG      (2)     /* PRV_OFS_* rows: 2 mg per step   */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_imu_stack[IMU_STACK_WORDS];

/* Averaging reciprocals: avg = (sum x recip[n]) / 32768 for n samples. */
static int32_t const g_recip_q15[IMU_ACC_SAMPLES_MAX + 1u] = {0, 32768, 16384,
                                                              10923, 8192};

static terrain_t            g_terrain;
static terrain_speed_hist_t g_speed_hist;

static int16_t g_mag_mx = 0;
static int16_t g_mag_my = 0;
static int16_t g_mag_mz = 0;

/* Packed private state (Phase 4, B4.4). imu_task is the only writer of
   every word: no ISR writes IMU data since the INT1 callback was removed
   (Change Requirement 2). */
static uint32_t g_imu_prv_a; /* I2C_ERR, GOOD_CNT, CLASS_GAP,
                                LAST_CLASS, MAG_DIV, LIFT_CNT, NEED_CFG */
static uint32_t g_imu_prv_b; /* CAL_CNT (HIST_IDX lives in the history
                                word inside terrain.c) */
static uint32_t g_imu_prv_c; /* PREV_HDG, PREV_HDG_VALID, RUN_PEAK */
static uint32_t g_imu_prv_d; /* PREV_DIFF: pos_l - pos_r, 50 ms ago */
static uint32_t g_imu_prv_e; /* PREV_POS_L */
static uint32_t g_imu_prv_f; /* PREV_POS_R */
static uint32_t g_ofs_word_a = (2048u) | (2048u << 12); /* encoded zero
                                       offsets (value + 2048 per
                                       PRV_OFS_A row) */
static uint32_t g_ofs_word_b = (2048u); /* encoded zero AZ offset */

/* E2 calibration sums stay natural (arithmetic headroom, live 2 s). */
static int32_t g_cal_sum_x      = 0;
static int32_t g_cal_sum_y      = 0;
static int32_t g_cal_sum_z      = 0;
static bool    gb_cal_active    = false;
static bool    gb_cal_done_once = false;

/* Per-cycle caches: recomputed from the packed state or the sensors and
   kept so the board write and the next cycle can read them. */
static int16_t  g_avg_raw_x         = 0; /* E4 raw averages, before offsets */
static int16_t  g_avg_raw_y         = 0;
static int16_t  g_avg_raw_z         = 0;
static int32_t  g_avg_ax_mg         = 0; /* calibrated averages */
static int32_t  g_avg_ay_mg         = 0;
static int32_t  g_avg_az_mg         = 1000; /* flat rest: az = +1000 mg */
/* SH_IMU_ACC: the three calibrated averages in 4 mg steps for debug
   telemetry; imu_task stores the whole word at once (single writer). */
static volatile uint32_t g_sh_imu_acc = 0u;
static int32_t  g_a_fwd_mg          = 0;
static int32_t  g_turn_dps          = 0;
static bool     gb_is_turn_from_enc = false;
static int32_t  g_pitch_ddeg        = 0;
static uint32_t g_hump_now_mm       = 0u;
static int32_t  g_enc_hdg_prev_cdeg = 0;

/* ---- private function prototypes ------------------------------------- */

static void imu_task(INT stacd, void * p_exinf);
static void imu_read_sensors(bool * pb_read_ok, bool * pb_mag_cycle,
                             bool * pb_mag_read);
static bool imu_read_acc(uint32_t * p_n_samples);
static void imu_read_mag_if_due(uint32_t n_samples, bool * pb_mag_cycle,
                                bool * pb_mag_read);
static void imu_terrain_update(int32_t pos_l, int32_t pos_r, int32_t speed_cpw);
static int32_t  imu_encoder_rate(int32_t pos_l, int32_t pos_r);
static void     imu_i2c_result(bool b_ok, bool b_is_acc);
static bool     imu_configure_if_needed(void);
static void     imu_need_cfg_set(bool b_need);
static void     imu_acc_publish(void);
static uint32_t imu_acc_code(int32_t mg);
static void     imu_calibrate(bool b_mode_set, bool b_read_ok);
static void     imu_run_start(int32_t pos_l, int32_t pos_r);
static void     imu_turn_rate(bool b_mag_cycle, bool b_mag_read, int32_t pos_l,
                              int32_t pos_r);
static int32_t  imu_mag_rate(bool b_mag_read, bool * pb_use_enc);
static bool     imu_turn_disagrees(int32_t imu_dps, int32_t enc_dps);
static void     imu_lift_check(bool b_has_data);
static void     imu_classify(int32_t speed_cpw);
static void     imu_write_board(uint32_t motion_class);
static uint32_t imu_terrain_state_get(void);
static void     imu_terrain_state_set(uint32_t state);
static void     imu_clamp_fault(bool b_was_clamped);

/* ---- public functions ------------------------------------------------- */

/**
 * imu_init
 * @brief   Initialises the LSM303D driver (I2C1, 400 kHz, all
 *          registers from the B4.1.1 table), up to IMU_INIT_TRIES times
 *          IMU_INIT_RETRY_MS apart: right after switch-on the sensor can
 *          still NACK (on-car boot log 2026-10-29: imu_init -2, then
 *          CAL_TIMEOUT). If every try fails, NEED_CFG stays set and
 *          imu_task keeps trying. imu_task owns no kernel objects:
 *          everything it needs comes from the shared services.
 * @return  0 (E_OK) or the last negative I2C error from the driver
 * @context usermain
 */
ER imu_init(void)
{
    int32_t  ercd  = -1;
    uint32_t tries = 0u;

    for (tries = 0u; (tries < IMU_INIT_TRIES) && (0 != ercd); tries++)
    {
        if (tries > 0u)
        {
            (void)tk_dly_tsk(IMU_INIT_RETRY_MS); /* E_OK or released early */
        }
        ercd = drv_lsm303_init();
    }
    imu_need_cfg_set(0 != ercd);
    if (0 == ercd)
    {
        /* Bring-up diagnosis (S4-U15). cast: 0x1D or 0x1E */
        app_log("IMU answered at I2C address %d (29 = 0x1D SA0 high, 30 ="
                " 0x1E SA0 low)\n", (int32_t) drv_lsm303_addr(), 0, 0);
    }

    return (ER)ercd; /* cast: driver returns 0 or <0, same sign
                        convention as ER */
}

/**
 * imu_start
 * @brief   Starts imu_task on its static stack. No R1 callback: the INT1
 *          impact interrupt was removed (Change Requirement 2).
 * @return  E_OK or the kernel error
 * @context usermain
 */
ER imu_start(void)
{
    ER ercd = E_OK;
    ID tid  = 0;

    tid = app_start_task((FP)imu_task, /* cast: kernel entry type */
                         APP_PRI_IMU, g_imu_stack, APP_STACK_IMU_BYTES,
                         "imu_task");
    if (E_OK > tid)
    {
        ercd = tid; /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * imu_acc_word
 * @brief   Last calibrated accelerometer averages (SH_IMU_ACC layout) for
 *          debug telemetry.
 * @return  SH_IMU_ACC word: AX, AY, AZ, 10 bits each, 4 mg steps, offset
 *          512
 * @context any task (one aligned load of a word written by imu_task)
 */
uint32_t imu_acc_word(void)
{
    return g_sh_imu_acc;
}

/* ---- private functions ------------------------------------------------ */

/**
 * imu_task
 * @brief   10 ms cyclic IMU and terrain task. See the file header for
 *          the fixed cycle order (B4.2.2).
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context imu_task
 */
static void imu_task(INT stacd, void * p_exinf)
{
    (void)stacd;   /* unused: kernel start code */
    (void)p_exinf; /* unused: no extended info  */

    terrain_reset(&g_terrain);
    terrain_speed_hist_reset(&g_speed_hist);
    (void)sys_tick_register(SYS_TICK_SLOT_IMU, tk_get_tid());

    for (;;)
    {
        bool           b_read_ok        = false;
        bool           b_mag_cycle      = false;
        bool           b_mag_read       = false;
        bool           b_mode_cal       = false;
        bool           b_mode_run_start = false;
        status_board_t snap;
        int32_t        pos_l     = 0;
        int32_t        pos_r     = 0;
        int32_t        speed_cpw = 0;

        (void)tk_slp_tsk(TMO_FEVR); /* released by R2 */
        sys_tick_cycle_start(SYS_TICK_SLOT_IMU);
        sys_health_check_in(SYS_HEALTH_BIT_IMU);

        /* 2. Sensors (or, after a failed transfer, rewrite the LSM303D
           registers first: a supply dip resets the sensor to power-down). */
        if (imu_configure_if_needed())
        {
            imu_read_sensors(&b_read_ok, &b_mag_cycle, &b_mag_read);
        }

        /* 3. Board snapshot. */
        status_board_read(&snap);
        pos_l = status_board_pos_l(&snap);
        pos_r = status_board_pos_r(&snap);
        speed_cpw =
            (status_board_speed_l(&snap) + status_board_speed_r(&snap)) / 2;

        /* 4. Calibration and run start. */
        b_mode_cal       = mission_mode_is_set(APP_MODE_CAL_IMU);
        b_mode_run_start = mission_mode_is_set(APP_MODE_RUN_START);

        imu_calibrate(b_mode_cal, b_read_ok);
        if (b_mode_run_start)
        {
            imu_run_start(pos_l, pos_r);
        }

        /* 4a. Lift-stop (Change Requirement 2). */
        imu_lift_check(b_read_ok && !b_mode_cal);

        if (TERRAIN_FAULT == imu_terrain_state_get())
        {
            /* FAULT: only the I2C retry path runs (step 2). The board
               keeps the last good values. */
        }
        else if (b_mode_cal || b_mode_run_start)
        {
            /* CAL and RUN_START: the robot is braked and the terrain
               state machine is not advanced. */
            imu_write_board(APP_MCLASS_STATIONARY);
        }
        else
        {
            imu_terrain_update(pos_l, pos_r, speed_cpw);

            /* 7. Turn rate, every 5th cycle (the magnetometer cycle). */
            imu_turn_rate(b_mag_cycle, b_mag_read, pos_l, pos_r);

            /* 8. Class. */
            imu_classify(speed_cpw);

            /* 9. Board. */
            imu_write_board(app_bits_get(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                                         PRV_IMU_A_LAST_CLASS_BITS));
        }
    }
}

/**
 * imu_terrain_update
 * @brief   B4.2.2 steps 5 and 6: forward acceleration over the full
 *          50 ms window, then one Terrain Monitor cycle. ds comes from
 *          the slower wheel; a rebase or a glitch gives 0 so the
 *          run-start rebase can never become a false climb (review R03).
 *          A finished hump updates the run peak and posts APP_EVT_HUMP.
 * @param   pos_l      current left position, counts
 * @param   pos_r      current right position, counts
 * @param   speed_cpw  this cycle's speed, counts per 50 ms window
 * @return  none
 * @context imu_task
 */
static void imu_terrain_update(int32_t pos_l, int32_t pos_r, int32_t speed_cpw)
{
    terrain_input_t  in;
    terrain_output_t out;
    int32_t          prev_l =
        app_bits_decode(app_bits_get(g_imu_prv_e, PRV_IMU_E_PREV_POS_L_POS,
                                     PRV_IMU_E_PREV_POS_L_BITS),
                        PRV_IMU_E_PREV_POS_L_OFFSET);
    int32_t prev_r =
        app_bits_decode(app_bits_get(g_imu_prv_f, PRV_IMU_F_PREV_POS_R_POS,
                                     PRV_IMU_F_PREV_POS_R_BITS),
                        PRV_IMU_F_PREV_POS_R_OFFSET);
    int32_t dpos_l    = pos_l - prev_l;
    int32_t dpos_r    = pos_r - prev_r;
    bool    b_clamped = false;

    /* 5. Forward acceleration: the difference across the full 50 ms
       window, never 10 ms. */
    g_a_fwd_mg = terrain_speed_hist_push(&g_speed_hist, speed_cpw);

    g_imu_prv_e = app_bits_put(
        g_imu_prv_e, PRV_IMU_E_PREV_POS_L_POS, PRV_IMU_E_PREV_POS_L_BITS,
        app_bits_encode(pos_l, PRV_IMU_E_PREV_POS_L_MIN,
                        PRV_IMU_E_PREV_POS_L_MAX, PRV_IMU_E_PREV_POS_L_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);
    g_imu_prv_f = app_bits_put(
        g_imu_prv_f, PRV_IMU_F_PREV_POS_R_POS, PRV_IMU_F_PREV_POS_R_BITS,
        app_bits_encode(pos_r, PRV_IMU_F_PREV_POS_R_MIN,
                        PRV_IMU_F_PREV_POS_R_MAX, PRV_IMU_F_PREV_POS_R_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);

    in.ax_mg     = g_avg_ax_mg;
    in.az_mg     = g_avg_az_mg;
    in.a_fwd_mg  = g_a_fwd_mg;
    in.ds_counts = terrain_ds_counts(dpos_l, dpos_r);

    terrain_step(&g_terrain, &in, &out);

    g_pitch_ddeg  = out.pitch_ddeg;
    g_hump_now_mm = out.hump_now_mm;

    if (out.hump_done_mm > 0u)
    {
        uint32_t run_peak = app_bits_get(g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS,
                                         PRV_IMU_C_RUN_PEAK_BITS);

        if (out.hump_done_mm > run_peak)
        {
            g_imu_prv_c = app_bits_put(
                g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS, PRV_IMU_C_RUN_PEAK_BITS,
                app_bits_encode((int32_t)out.hump_done_mm,
                                PRV_IMU_C_RUN_PEAK_MIN, PRV_IMU_C_RUN_PEAK_MAX,
                                PRV_IMU_C_RUN_PEAK_OFFSET, &b_clamped));
            /* cast: hump_done_mm < 2^31 */
            imu_clamp_fault(b_clamped);
        }
        (void)mission_post_event(APP_EVT_HUMP, out.hump_done_mm);
        /* ignored: K4 drops are counted by
           mission_post_event itself, and its VALUE clamp raises
           APP_FAULT_RANGE_CLAMP for a peak beyond the 7-bit event
           field (A25 / correction T6) */
#if (IMU_LOG_HUMPS != 0)
        app_log("hump int %d geo %d\n", (int32_t)out.hump_done_mm,
                (int32_t)out.hump_geo_mm, 0);
#endif
    }
}

/**
 * imu_read_sensors
 * @brief   One accelerometer FIFO read (<= 4 samples, averaged with the
 *          reciprocal table) and, every IMU_MAG_EVERY_N-th cycle, one
 *          magnetometer read. The magnetometer is postponed when the
 *          cycle already read more than 2 samples (B4.1.1 timing).
 * @param   pb_read_ok    out: true when the accelerometer read succeeded
 *                        and returned at least one new sample (an empty
 *                        FIFO is not an I2C error, but gives no new data)
 * @param   pb_mag_cycle  out: true when this is a magnetometer cycle,
 *                        even when the read was postponed or failed
 * @param   pb_mag_read   out: true when the magnetometer was read
 * @return  none
 * @context imu_task
 */
static void imu_read_sensors(bool * pb_read_ok, bool * pb_mag_cycle,
                             bool * pb_mag_read)
{
    uint32_t n_samples = 0u;

    bool b_acc_ok = imu_read_acc(&n_samples);


    /* Calibration may only sum a cycle that brought new samples: on an
       empty FIFO g_avg_raw_* still holds last cycle's average, which
       would be counted twice. */
    *pb_read_ok = b_acc_ok && (n_samples > 0u);
    imu_read_mag_if_due(n_samples, pb_mag_cycle, pb_mag_read);
}

/**
 * imu_read_acc
 * @brief   One accelerometer FIFO read (0..4 samples) averaged with the
 *          reciprocal table and converted to mg with the calibration
 *          offsets applied. An empty FIFO or an overrun keeps the last
 *          cycle's averages.
 * @param   p_n_samples  out: samples taken this cycle (0 on overrun)
 * @return  true when the transfer succeeded
 * @context imu_task
 */
static bool imu_read_acc(uint32_t * p_n_samples)
{
    drv_lsm303_acc_t samples[IMU_ACC_SAMPLES_MAX];
    int32_t          ercd      = 0;
    uint32_t         n_samples = 0u;

    *p_n_samples = 0u;

    ercd = drv_lsm303_read_acc(samples, IMU_ACC_SAMPLES_MAX);
    if (ercd < 0)
    {
        imu_i2c_result(false, true);
        return false;
    }

    {
        uint32_t idx      = 0u;
        int32_t  sum_x    = 0;
        int32_t  sum_y    = 0;
        int32_t  sum_z    = 0;
        uint32_t fifo_ovr = drv_lsm303_fifo_overruns();
        int32_t  ofs_ax =
            app_bits_decode(
                app_bits_get(g_ofs_word_a, PRV_OFS_A_AX_POS, PRV_OFS_A_AX_BITS),
                PRV_OFS_A_AX_OFFSET) *
            IMU_OFFSET_LSB_MG;
        int32_t ofs_ay =
            app_bits_decode(
                app_bits_get(g_ofs_word_a, PRV_OFS_A_AY_POS, PRV_OFS_A_AY_BITS),
                PRV_OFS_A_AY_OFFSET) *
            IMU_OFFSET_LSB_MG;
        int32_t ofs_az =
            app_bits_decode(
                app_bits_get(g_ofs_word_b, PRV_OFS_B_AZ_POS, PRV_OFS_B_AZ_BITS),
                PRV_OFS_B_AZ_OFFSET) *
            IMU_OFFSET_LSB_MG;
        /* LSB weight from the layout (B4.4): one stored step is 2 mg. */

        /* PRV_OFS_B FIFO_OVR: the driver's counter, packed here. */
        g_ofs_word_b = app_bits_put(g_ofs_word_b, PRV_OFS_B_FIFO_OVR_POS,
                                    PRV_OFS_B_FIFO_OVR_BITS, fifo_ovr);

        imu_i2c_result(true, true);
        n_samples = (uint32_t)ercd; /* cast: count >= 0 here */
        if (n_samples > 0u)
        {
            for (idx = 0u; idx < n_samples; idx++)
            {
                sum_x += (int32_t)samples[idx].acc_x;
                sum_y += (int32_t)samples[idx].acc_y;
                sum_z += (int32_t)samples[idx].acc_z;
            }
            /* Signed average: (sum x recip[n]) / 32768, never >> on a
               signed value (review F-07). |sum| <= 4 x 32768, so the
               product fits int32. The LSM303D delivers a plain 16-bit
               sample, so the average fits an int16 (E4 image). */
            g_avg_raw_x =
                (int16_t)((sum_x * g_recip_q15[n_samples]) / IMU_AVG_Q15_ONE);
            g_avg_raw_y =
                (int16_t)((sum_y * g_recip_q15[n_samples]) / IMU_AVG_Q15_ONE);
            g_avg_raw_z =
                (int16_t)((sum_z * g_recip_q15[n_samples]) / IMU_AVG_Q15_ONE);
            g_avg_ax_mg =
                (((int32_t)g_avg_raw_x * IMU_MG_NUM) / IMU_MG_DEN) - ofs_ax;
            g_avg_ax_mg = g_avg_ax_mg * IMU_X_FORWARD_SIGN;
            g_avg_ay_mg =
                (((int32_t)g_avg_raw_y * IMU_MG_NUM) / IMU_MG_DEN) - ofs_ay;
            g_avg_az_mg =
                (((int32_t)g_avg_raw_z * IMU_MG_NUM) / IMU_MG_DEN) - ofs_az;
            imu_acc_publish();
        }
    }

    *p_n_samples = n_samples;
    return true;
}

/**
 * imu_read_mag_if_due
 * @brief   Magnetometer read on every IMU_MAG_EVERY_N-th cycle. It is
 *          postponed one cycle when this cycle already took more than
 *          IMU_MAG_BACKLOG_LIMIT samples (B4.1.1 bus budget).
 * @param   n_samples    accelerometer samples taken this cycle
 * @param   pb_mag_cycle out: true when this cycle was due a mag read
 * @param   pb_mag_read  out: true when the mag was actually read
 * @return  none
 * @context imu_task
 */
static void imu_read_mag_if_due(uint32_t n_samples, bool * pb_mag_cycle,
                                bool * pb_mag_read)
{
    uint32_t mag_div   = app_bits_get(g_imu_prv_a, PRV_IMU_A_MAG_DIV_POS,
                                      PRV_IMU_A_MAG_DIV_BITS);
    bool     b_mag_due = (0u == mag_div);
    int32_t  ercd      = 0;

    *pb_mag_cycle = b_mag_due;
    *pb_mag_read  = false;

    if (!b_mag_due)
    {
        mag_div--;
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_MAG_DIV_POS,
                                   PRV_IMU_A_MAG_DIV_BITS, mag_div);
        return;
    }
    if (n_samples > IMU_MAG_BACKLOG_LIMIT)
    {
        return; /* backlog cycle: postponed, MAG_DIV stays 0 */
    }

    ercd = drv_lsm303_read_mag(&g_mag_mx, &g_mag_my, &g_mag_mz);
    if (ercd < 0)
    {
        imu_i2c_result(false, false);
    }
    else
    {
        imu_i2c_result(true, false);
        *pb_mag_read = true;
    }
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_MAG_DIV_POS,
                               PRV_IMU_A_MAG_DIV_BITS, IMU_MAG_DIV_PERIOD);
}

/**
 * imu_i2c_result
 * @brief   Tracks consecutive I2C errors. At IMU_I2C_ERR_MAX in a row
 *          the monitor enters TERRAIN_FAULT, APP_FAULT_IMU is set and
 *          the bus is reset. In FAULT, IMU_GOOD_READS_RECOVER good
 *          accelerometer reads return to FLAT (the fault bit stays set
 *          until mission's RESET).
 * @param   b_ok      true for a successful transaction
 * @param   b_is_acc  true when the transaction was the accelerometer
 *                    read (the one that counts for recovery)
 * @return  none
 * @context imu_task
 */
static void imu_i2c_result(bool b_ok, bool b_is_acc)
{
    uint32_t err_cnt  = app_bits_get(g_imu_prv_a, PRV_IMU_A_I2C_ERR_POS,
                                     PRV_IMU_A_I2C_ERR_BITS);
    uint32_t good_cnt = app_bits_get(g_imu_prv_a, PRV_IMU_A_GOOD_CNT_POS,
                                     PRV_IMU_A_GOOD_CNT_BITS);

    if (b_ok)
    {
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_I2C_ERR_POS,
                                   PRV_IMU_A_I2C_ERR_BITS, 0u);
        if (b_is_acc && (TERRAIN_FAULT == imu_terrain_state_get()))
        {
            good_cnt++;
            if (good_cnt >= IMU_GOOD_READS_RECOVER)
            {
                imu_terrain_state_set(TERRAIN_FLAT);
                terrain_reset(&g_terrain);
                good_cnt = 0u;
            }
            g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_GOOD_CNT_POS,
                                       PRV_IMU_A_GOOD_CNT_BITS, good_cnt);
        }
        return;
    }

    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_GOOD_CNT_POS,
                               PRV_IMU_A_GOOD_CNT_BITS, 0u);
    imu_need_cfg_set(true); /* the sensor may have lost its registers */
    if (err_cnt < IMU_I2C_ERR_MAX)
    {
        err_cnt++;
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_I2C_ERR_POS,
                                   PRV_IMU_A_I2C_ERR_BITS, err_cnt);
    }
    /* Not before start-up has finished (radio settled, BOOT ends): the
       radio's power-up can make the LSM303D NACK for a while (on-car
       boot log 2026-10-29); the registers are rewritten as soon as it
       answers. A sensor still failing afterwards faults on its next
       error, err_cnt having stayed at the maximum. */
    if ((err_cnt >= IMU_I2C_ERR_MAX) && sys_health_is_started() &&
        (TERRAIN_FAULT != imu_terrain_state_get()))
    {
        imu_terrain_state_set(TERRAIN_FAULT);
        /* Bring-up diagnosis: -1 = the bus stayed silent for a whole
           1 ms step (wiring, power, a hung LSM303D), -2 = NACK (sensor
           not answering, e.g. still booting after a supply dip). */
        app_log("IMU I2C: last error %d (-1 timeout, -2 NACK)\n",
                drv_lsm303_last_error(), 0, 0);
        fault_set(APP_FAULT_IMU);
        drv_lsm303_bus_reset();
    }
}

/**
 * imu_configure_if_needed
 * @brief   While NEED_CFG is set, rewrites the LSM303D registers instead
 *          of reading it this cycle (about 12 short writes, under 1 ms).
 *          A failed rewrite counts as an I2C error like any other.
 * @return  true if the sensor is configured and may be read this cycle
 * @context imu_task
 */
static bool imu_configure_if_needed(void)
{
    if (0u == app_bits_get(g_imu_prv_a, PRV_IMU_A_NEED_CFG_POS,
                           PRV_IMU_A_NEED_CFG_BITS))
    {
        return true;
    }
    if (0 != drv_lsm303_init())
    {
        imu_i2c_result(false, false);
        return false;
    }
    imu_need_cfg_set(false);
    /* Bring-up diagnosis */
    app_log("IMU registers written again (sensor answered at address %d)\n",
            (int32_t) drv_lsm303_addr(), 0, 0); /* cast: 0x1D or 0x1E */
    return false; /* read from the next cycle: the FIFO restarts empty */
}

/**
 * imu_need_cfg_set
 * @brief   Sets or clears the packed NEED_CFG flag.
 * @param   b_need  true if the registers must be (re)written
 * @return  none
 * @context usermain (imu_init) or imu_task
 */
static void imu_need_cfg_set(bool b_need)
{
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_NEED_CFG_POS,
                               PRV_IMU_A_NEED_CFG_BITS, b_need ? 1u : 0u);
}


/**
 * imu_calibrate
 * @brief   APP_MODE_CAL_IMU: sums IMU_CAL_SAMPLES raw samples (robot
 *          braked), then sets the offsets so that at rest ax = ay = 0
 *          and az = +1000 mg. CAL_DONE is posted exactly once per mode
 *          activation (B4.2.2 step 4). The magnetometer is NOT
 *          calibrated at run time (review R15).
 * @param   b_mode_set  current state of the mode bit
 * @param   b_read_ok   true when this cycle's read succeeded with new
 *                      samples (no
 *                      summation on I2C failures, so a dead bus ends
 *                      in mission's calibration timeout, not in
 *                      garbage offsets)
 * @return  none
 * @context imu_task
 */
static void imu_calibrate(bool b_mode_set, bool b_read_ok)
{
    uint32_t cal_cnt = app_bits_get(g_imu_prv_b, PRV_IMU_B_CAL_CNT_POS,
                                    PRV_IMU_B_CAL_CNT_BITS);

    if (b_mode_set && !gb_cal_active)
    {
        gb_cal_active    = true;
        gb_cal_done_once = false;
        cal_cnt          = 0u; /* the local copy too: on a second
                                  calibration it still held 200 and the
                                  offsets were computed from empty sums */
        g_imu_prv_b      = app_bits_put(g_imu_prv_b, PRV_IMU_B_CAL_CNT_POS,
                                        PRV_IMU_B_CAL_CNT_BITS, 0u);
        g_cal_sum_x      = 0;
        g_cal_sum_y      = 0;
        g_cal_sum_z      = 0;
        imu_terrain_state_set(TERRAIN_CAL);
    }
    else if (!b_mode_set)
    {
        gb_cal_active = false;
        return;
    }

    if (!b_read_ok)
    {
        return; /* no samples this cycle */
    }
    if (cal_cnt < IMU_CAL_SAMPLES)
    {
        g_cal_sum_x += g_avg_raw_x; /* raw, before the old offsets */
        g_cal_sum_y += g_avg_raw_y;
        g_cal_sum_z += g_avg_raw_z;
        cal_cnt++;
        g_imu_prv_b = app_bits_put(g_imu_prv_b, PRV_IMU_B_CAL_CNT_POS,
                                   PRV_IMU_B_CAL_CNT_BITS, cal_cnt);
        return;
    }

    if (!gb_cal_done_once)
    {
        bool    b_clamped = false;
        int32_t avg_raw_x = 0;
        int32_t avg_raw_y = 0;
        int32_t avg_raw_z = 0;
        int32_t ofs_ax    = 0;
        int32_t ofs_ay    = 0;
        int32_t ofs_az    = 0;

        /* Event-time division by the sample count (once per mode). The
           sums are in raw register units, so each average is converted
           to mg exactly like imu_read_acc does before it can be
           subtracted there: mg = raw x 61 / 500 (0.122 mg/LSB). */
        avg_raw_x = g_cal_sum_x / (int32_t)IMU_CAL_SAMPLES;
        avg_raw_y = g_cal_sum_y / (int32_t)IMU_CAL_SAMPLES;
        avg_raw_z = g_cal_sum_z / (int32_t)IMU_CAL_SAMPLES;
        ofs_ax    = ((avg_raw_x * IMU_MG_NUM) / IMU_MG_DEN);
        ofs_ay    = ((avg_raw_y * IMU_MG_NUM) / IMU_MG_DEN);
        ofs_az    = ((avg_raw_z * IMU_MG_NUM) / IMU_MG_DEN) - IMU_REST_AZ_MG;

        /* Offsets are stored in 2 mg steps, so the conversion divides by
           IMU_OFFSET_LSB_MG (a power of two; never >> on a signed
           value). */
        g_ofs_word_a = app_bits_put(
            g_ofs_word_a, PRV_OFS_A_AX_POS, PRV_OFS_A_AX_BITS,
            app_bits_encode(ofs_ax / IMU_OFFSET_LSB_MG, PRV_OFS_A_AX_MIN,
                            PRV_OFS_A_AX_MAX, PRV_OFS_A_AX_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);
        g_ofs_word_a = app_bits_put(
            g_ofs_word_a, PRV_OFS_A_AY_POS, PRV_OFS_A_AY_BITS,
            app_bits_encode(ofs_ay / IMU_OFFSET_LSB_MG, PRV_OFS_A_AY_MIN,
                            PRV_OFS_A_AY_MAX, PRV_OFS_A_AY_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);
        g_ofs_word_b = app_bits_put(
            g_ofs_word_b, PRV_OFS_B_AZ_POS, PRV_OFS_B_AZ_BITS,
            app_bits_encode(ofs_az / IMU_OFFSET_LSB_MG, PRV_OFS_B_AZ_MIN,
                            PRV_OFS_B_AZ_MAX, PRV_OFS_B_AZ_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);

        /* Bring-up diagnosis: gravity at rest on each axis. A tilt here
           moves az away from +1000 mg before the offsets are applied. */
        app_log("IMU at rest, raw before offsets: ax %d ay %d az %d mg\n",
                ofs_ax, ofs_ay,
                ofs_az + IMU_REST_AZ_MG);
        gb_cal_done_once = true;
        imu_terrain_state_set(TERRAIN_FLAT);
        terrain_reset(&g_terrain);
        (void)mission_post_event(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU);
        /* ignored: K4 drops are counted by
           mission_post_event itself */
    }
}

/**
 * imu_run_start
 * @brief   APP_MODE_RUN_START: per-run state reset (B4.2.2 step 4).
 *          Idempotent, so it may run on every cycle while the bit is
 *          set. The previous positions start at the current ones and
 *          terrain_ds_counts absorbs the rebase.
 * @param   pos_l  current left position
 * @param   pos_r  current right position
 * @return  none
 * @context imu_task
 */
static void imu_run_start(int32_t pos_l, int32_t pos_r)
{
    bool b_clamped = false;

    terrain_reset(&g_terrain);
    terrain_speed_hist_reset(&g_speed_hist);
    g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS,
                               PRV_IMU_C_RUN_PEAK_BITS, 0u);
    g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_POS,
                               PRV_IMU_C_PREV_HDG_BITS, 0u);
    g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                               PRV_IMU_C_PREV_HDG_VALID_BITS, 0u);
    g_imu_prv_d = app_bits_put(
        g_imu_prv_d, PRV_IMU_D_PREV_DIFF_POS, PRV_IMU_D_PREV_DIFF_BITS,
        app_bits_encode(pos_l - pos_r, PRV_IMU_D_PREV_DIFF_MIN,
                        PRV_IMU_D_PREV_DIFF_MAX, PRV_IMU_D_PREV_DIFF_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);
    g_imu_prv_e = app_bits_put(
        g_imu_prv_e, PRV_IMU_E_PREV_POS_L_POS, PRV_IMU_E_PREV_POS_L_BITS,
        app_bits_encode(pos_l, PRV_IMU_E_PREV_POS_L_MIN,
                        PRV_IMU_E_PREV_POS_L_MAX, PRV_IMU_E_PREV_POS_L_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);
    g_imu_prv_f = app_bits_put(
        g_imu_prv_f, PRV_IMU_F_PREV_POS_R_POS, PRV_IMU_F_PREV_POS_R_BITS,
        app_bits_encode(pos_r, PRV_IMU_F_PREV_POS_R_MIN,
                        PRV_IMU_F_PREV_POS_R_MAX, PRV_IMU_F_PREV_POS_R_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);

    g_a_fwd_mg          = 0;
    g_enc_hdg_prev_cdeg = 0;
    g_turn_dps          = 0;
    gb_is_turn_from_enc = false;
    g_pitch_ddeg        = 0;
    g_hump_now_mm       = 0u;
    g_imu_prv_a         = app_bits_put(g_imu_prv_a, PRV_IMU_A_LIFT_CNT_POS,
                                       PRV_IMU_A_LIFT_CNT_BITS, 0u);
    g_imu_prv_a =
        app_bits_put(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                     PRV_IMU_A_LAST_CLASS_BITS, APP_MCLASS_STATIONARY);
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_CLASS_GAP_POS,
                               PRV_IMU_A_CLASS_GAP_BITS, 0u);
}

/**
 * imu_turn_rate
 * @brief   Turn rate every 5th cycle. Flat (|pitch| <=
 *          IMU_MAG_MAX_PITCH_DDEG): magnetometer heading difference
 *          over 50 ms, x 2 = deg/s. On a slope, on a postponed cycle or
 *          after an I2C error the encoder rate is used and
 *          PREV_HDG_VALID is cleared, so a 100 ms gap is never treated
 *          as 50 ms (B4.2.2 step 7). A disagreement beyond
 *          IMU_TURN_DISAGREE_PCT (A7) reports the encoder rate with the
 *          source flag.
 * @param   b_mag_cycle  true on the every-5th cycle
 * @param   b_mag_read   true when the magnetometer was actually read
 * @param   pos_l        current left position
 * @param   pos_r        current right position
 * @return  none
 * @context imu_task
 */
static void imu_turn_rate(bool b_mag_cycle, bool b_mag_read, int32_t pos_l,
                          int32_t pos_r)
{
    int32_t enc_dps   = 0;
    int32_t imu_dps   = 0;
    bool    b_use_enc = false;

    if (!b_mag_cycle)
    {
        return; /* keep the last reported rate */
    }

    enc_dps = imu_encoder_rate(pos_l, pos_r);
    imu_dps = imu_mag_rate(b_mag_read, &b_use_enc);

    if ((!b_use_enc) && imu_turn_disagrees(imu_dps, enc_dps))
    {
        b_use_enc = true; /* A7: the encoder rate wins */
    }

    g_turn_dps          = (b_use_enc) ? enc_dps : imu_dps;
    gb_is_turn_from_enc = b_use_enc;
}

/**
 * imu_encoder_rate
 * @brief   Encoder turn rate over the same 50 ms window, in deg/s.
 *          Event-time division by IMU_TURN_WINDOW_CYCLES (only every
 *          5th cycle). Also refreshes PRV_IMU_D PREV_DIFF.
 * @param   pos_l  current left position, counts
 * @param   pos_r  current right position, counts
 * @return  deg/s
 * @context imu_task
 */
static int32_t imu_encoder_rate(int32_t pos_l, int32_t pos_r)
{
    int32_t enc_hdg = odometry_heading_cdeg(pos_l, pos_r);
    int32_t enc_dps = (enc_hdg - g_enc_hdg_prev_cdeg) / IMU_TURN_WINDOW_CYCLES;
    bool    b_clamped = false;

    g_enc_hdg_prev_cdeg = enc_hdg;

    /* PREV_DIFF: the position difference that will be 50 ms old on the
       next magnetometer cycle. */
    g_imu_prv_d = app_bits_put(
        g_imu_prv_d, PRV_IMU_D_PREV_DIFF_POS, PRV_IMU_D_PREV_DIFF_BITS,
        app_bits_encode(pos_l - pos_r, PRV_IMU_D_PREV_DIFF_MIN,
                        PRV_IMU_D_PREV_DIFF_MAX, PRV_IMU_D_PREV_DIFF_OFFSET,
                        &b_clamped));
    imu_clamp_fault(b_clamped);

    return enc_dps;
}

/**
 * imu_mag_rate
 * @brief   Magnetometer turn rate: the heading difference over 50 ms,
 *          doubled to deg/s. On a hump, after a postponed cycle or
 *          after an I2C error the reference is dropped, the encoder rate
 *          is requested and a 100 ms gap is never treated as 50 ms
 *          (B4.2.2 step 7).
 * @param   b_mag_read  true when the magnetometer was actually read
 * @param   pb_use_enc  out: true when the encoder rate must be used
 * @return  deg/s from the magnetometer (0 when re-seeding)
 * @context imu_task
 */
static int32_t imu_mag_rate(bool b_mag_read, bool * pb_use_enc)
{
    int32_t imu_dps = 0;

    *pb_use_enc = false;

    if (!b_mag_read)
    {
        /* This cycle should have read the magnetometer but did not
           (postponed backlog or I2C error): drop the reference. */
        g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                   PRV_IMU_C_PREV_HDG_VALID_BITS, 0u);
        *pb_use_enc = true;
        return 0;
    }

    if (terrain_abs(g_pitch_ddeg) > (uint32_t)IMU_MAG_MAX_PITCH_DDEG)
    /* cast: positive constant, comparison only */
    {
        /* On a hump there is no tilt compensation: use the encoder rate
           and drop the magnetometer reference. */
        g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                   PRV_IMU_C_PREV_HDG_VALID_BITS, 0u);
        *pb_use_enc = true;
        return 0;
    }

    {
        int32_t  heading      = terrain_atan2_ddeg(g_mag_my - IMU_MAG_OFS_Y,
                                                   g_mag_mx - IMU_MAG_OFS_X) *
                                IMU_MAG_HEADING_SIGN;
        uint32_t heading_wrap = (uint32_t)terrain_heading_wrap_ddeg(heading);
        /* cast: wrap >= 0 */
        bool b_hdg_valid =
            (0u != app_bits_get(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                PRV_IMU_C_PREV_HDG_VALID_BITS));

        if (b_hdg_valid)
        {
            uint32_t prev_hdg = app_bits_get(
                g_imu_prv_c, PRV_IMU_C_PREV_HDG_POS, PRV_IMU_C_PREV_HDG_BITS);

            imu_dps = terrain_heading_delta_ddeg(prev_hdg, heading_wrap) *
                      IMU_DPS_PER_WINDOW;
        }
        else
        {
            /* Re-seed after a gap: report the encoder rate for this
               cycle. */
            *pb_use_enc = true;
        }
        g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_POS,
                                   PRV_IMU_C_PREV_HDG_BITS, heading_wrap);
        g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                   PRV_IMU_C_PREV_HDG_VALID_BITS, 1u);
    }

    return imu_dps;
}

/**
 * imu_turn_disagrees
 * @brief   Assumption A7 test: a meaningful encoder rate that the
 *          magnetometer contradicts by more than IMU_TURN_DISAGREE_PCT.
 * @param   imu_dps  magnetometer rate, deg/s
 * @param   enc_dps  encoder rate, deg/s
 * @return  true when the encoder rate should be reported
 * @context imu_task
 */
static bool imu_turn_disagrees(int32_t imu_dps, int32_t enc_dps)
{
    int32_t diff = imu_dps - enc_dps;

    if (diff < 0)
    {
        diff = 0 - diff;
    }
    if (terrain_abs(enc_dps) < (uint32_t)IMU_TURN_CHECK_MIN_DPS)
    /* cast: positive constant, comparison only */
    {
        return false; /* too slow to judge */
    }
    return ((diff * 100) >
            ((int32_t)IMU_TURN_DISAGREE_PCT * (int32_t)terrain_abs(enc_dps)));
    /* casts: both products are far below 2^31 */
}

/**
 * imu_lift_check
 * @brief   Lift-stop (Change Requirement 2). Armed only while
 *          APP_MODE_MOTOR_EN is set (the CAL_IR sweep and RUN): handling
 *          or carrying the car with the motors off never latches a stop.
 *          On a cycle with new samples, terrain_lift_step() counts the
 *          cycles in a row the car is lifted or overturned; at
 *          IMU_LIFT_CONFIRM_CYCLES SAFETY_LIFT is set and motion_task
 *          brakes at its next cycle. Only mission_task clears the bit.
 * @param   b_has_data  true when this cycle brought new calibrated
 *                      samples (and the IMU is not calibrating)
 * @return  none
 * @context imu_task
 */
static void imu_lift_check(bool b_has_data)
{
    uint32_t lift_cnt     = app_bits_get(g_imu_prv_a, PRV_IMU_A_LIFT_CNT_POS,
                                         PRV_IMU_A_LIFT_CNT_BITS);
    bool     b_is_lifted  = false;

    if (!mission_mode_is_set(APP_MODE_MOTOR_EN)
        || (TERRAIN_FAULT == imu_terrain_state_get()))
    {
        lift_cnt = 0u; /* disarmed: start again from zero */
    }
    else if (b_has_data)
    {
        lift_cnt = terrain_lift_step(lift_cnt, g_avg_ax_mg, g_avg_ay_mg,
                                     g_avg_az_mg, &b_is_lifted);
        if (b_is_lifted && (0u == (safety_get() & SAFETY_LIFT)))
        {
            safety_set(SAFETY_LIFT); /* brake first, then report */
            app_log("LIFT STOP: robot lifted or overturned (az %d mg)\n",
                    g_avg_az_mg, 0, 0);
        }
    }
    else
    {
        /* no new samples: keep the count */
    }
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_LIFT_CNT_POS,
                               PRV_IMU_A_LIFT_CNT_BITS, lift_cnt);
}

/**
 * imu_classify
 * @brief   Classifier, then posts APP_EVT_MOTION_CLASS when the class
 *          changed and at least IMU_CLASS_MIN_GAP_TICKS passed since the
 *          last post (D.13, B4.2.2 step 8). LIFTED is shown while
 *          SAFETY_LIFT is set.
 * @param   speed_cpw  this cycle's speed
 * @return  none
 * @context imu_task
 */
static void imu_classify(int32_t speed_cpw)
{
    uint32_t cls        = 0u;
    uint32_t last_class = app_bits_get(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                                       PRV_IMU_A_LAST_CLASS_BITS);
    uint32_t class_gap  = app_bits_get(g_imu_prv_a, PRV_IMU_A_CLASS_GAP_POS,
                                       PRV_IMU_A_CLASS_GAP_BITS);

    cls = terrain_classify(imu_terrain_state_get(),
                           (0u != (safety_get() & SAFETY_LIFT)), g_turn_dps,
                           g_a_fwd_mg, speed_cpw);

    if (cls != last_class)
    {
        if (class_gap >= IMU_CLASS_MIN_GAP_TICKS)
        {
            (void)mission_post_event(APP_EVT_MOTION_CLASS, cls);
            /* ignored: K4 drops are counted by
               mission_post_event itself */
            last_class = cls;
            class_gap  = 0u;
        }
        else if (class_gap < IMU_CLASS_GAP_MAX)
        {
            class_gap++;
        }
    }
    else if (class_gap < IMU_CLASS_GAP_MAX)
    {
        class_gap++;
    }
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                               PRV_IMU_A_LAST_CLASS_BITS, last_class);
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_CLASS_GAP_POS,
                               PRV_IMU_A_CLASS_GAP_BITS, class_gap);
}

/**
 * imu_write_board
 * @brief   Writes the imu part of the status board (step 9).
 * @param   motion_class  APP_MCLASS_*
 * @return  none
 * @context imu_task
 */
static void imu_write_board(uint32_t motion_class)
{
    status_board_imu_t part;

    part.pitch_ddeg         = g_pitch_ddeg;
    part.hump_now_mm        = g_hump_now_mm;
    part.hump_peak_mm       = app_bits_get(g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS,
                                           PRV_IMU_C_RUN_PEAK_BITS);
    part.motion_class       = motion_class;
    part.b_is_turn_from_enc = gb_is_turn_from_enc;
    part.turn_rate_dps      = g_turn_dps;

    status_board_write_imu(&part);
}

/**
 * imu_terrain_state_get / _set
 * @brief   PRV_IMU_A STATE field inside g_terrain (terrain.c owns the
 *          word; these keep the access in one place for imu_task).
 * @return  current state / none
 * @context imu_task
 */
static uint32_t imu_terrain_state_get(void)
{
    return app_bits_get(g_terrain.word_state, PRV_IMU_A_STATE_POS,
                        PRV_IMU_A_STATE_BITS);
}

static void imu_terrain_state_set(uint32_t state)
{
    g_terrain.word_state = app_bits_put(
        g_terrain.word_state, PRV_IMU_A_STATE_POS, PRV_IMU_A_STATE_BITS, state);
}

/**
 * imu_clamp_fault
 * @brief   An RNG field was clamped while packing: set the shared
 *          RANGE_CLAMP fault (Part B rule 2, kind RNG).
 * @param   b_was_clamped  true when app_bits_encode clamped
 * @return  none
 * @context imu_task
 */
static void imu_clamp_fault(bool b_was_clamped)
{
    if (b_was_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
}

/**
 * imu_acc_publish
 * @brief   Packs the three calibrated averages into SH_IMU_ACC and stores
 *          the whole word at once.
 * @context imu_task
 */
static void imu_acc_publish(void)
{
    uint32_t word = 0u;

    word = app_bits_put(word, SH_IMU_ACC_AX_POS, SH_IMU_ACC_AX_BITS,
                        imu_acc_code(g_avg_ax_mg));
    word = app_bits_put(word, SH_IMU_ACC_AY_POS, SH_IMU_ACC_AY_BITS,
                        imu_acc_code(g_avg_ay_mg));
    word = app_bits_put(word, SH_IMU_ACC_AZ_POS, SH_IMU_ACC_AZ_BITS,
                        imu_acc_code(g_avg_az_mg));
    g_sh_imu_acc = word;
}

/**
 * imu_acc_code
 * @brief   One axis in mg as its SH_IMU_ACC code (4 mg steps, clamped to
 *          +/-2044 mg, offset 512). The three axes share one range.
 * @param   mg  calibrated acceleration, mg
 * @return  0..1023
 * @context imu_task
 */
static uint32_t imu_acc_code(int32_t mg)
{
    bool b_clamped = false;

    /* integer division truncates toward zero; never >> on a signed value */
    return app_bits_encode(mg / 4, SH_IMU_ACC_AX_MIN, SH_IMU_ACC_AX_MAX,
                           SH_IMU_ACC_AX_OFFSET, &b_clamped);
}

/*** end of file ***/
