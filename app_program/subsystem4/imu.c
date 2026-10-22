/**
 * @file    imu.c
 * @brief   imu_task (10 ms, R2): accelerometer FIFO + magnetometer on
 *          I2C1, calibration, hump detection, motion classification,
 *          turn rate and the INT1 impact callback (SAFETY_IMPACT).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    task
 * @reqs    FR4, FR5, FR13 (detection), FR18, NFR7, NFR11
 *
 * Cycle order is fixed by B4.2.2: check-in, sensor reads, board read,
 * calibration / run start, forward acceleration, terrain step, turn
 * rate, class, board write.
 *
 * Phase 2 (baseline) representation: plain static variables, all
 * written only by imu_task, except g_imu_impact_seq which is written
 * only by the INT1 callback (R1). Phase 4 packs the statics into the
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
#include "../board_pins.h"
#include "../subsystem2/odometry.h"
#include "../subsystem2/safety.h"
#include "../subsystem2/fault.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/sys_tick.h"
#include "../subsystem5/sys_health.h"
#include "../subsystem5/drv_gpio_irq.h"

/* ---- private constants and types ------------------------------------ */

#define IMU_STACK_WORDS        (APP_STACK_IMU_BYTES / 8u)
#define IMU_AVG_Q15_ONE        (32768)
#define IMU_MG_RAW_SHIFT       (16) /* raw / 16: high-resolution mode */
#define IMU_MAG_DIV_PERIOD     (IMU_MAG_EVERY_N - 1u)
#define IMU_GOOD_READS_RECOVER (3u)
#define IMU_CLASS_GAP_MAX      (31u)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_imu_stack[IMU_STACK_WORDS];

/* Averaging reciprocals: avg = (sum x recip[n]) / 32768 for n samples. */
static int32_t const g_recip_q15[IMU_ACC_SAMPLES_MAX + 1u] = {0, 32768, 16384,
                                                              10923, 8192};

static terrain_t            g_terrain;
static terrain_speed_hist_t g_speed_hist;

/* Written only by the INT1 callback (R1), read by imu_task. */
static volatile uint8_t g_imu_impact_seq = 0;

static int16_t g_mag_mx = 0;
static int16_t g_mag_my = 0;
static int16_t g_mag_mz = 0;

static int32_t  g_ofs_ax_mg      = 0; /* calibration offsets */
static int32_t  g_ofs_ay_mg      = 0;
static int32_t  g_ofs_az_mg      = 0;
static int32_t  g_cal_sum_x      = 0; /* E2 calibration sums, live 2 s */
static int32_t  g_cal_sum_y      = 0;
static int32_t  g_cal_sum_z      = 0;
static uint32_t g_cal_cnt        = 0u;
static bool     gb_cal_active    = false;
static bool     gb_cal_done_once = false;

static int32_t g_avg_raw_x = 0; /* raw averages, before offsets */
static int32_t g_avg_raw_y = 0;
static int32_t g_avg_raw_z = 0;
static int32_t g_avg_ax_mg = 0; /* calibrated averages */
static int32_t g_avg_ay_mg = 0;
static int32_t g_avg_az_mg = 1000; /* flat rest: az = +1000 mg */

static uint32_t g_run_peak_mm       = 0u;
static int32_t  g_a_fwd_mg          = 0;
static int32_t  g_turn_dps          = 0;
static bool     gb_is_turn_from_enc = false;
static int32_t  g_pitch_ddeg        = 0;
static uint32_t g_hump_now_mm       = 0u;

static uint32_t g_mag_div           = 0u;
static uint32_t g_prev_hdg_ddeg     = 0u;
static bool     gb_prev_hdg_valid   = false;
static int32_t  g_enc_hdg_prev_cdeg = 0;
static int32_t  g_prev_pos_l        = 0;
static int32_t  g_prev_pos_r        = 0;

static uint32_t g_i2c_err_cnt     = 0u;
static uint32_t g_i2c_good_cnt    = 0u;
static uint32_t g_class_gap       = 0u;
static uint32_t g_last_class      = APP_MCLASS_STATIONARY;
static uint32_t g_impact_hold     = 0u;
static uint32_t g_impact_seq_seen = 0u;

/* ---- private function prototypes ------------------------------------- */

static void imu_task(INT stacd, void * p_exinf);
static void imu_int1_isr(uint32_t events, uint32_t now_us);
static void imu_read_sensors(bool * pb_read_ok, bool * pb_mag_cycle,
                             bool * pb_mag_read);
static void imu_i2c_result(bool b_ok, bool b_is_acc);
static void imu_calibrate(bool b_mode_set, bool b_read_ok);
static void imu_run_start(int32_t pos_l, int32_t pos_r);
static void imu_turn_rate(bool b_mag_cycle, bool b_mag_read, int32_t pos_l,
                          int32_t pos_r);
static void imu_classify(int32_t speed_cpw);
static void imu_write_board(uint32_t motion_class);

/* ---- public functions ------------------------------------------------- */

/**
 * imu_init
 * @brief   Initialises the LSM303DLHC driver (I2C1, 400 kHz, all
 *          registers from the B4.1.1 table). imu_task owns no kernel
 *          objects: everything it needs comes from the shared services.
 * @return  0 (E_OK) or a negative I2C error from the driver
 * @context usermain
 */
ER imu_init(void)
{
    return (ER)drv_lsm303_init(); /* cast: driver returns 0 or <0,
                                     same sign convention as ER */
}

/**
 * imu_start
 * @brief   Registers the INT1 callback with R1 and starts imu_task on
 *          its static stack.
 * @return  E_OK or the first kernel error
 * @context usermain
 */
ER imu_start(void)
{
    ER ercd = E_OK;
    ID tid  = 0;

    ercd = drv_gpio_irq_attach(PIN_IMU_INT1, DRV_GPIO_IRQ_RISE, imu_int1_isr);
    if (E_OK != ercd)
    {
        return ercd;
    }

    tid = app_start_task((FP)imu_task, /* cast: kernel entry type */
                         APP_PRI_IMU, g_imu_stack, APP_STACK_IMU_BYTES,
                         "imu_task");
    if (E_OK > tid)
    {
        ercd = tid; /* negative ID = kernel error code */
    }

    return ercd;
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

        /* 2. Sensors. */
        imu_read_sensors(&b_read_ok, &b_mag_cycle, &b_mag_read);

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

        if (TERRAIN_FAULT == g_terrain.state)
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
            /* 5. Forward acceleration: the difference across the full
               50 ms window, never 10 ms. */
            g_a_fwd_mg = terrain_speed_hist_push(&g_speed_hist, speed_cpw);

            /* 6. Terrain step. ds comes from the slower wheel; a rebase
               or a glitch gives 0 so the run-start rebase can never
               become a false climb (review R03, B4.2.2 step 6). */
            {
                terrain_input_t  in;
                terrain_output_t out;
                int32_t          dpos_l = pos_l - g_prev_pos_l;
                int32_t          dpos_r = pos_r - g_prev_pos_r;

                g_prev_pos_l = pos_l;
                g_prev_pos_r = pos_r;

                in.ax_mg     = g_avg_ax_mg;
                in.az_mg     = g_avg_az_mg;
                in.a_fwd_mg  = g_a_fwd_mg;
                in.ds_counts = terrain_ds_counts(dpos_l, dpos_r);

                terrain_step(&g_terrain, &in, &out);

                g_pitch_ddeg  = out.pitch_ddeg;
                g_hump_now_mm = out.hump_now_mm;

                if (out.hump_done_mm > 0u)
                {
                    if (out.hump_done_mm > g_run_peak_mm)
                    {
                        g_run_peak_mm = out.hump_done_mm;
                    }
                    (void)mission_post_event(APP_EVT_HUMP, out.hump_done_mm);
                    /* ignored: K4 drops are counted by
                       mission_post_event itself */
#if (IMU_LOG_HUMPS != 0)
                    app_log("hump int %d geo %d\n", (int32_t)out.hump_done_mm,
                            (int32_t)out.hump_geo_mm, 0);
#endif
                }
            }

            /* 7. Turn rate, every 5th cycle (the magnetometer cycle). */
            imu_turn_rate(b_mag_cycle, b_mag_read, pos_l, pos_r);

            /* 8. Class. */
            imu_classify(speed_cpw);

            /* 9. Board. */
            imu_write_board(g_last_class);
        }
    }
}

/**
 * imu_int1_isr
 * @brief   R1 callback on GP19 rising: sets SAFETY_IMPACT and bumps the
 *          impact sequence. Nothing else happens here (B4.6).
 * @param   events  edge mask (unused: only rising edges are enabled)
 * @param   now_us  R1 entry time (unused)
 * @return  none
 * @context R1 (interrupt)
 */
static void imu_int1_isr(uint32_t events, uint32_t now_us)
{
    (void)events; /* only rising edges are enabled */
    (void)now_us; /* not needed */

    safety_set_from_isr(SAFETY_IMPACT);
    g_imu_impact_seq++; /* volatile uint8_t, ISR is the only writer */
}

/**
 * imu_read_sensors
 * @brief   One accelerometer FIFO read (<= 4 samples, averaged with the
 *          reciprocal table) and, every IMU_MAG_EVERY_N-th cycle, one
 *          magnetometer read. The magnetometer is postponed when the
 *          cycle already read more than 2 samples (B4.1.1 timing).
 * @param   pb_read_ok    out: true when the accelerometer read succeeded
 *                        (an empty FIFO is still success)
 * @param   pb_mag_cycle  out: true when this is a magnetometer cycle,
 *                        even when the read was postponed or failed
 * @param   pb_mag_read   out: true when the magnetometer was read
 * @return  none
 * @context imu_task
 */
static void imu_read_sensors(bool * pb_read_ok, bool * pb_mag_cycle,
                             bool * pb_mag_read)
{
    drv_lsm303_acc_t samples[IMU_ACC_SAMPLES_MAX];
    int32_t          ercd      = 0;
    uint32_t         n         = 0u;
    bool             b_mag_due = (0u == g_mag_div);

    *pb_read_ok   = false;
    *pb_mag_cycle = b_mag_due;
    *pb_mag_read  = false;

    ercd = drv_lsm303_read_acc(samples, IMU_ACC_SAMPLES_MAX);
    if (ercd < 0)
    {
        imu_i2c_result(false, true);
    }
    else
    {
        uint32_t idx   = 0u;
        int32_t  sum_x = 0;
        int32_t  sum_y = 0;
        int32_t  sum_z = 0;

        imu_i2c_result(true, true);
        n = (uint32_t)ercd; /* cast: count >= 0 here */
        if (n > 0u)
        {
            for (idx = 0u; idx < n; idx++)
            {
                sum_x += (int32_t)samples[idx].acc_x;
                sum_y += (int32_t)samples[idx].acc_y;
                sum_z += (int32_t)samples[idx].acc_z;
            }
            /* Signed average: (sum x recip[n]) / 32768, never >> on a
               signed value (review F-07). |sum| <= 4 x 32768, so the
               product fits int32. */
            g_avg_raw_x = (sum_x * g_recip_q15[n]) / IMU_AVG_Q15_ONE;
            g_avg_raw_y = (sum_y * g_recip_q15[n]) / IMU_AVG_Q15_ONE;
            g_avg_raw_z = (sum_z * g_recip_q15[n]) / IMU_AVG_Q15_ONE;
            g_avg_ax_mg = ((g_avg_raw_x / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                          g_ofs_ax_mg;
            g_avg_ax_mg = g_avg_ax_mg * IMU_X_FORWARD_SIGN;
            g_avg_ay_mg = ((g_avg_raw_y / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                          g_ofs_ay_mg;
            g_avg_az_mg = ((g_avg_raw_z / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                          g_ofs_az_mg;
        }
        *pb_read_ok = true;
    }

    if (!b_mag_due)
    {
        g_mag_div--;
        return;
    }
    if (n > 2u)
    {
        return; /* backlog cycle: postpone one cycle */
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
    g_mag_div = IMU_MAG_DIV_PERIOD;
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
    if (b_ok)
    {
        g_i2c_err_cnt = 0u;
        if (b_is_acc && (TERRAIN_FAULT == g_terrain.state))
        {
            g_i2c_good_cnt++;
            if (g_i2c_good_cnt >= IMU_GOOD_READS_RECOVER)
            {
                g_terrain.state = TERRAIN_FLAT;
                terrain_reset(&g_terrain);
                g_i2c_good_cnt = 0u;
            }
        }
        return;
    }

    g_i2c_good_cnt = 0u;
    if (g_i2c_err_cnt < IMU_I2C_ERR_MAX)
    {
        g_i2c_err_cnt++;
    }
    if ((g_i2c_err_cnt >= IMU_I2C_ERR_MAX) &&
        (TERRAIN_FAULT != g_terrain.state))
    {
        g_terrain.state = TERRAIN_FAULT;
        fault_set(APP_FAULT_IMU);
        drv_lsm303_bus_reset();
    }
}

/**
 * imu_calibrate
 * @brief   APP_MODE_CAL_IMU: sums IMU_CAL_SAMPLES raw samples (robot
 *          braked), then sets the offsets so that at rest ax = ay = 0
 *          and az = +1000 mg. CAL_DONE is posted exactly once per mode
 *          activation (B4.2.2 step 4). The magnetometer is NOT
 *          calibrated at run time (review R15).
 * @param   b_mode_set  current state of the mode bit
 * @param   b_read_ok   true when this cycle's read succeeded (no
 *                      summation on I2C failures, so a dead bus ends
 *                      in mission's calibration timeout, not in
 *                      garbage offsets)
 * @return  none
 * @context imu_task
 */
static void imu_calibrate(bool b_mode_set, bool b_read_ok)
{
    if (b_mode_set && !gb_cal_active)
    {
        gb_cal_active    = true;
        gb_cal_done_once = false;
        g_cal_cnt        = 0u;
        g_cal_sum_x      = 0;
        g_cal_sum_y      = 0;
        g_cal_sum_z      = 0;
        g_terrain.state  = TERRAIN_CAL;
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
    if (g_cal_cnt < IMU_CAL_SAMPLES)
    {
        g_cal_sum_x += g_avg_raw_x; /* raw, before the old offsets */
        g_cal_sum_y += g_avg_raw_y;
        g_cal_sum_z += g_avg_raw_z;
        g_cal_cnt++;
        return;
    }

    if (!gb_cal_done_once)
    {
        /* Event-time division by the sample count (once per mode). */
        g_ofs_ax_mg      = g_cal_sum_x / (int32_t)IMU_CAL_SAMPLES;
        g_ofs_ay_mg      = g_cal_sum_y / (int32_t)IMU_CAL_SAMPLES;
        g_ofs_az_mg      = (g_cal_sum_z / (int32_t)IMU_CAL_SAMPLES) - 1000;
        gb_cal_done_once = true;
        g_terrain.state  = TERRAIN_FLAT;
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
    g_run_peak_mm = 0u;
    g_a_fwd_mg    = 0;
    terrain_reset(&g_terrain);
    terrain_speed_hist_reset(&g_speed_hist);
    g_prev_pos_l        = pos_l;
    g_prev_pos_r        = pos_r;
    g_enc_hdg_prev_cdeg = 0;
    g_prev_hdg_ddeg     = 0u;
    gb_prev_hdg_valid   = false;
    g_turn_dps          = 0;
    gb_is_turn_from_enc = false;
    g_pitch_ddeg        = 0;
    g_hump_now_mm       = 0u;
    g_impact_hold       = 0u;
    g_last_class        = APP_MCLASS_STATIONARY;
    g_class_gap         = 0u;
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
    int32_t enc_hdg   = 0;
    int32_t enc_dps   = 0;
    int32_t imu_dps   = 0;
    bool    b_use_enc = false;
    int32_t diff      = 0;

    if (!b_mag_cycle)
    {
        return; /* keep the last reported rate */
    }

    /* Encoder rate over the same 50 ms window (event-time division
       by 5: only every 5th cycle). */
    enc_hdg             = odometry_heading_cdeg(pos_l, pos_r);
    enc_dps             = (enc_hdg - g_enc_hdg_prev_cdeg) / 5;
    g_enc_hdg_prev_cdeg = enc_hdg;

    if (b_mag_read)
    {
        if (terrain_abs(g_pitch_ddeg) > (uint32_t)IMU_MAG_MAX_PITCH_DDEG)
        /* cast: positive constant, comparison only */
        {
            /* On a hump there is no tilt compensation: use the encoder
               rate and drop the magnetometer reference. */
            gb_prev_hdg_valid = false;
            b_use_enc         = true;
        }
        else
        {
            int32_t  heading = terrain_atan2_ddeg(g_mag_my - IMU_MAG_OFS_Y,
                                                  g_mag_mx - IMU_MAG_OFS_X) *
                               IMU_MAG_HEADING_SIGN;
            uint32_t heading_wrap =
                (uint32_t)terrain_heading_wrap_ddeg(heading);
            /* cast: wrap >= 0 */

            if (gb_prev_hdg_valid)
            {
                imu_dps =
                    terrain_heading_delta_ddeg(g_prev_hdg_ddeg, heading_wrap) *
                    2;
            }
            else
            {
                /* Re-seed after a gap: report the encoder rate for
                   this cycle. */
                b_use_enc = true;
            }
            g_prev_hdg_ddeg   = heading_wrap;
            gb_prev_hdg_valid = true;
        }
    }
    else
    {
        /* This cycle should have read the magnetometer but did not
           (postponed backlog or I2C error): drop the reference. */
        gb_prev_hdg_valid = false;
        b_use_enc         = true;
    }

    if (!b_use_enc)
    {
        diff = imu_dps - enc_dps;
        if (diff < 0)
        {
            diff = 0 - diff;
        }
        if ((terrain_abs(enc_dps) >= (uint32_t)IMU_TURN_CHECK_MIN_DPS)
            /* cast: positive constant, comparison only */
            && ((diff * 100) > ((int32_t)IMU_TURN_DISAGREE_PCT *
                                (int32_t)terrain_abs(enc_dps))))
        /* casts: both products are far below 2^31 */
        {
            b_use_enc = true; /* A7: the encoder rate wins */
        }
    }

    g_turn_dps          = (b_use_enc) ? enc_dps : imu_dps;
    gb_is_turn_from_enc = b_use_enc;
}

/**
 * imu_classify
 * @brief   Impact hold from the INT1 sequence, then the classifier.
 *          Posts APP_EVT_MOTION_CLASS when the class changed and at
 *          least IMU_CLASS_MIN_GAP_TICKS passed since the last post
 *          (D.13, B4.2.2 step 8).
 * @param   speed_cpw  this cycle's speed
 * @return  none
 * @context imu_task
 */
static void imu_classify(int32_t speed_cpw)
{
    uint32_t seq_now = (uint32_t)g_imu_impact_seq; /* 8-bit load,
                                                      atomic */
    uint32_t cls = 0u;

    if (seq_now != g_impact_seq_seen)
    {
        g_impact_seq_seen = seq_now;
        g_impact_hold     = IMU_IMPACT_HOLD_TICKS;
    }
    else if (g_impact_hold > 0u)
    {
        g_impact_hold--;
    }

    cls = terrain_classify(g_terrain.state, (g_impact_hold > 0u), g_turn_dps,
                           g_a_fwd_mg, speed_cpw);

    if (cls != g_last_class)
    {
        if (g_class_gap >= IMU_CLASS_MIN_GAP_TICKS)
        {
            (void)mission_post_event(APP_EVT_MOTION_CLASS, cls);
            /* ignored: K4 drops are counted by
               mission_post_event itself */
            g_last_class = cls;
            g_class_gap  = 0u;
        }
        else if (g_class_gap < IMU_CLASS_GAP_MAX)
        {
            g_class_gap++;
        }
    }
    else if (g_class_gap < IMU_CLASS_GAP_MAX)
    {
        g_class_gap++;
    }
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
    part.hump_peak_mm       = g_run_peak_mm;
    part.motion_class       = motion_class;
    part.b_is_turn_from_enc = gb_is_turn_from_enc;
    part.turn_rate_dps      = g_turn_dps;

    status_board_write_imu(&part);
}

/*** end of file ***/
