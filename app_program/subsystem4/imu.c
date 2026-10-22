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
#include "../app_bits.h"
#include "../app_layout.h"
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

/* Packed private state (Phase 4, B4.4). One writer context (imu_task)
   per word; the ISR-owned SH_IMU byte stays separate (rule 1). */
static uint32_t g_imu_prv_a; /* I2C_ERR, GOOD_CNT, CLASS_GAP,
                                LAST_CLASS, MAG_DIV, IMPACT_HOLD,
                                IMPACT_SEQ (low 4 bits) */
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
static int32_t  g_a_fwd_mg          = 0;
static int32_t  g_turn_dps          = 0;
static bool     gb_is_turn_from_enc = false;
static int32_t  g_pitch_ddeg        = 0;
static uint32_t g_hump_now_mm       = 0u;
static int32_t  g_enc_hdg_prev_cdeg = 0;

/* ---- private function prototypes ------------------------------------- */

static void     imu_task(INT stacd, void * p_exinf);
static void     imu_int1_isr(uint32_t events, uint32_t now_us);
static void     imu_read_sensors(bool * pb_read_ok, bool * pb_mag_cycle,
                                 bool * pb_mag_read);
static void     imu_i2c_result(bool b_ok, bool b_is_acc);
static void     imu_calibrate(bool b_mode_set, bool b_read_ok);
static void     imu_run_start(int32_t pos_l, int32_t pos_r);
static void     imu_turn_rate(bool b_mag_cycle, bool b_mag_read, int32_t pos_l,
                              int32_t pos_r);
static void     imu_classify(int32_t speed_cpw);
static void     imu_write_board(uint32_t motion_class);
static uint32_t imu_terrain_state_get(void);
static void     imu_terrain_state_set(uint32_t state);
static void     imu_clamp_fault(bool b_was_clamped);

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
            /* 5. Forward acceleration: the difference across the full
               50 ms window, never 10 ms. */
            g_a_fwd_mg = terrain_speed_hist_push(&g_speed_hist, speed_cpw);

            /* 6. Terrain step. ds comes from the slower wheel; a rebase
               or a glitch gives 0 so the run-start rebase can never
               become a false climb (review R03, B4.2.2 step 6). */
            {
                terrain_input_t  in;
                terrain_output_t out;
                int32_t          prev_l = app_bits_decode(
                    app_bits_get(g_imu_prv_e, PRV_IMU_E_PREV_POS_L_POS,
                                 PRV_IMU_E_PREV_POS_L_BITS),
                    PRV_IMU_E_PREV_POS_L_OFFSET);
                int32_t prev_r = app_bits_decode(
                    app_bits_get(g_imu_prv_f, PRV_IMU_F_PREV_POS_R_POS,
                                 PRV_IMU_F_PREV_POS_R_BITS),
                    PRV_IMU_F_PREV_POS_R_OFFSET);
                int32_t dpos_l    = pos_l - prev_l;
                int32_t dpos_r    = pos_r - prev_r;
                bool    b_clamped = false;

                g_imu_prv_e = app_bits_put(
                    g_imu_prv_e, PRV_IMU_E_PREV_POS_L_POS,
                    PRV_IMU_E_PREV_POS_L_BITS,
                    app_bits_encode(pos_l, PRV_IMU_E_PREV_POS_L_MIN,
                                    PRV_IMU_E_PREV_POS_L_MAX,
                                    PRV_IMU_E_PREV_POS_L_OFFSET, &b_clamped));
                imu_clamp_fault(b_clamped);
                g_imu_prv_f = app_bits_put(
                    g_imu_prv_f, PRV_IMU_F_PREV_POS_R_POS,
                    PRV_IMU_F_PREV_POS_R_BITS,
                    app_bits_encode(pos_r, PRV_IMU_F_PREV_POS_R_MIN,
                                    PRV_IMU_F_PREV_POS_R_MAX,
                                    PRV_IMU_F_PREV_POS_R_OFFSET, &b_clamped));
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
                    uint32_t run_peak =
                        app_bits_get(g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS,
                                     PRV_IMU_C_RUN_PEAK_BITS);

                    if (out.hump_done_mm > run_peak)
                    {
                        g_imu_prv_c = app_bits_put(
                            g_imu_prv_c, PRV_IMU_C_RUN_PEAK_POS,
                            PRV_IMU_C_RUN_PEAK_BITS,
                            app_bits_encode(
                                (int32_t)out.hump_done_mm,
                                PRV_IMU_C_RUN_PEAK_MIN, PRV_IMU_C_RUN_PEAK_MAX,
                                PRV_IMU_C_RUN_PEAK_OFFSET, &b_clamped));
                        /* cast: hump_done < 2^31 */
                        imu_clamp_fault(b_clamped);
                    }
                    if (out.hump_done_mm >= PRV_IMU_C_RUN_PEAK_MAX)
                    {
                        /* A25: 127 means ">= 127 mm" (correction T6);
                           reaching the top code sets the clamp fault
                           (S4-U11). */
                        fault_set(APP_FAULT_RANGE_CLAMP);
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
            imu_write_board(app_bits_get(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                                         PRV_IMU_A_LAST_CLASS_BITS));
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
    int32_t          ercd    = 0;
    uint32_t         n       = 0u;
    uint32_t         mag_div = app_bits_get(g_imu_prv_a, PRV_IMU_A_MAG_DIV_POS,
                                            PRV_IMU_A_MAG_DIV_BITS);
    bool             b_mag_due = (0u == mag_div);

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
        uint32_t idx      = 0u;
        int32_t  sum_x    = 0;
        int32_t  sum_y    = 0;
        int32_t  sum_z    = 0;
        uint32_t fifo_ovr = drv_lsm303_fifo_overruns();
        int32_t  ofs_ax   = app_bits_decode(
            app_bits_get(g_ofs_word_a, PRV_OFS_A_AX_POS, PRV_OFS_A_AX_BITS),
            PRV_OFS_A_AX_OFFSET);
        int32_t ofs_ay = app_bits_decode(
            app_bits_get(g_ofs_word_a, PRV_OFS_A_AY_POS, PRV_OFS_A_AY_BITS),
            PRV_OFS_A_AY_OFFSET);
        int32_t ofs_az = app_bits_decode(
            app_bits_get(g_ofs_word_b, PRV_OFS_B_AZ_POS, PRV_OFS_B_AZ_BITS),
            PRV_OFS_B_AZ_OFFSET);

        /* PRV_OFS_B FIFO_OVR: the driver's counter, packed here. */
        g_ofs_word_b = app_bits_put(g_ofs_word_b, PRV_OFS_B_FIFO_OVR_POS,
                                    PRV_OFS_B_FIFO_OVR_BITS, fifo_ovr);

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
               product fits int32. High-resolution mode keeps the 12
               bits << 4, so the average fits an int16 (E4 image). */
            g_avg_raw_x = (int16_t)((sum_x * g_recip_q15[n]) / IMU_AVG_Q15_ONE);
            g_avg_raw_y = (int16_t)((sum_y * g_recip_q15[n]) / IMU_AVG_Q15_ONE);
            g_avg_raw_z = (int16_t)((sum_z * g_recip_q15[n]) / IMU_AVG_Q15_ONE);
            g_avg_ax_mg =
                (((int32_t)g_avg_raw_x / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                ofs_ax;
            g_avg_ax_mg = g_avg_ax_mg * IMU_X_FORWARD_SIGN;
            g_avg_ay_mg =
                (((int32_t)g_avg_raw_y / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                ofs_ay;
            g_avg_az_mg =
                (((int32_t)g_avg_raw_z / IMU_MG_RAW_SHIFT) * IMU_MG_PER_LSB) -
                ofs_az;
        }
        *pb_read_ok = true;
    }

    if (!b_mag_due)
    {
        mag_div--;
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_MAG_DIV_POS,
                                   PRV_IMU_A_MAG_DIV_BITS, mag_div);
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
    if (err_cnt < IMU_I2C_ERR_MAX)
    {
        err_cnt++;
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_I2C_ERR_POS,
                                   PRV_IMU_A_I2C_ERR_BITS, err_cnt);
    }
    if ((err_cnt >= IMU_I2C_ERR_MAX) &&
        (TERRAIN_FAULT != imu_terrain_state_get()))
    {
        imu_terrain_state_set(TERRAIN_FAULT);
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
    uint32_t cal_cnt = app_bits_get(g_imu_prv_b, PRV_IMU_B_CAL_CNT_POS,
                                    PRV_IMU_B_CAL_CNT_BITS);

    if (b_mode_set && !gb_cal_active)
    {
        gb_cal_active    = true;
        gb_cal_done_once = false;
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
        int32_t ofs_ax    = 0;
        int32_t ofs_ay    = 0;
        int32_t ofs_az    = 0;

        /* Event-time division by the sample count (once per mode).
           The offsets are stored in 2 mg steps (PRV_OFS_*): / 2 is a
           power-of-two division, never >> on a signed value. */
        ofs_ax = g_cal_sum_x / (int32_t)IMU_CAL_SAMPLES;
        ofs_ay = g_cal_sum_y / (int32_t)IMU_CAL_SAMPLES;
        ofs_az = (g_cal_sum_z / (int32_t)IMU_CAL_SAMPLES) - 1000;

        g_ofs_word_a = app_bits_put(
            g_ofs_word_a, PRV_OFS_A_AX_POS, PRV_OFS_A_AX_BITS,
            app_bits_encode(ofs_ax / 2, PRV_OFS_A_AX_MIN, PRV_OFS_A_AX_MAX,
                            PRV_OFS_A_AX_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);
        g_ofs_word_a = app_bits_put(
            g_ofs_word_a, PRV_OFS_A_AY_POS, PRV_OFS_A_AY_BITS,
            app_bits_encode(ofs_ay / 2, PRV_OFS_A_AY_MIN, PRV_OFS_A_AY_MAX,
                            PRV_OFS_A_AY_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);
        g_ofs_word_b = app_bits_put(
            g_ofs_word_b, PRV_OFS_B_AZ_POS, PRV_OFS_B_AZ_BITS,
            app_bits_encode(ofs_az / 2, PRV_OFS_B_AZ_MIN, PRV_OFS_B_AZ_MAX,
                            PRV_OFS_B_AZ_OFFSET, &b_clamped));
        imu_clamp_fault(b_clamped);

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
    g_imu_prv_a         = app_bits_put(g_imu_prv_a, PRV_IMU_A_IMPACT_HOLD_POS,
                                       PRV_IMU_A_IMPACT_HOLD_BITS, 0u);
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
    {
        bool b_clamped = false;

        /* PREV_DIFF: the position difference that will be 50 ms old
           on the next magnetometer cycle. */
        g_imu_prv_d = app_bits_put(
            g_imu_prv_d, PRV_IMU_D_PREV_DIFF_POS, PRV_IMU_D_PREV_DIFF_BITS,
            app_bits_encode(pos_l - pos_r, PRV_IMU_D_PREV_DIFF_MIN,
                            PRV_IMU_D_PREV_DIFF_MAX, PRV_IMU_D_PREV_DIFF_OFFSET,
                            &b_clamped));
        imu_clamp_fault(b_clamped);
    }

    if (b_mag_read)
    {
        if (terrain_abs(g_pitch_ddeg) > (uint32_t)IMU_MAG_MAX_PITCH_DDEG)
        /* cast: positive constant, comparison only */
        {
            /* On a hump there is no tilt compensation: use the encoder
               rate and drop the magnetometer reference. */
            g_imu_prv_c =
                app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                             PRV_IMU_C_PREV_HDG_VALID_BITS, 0u);
            b_use_enc = true;
        }
        else
        {
            int32_t  heading = terrain_atan2_ddeg(g_mag_my - IMU_MAG_OFS_Y,
                                                  g_mag_mx - IMU_MAG_OFS_X) *
                               IMU_MAG_HEADING_SIGN;
            uint32_t heading_wrap =
                (uint32_t)terrain_heading_wrap_ddeg(heading);
            /* cast: wrap >= 0 */
            bool b_hdg_valid =
                (0u != app_bits_get(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                    PRV_IMU_C_PREV_HDG_VALID_BITS));

            if (b_hdg_valid)
            {
                uint32_t prev_hdg =
                    app_bits_get(g_imu_prv_c, PRV_IMU_C_PREV_HDG_POS,
                                 PRV_IMU_C_PREV_HDG_BITS);

                imu_dps =
                    terrain_heading_delta_ddeg(prev_hdg, heading_wrap) * 2;
            }
            else
            {
                /* Re-seed after a gap: report the encoder rate for
                   this cycle. */
                b_use_enc = true;
            }
            g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_POS,
                                       PRV_IMU_C_PREV_HDG_BITS, heading_wrap);
            g_imu_prv_c =
                app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                             PRV_IMU_C_PREV_HDG_VALID_BITS, 1u);
        }
    }
    else
    {
        /* This cycle should have read the magnetometer but did not
           (postponed backlog or I2C error): drop the reference. */
        g_imu_prv_c = app_bits_put(g_imu_prv_c, PRV_IMU_C_PREV_HDG_VALID_POS,
                                   PRV_IMU_C_PREV_HDG_VALID_BITS, 0u);
        b_use_enc   = true;
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
    uint32_t cls             = 0u;
    uint32_t impact_seq_seen = app_bits_get(
        g_imu_prv_a, PRV_IMU_A_IMPACT_SEQ_POS, PRV_IMU_A_IMPACT_SEQ_BITS);
    uint32_t impact_hold = app_bits_get(g_imu_prv_a, PRV_IMU_A_IMPACT_HOLD_POS,
                                        PRV_IMU_A_IMPACT_HOLD_BITS);
    uint32_t last_class  = app_bits_get(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                                        PRV_IMU_A_LAST_CLASS_BITS);
    uint32_t class_gap   = app_bits_get(g_imu_prv_a, PRV_IMU_A_CLASS_GAP_POS,
                                        PRV_IMU_A_CLASS_GAP_BITS);

    if ((seq_now & 0xFu) != impact_seq_seen)
    {
        g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_IMPACT_SEQ_POS,
                                   PRV_IMU_A_IMPACT_SEQ_BITS, seq_now & 0xFu);
        impact_hold = IMU_IMPACT_HOLD_TICKS;
    }
    else if (impact_hold > 0u)
    {
        impact_hold--;
    }
    g_imu_prv_a = app_bits_put(g_imu_prv_a, PRV_IMU_A_IMPACT_HOLD_POS,
                               PRV_IMU_A_IMPACT_HOLD_BITS, impact_hold);

    cls = terrain_classify(imu_terrain_state_get(), (impact_hold > 0u),
                           g_turn_dps, g_a_fwd_mg, speed_cpw);

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

/*** end of file ***/
