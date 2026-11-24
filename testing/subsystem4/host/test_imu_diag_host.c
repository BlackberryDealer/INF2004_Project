/*
 *  test_imu_diag_host.c
 *  Host task simulation and TEST=1 diagnostic verification for Subsystem 4
 *  (imu.c + test_imu.c compiled with -DAPP_TEST=1 -Wconversion
 *  -Wsign-conversion -Werror):
 *    S4-U20  I2C 3-error TERRAIN_FAULT + APP_FAULT_IMU + 5-good-read recovery,
 *            IR-lift fallback while IMU is faulted, turn-rate 10% disagreement
 *            fallback (S4-I04/I11), mag logging (S4-I10), WCET logging
 *            (S4-I09), and target test_imu_run().
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef APP_TEST
#define APP_TEST (1)
#endif

#include "imu.c"
#include "test_imu.h"
#include <tm/tmonitor.h>

static char               g_imu_log[16384];
static size_t             g_imu_log_len   = 0u;
static uint32_t           g_sim_now_us    = 100000u;
static uint32_t           g_sim_exec_us   = 420u;
static bool               gb_in_read      = false;
static bool               gb_started      = true;
static uint32_t           g_sim_modes     = 0u;
static uint32_t           g_sim_faults    = 0u;
static uint32_t           g_sim_safety    = 0u;
static uint32_t           g_sim_ir_word   = 0u;
static int32_t            g_sim_acc_rc    = 1;
static drv_lsm303_acc_t   g_sim_acc_smp   = {0, 0, 16000}; /* +1000 mg */
static int32_t            g_sim_mag_rc    = 0;
static int16_t            g_sim_mx        = 1000;
static int16_t            g_sim_my        = 0;
static int16_t            g_sim_mz        = -500;
static int32_t            g_sim_pos_l     = 0;
static int32_t            g_sim_pos_r     = 0;
static int32_t            g_sim_spd_cpw   = 0;
static status_board_imu_t g_sim_imu_board = {0, 0u, 0u, 0u, false, 0};

ID tk_cre_sem(T_CSEM const * pk_csem) { (void) pk_csem; return 1; }
ER tk_sig_sem(ID semid, INT cnt) { (void) semid; (void) cnt; return E_OK; }
ER tk_wai_sem(ID semid, INT cnt, TMO tmout) { (void) semid; (void) cnt; (void) tmout; return E_OK; }
ER tk_slp_tsk(TMO tmout) { (void) tmout; return E_OK; }
ER tk_dly_tsk(RELTIM dlytim) { (void) dlytim; return E_OK; }
ID tk_get_tid(void) { return 2; }
UW in_w(UW port) { (void) port; return 0u; }

void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    if (g_imu_log_len < (sizeof(g_imu_log) - 256u))
    {
        int const n = snprintf(&g_imu_log[g_imu_log_len],
                               sizeof(g_imu_log) - g_imu_log_len,
                               p_format, arg_a, arg_b, arg_c);
        if (n > 0)
        {
            g_imu_log_len += (size_t) n;
        }
    }
}

INT tm_printf(UB const * format, ...)
{
    va_list ap;
    int     n = 0;

    va_start(ap, format);
    if (g_imu_log_len < (sizeof(g_imu_log) - 256u))
    {
        n = vsnprintf(&g_imu_log[g_imu_log_len],
                      sizeof(g_imu_log) - g_imu_log_len,
                      (char const *) format, ap);
        if (n > 0)
        {
            printf("%s", &g_imu_log[g_imu_log_len]);
            g_imu_log_len += (size_t) n;
        }
    }
    va_end(ap);
    return (INT) n;
}

ID app_start_task(FP p_entry, PRI priority, uint64_t * p_stack,
                  uint32_t stack_bytes, char const * p_name)
{
    (void) p_entry; (void) priority; (void) p_stack;
    (void) stack_bytes; (void) p_name;
    return 2;
}

ER sys_tick_register(uint32_t slot, ID task_id) { (void) slot; (void) task_id; return E_OK; }
void sys_tick_cycle_start(uint32_t slot) { (void) slot; }

uint32_t sys_tick_now_us(void)
{
    uint32_t out = g_sim_now_us;

    if (gb_in_read)
    {
        out += g_sim_exec_us;
        gb_in_read = false;
    }
    else
    {
        gb_in_read = true;
    }
    return out;
}

void sys_health_check_in(uint32_t task_bit) { (void) task_bit; }
bool sys_health_is_started(void) { return gb_started; }
void fault_set(uint32_t bit) { g_sim_faults |= bit; }
void safety_set(uint32_t bit) { g_sim_safety |= bit; }
uint32_t safety_get(void) { return g_sim_safety; }
uint32_t line_raw_word(void) { return g_sim_ir_word; }
bool mission_mode_is_set(uint32_t mask) { return (0u != (g_sim_modes & mask)); }
bool mission_post_event(uint32_t type, uint32_t value) { (void) type; (void) value; return true; }

void status_board_read(status_board_t * p_snap) { (void) p_snap; }
int32_t status_board_pos_l(status_board_t const * p_snap) { (void) p_snap; return g_sim_pos_l; }
int32_t status_board_pos_r(status_board_t const * p_snap) { (void) p_snap; return g_sim_pos_r; }
int32_t status_board_speed_l(status_board_t const * p_snap) { (void) p_snap; return g_sim_spd_cpw; }
int32_t status_board_speed_r(status_board_t const * p_snap) { (void) p_snap; return g_sim_spd_cpw; }
void status_board_write_imu(status_board_imu_t const * p_part) { g_sim_imu_board = *p_part; }

int32_t drv_lsm303_init(void) { return (g_sim_acc_rc < 0) ? g_sim_acc_rc : 0; }
uint32_t drv_lsm303_addr(void) { return 0x1Du; }
int32_t drv_lsm303_last_error(void) { return g_sim_acc_rc; }
void drv_lsm303_bus_reset(void) {}
uint32_t drv_lsm303_fifo_overruns(void) { return 0u; }

int32_t drv_lsm303_read_acc(drv_lsm303_acc_t * p_out, uint32_t max_samples)
{
    if ((g_sim_acc_rc > 0) && (max_samples > 0u))
    {
        p_out[0] = g_sim_acc_smp;
    }
    return g_sim_acc_rc;
}

int32_t drv_lsm303_read_mag(int16_t * p_mx, int16_t * p_my, int16_t * p_mz)
{
    *p_mx = g_sim_mx;
    *p_my = g_sim_my;
    *p_mz = g_sim_mz;
    return g_sim_mag_rc;
}

/* Executes one exact 10 ms pass of imu_task (steps 2-9). */
static void sim_imu_cycle(void)
{
    bool           b_read_ok        = false;
    bool           b_mag_cycle      = false;
    bool           b_mag_read       = false;
    bool           b_mode_cal       = false;
    bool           b_mode_run_start = false;
    status_board_t snap             = {0u};
    int32_t        pos_l            = 0;
    int32_t        pos_r            = 0;
    int32_t        speed_cpw        = 0;

    g_sim_now_us += 10000u;
    gb_in_read    = false;

    if (imu_configure_if_needed())
    {
        imu_read_sensors(&b_read_ok, &b_mag_cycle, &b_mag_read);
    }

    status_board_read(&snap);
    pos_l     = status_board_pos_l(&snap);
    pos_r     = status_board_pos_r(&snap);
    speed_cpw = (status_board_speed_l(&snap) + status_board_speed_r(&snap)) / 2;

    b_mode_cal       = mission_mode_is_set(APP_MODE_CAL_IMU);
    b_mode_run_start = mission_mode_is_set(APP_MODE_RUN_START);

    imu_calibrate(b_mode_cal, b_read_ok);
    if (b_mode_run_start)
    {
        imu_run_start(pos_l, pos_r);
    }
    imu_lift_check(b_read_ok && !b_mode_cal);

    if (TERRAIN_FAULT == imu_terrain_state_get())
    {
        /* FAULT: board keeps last good values. */
    }
    else if (b_mode_cal || b_mode_run_start)
    {
        imu_write_board(APP_MCLASS_STATIONARY);
    }
    else
    {
        imu_terrain_update(pos_l, pos_r, speed_cpw);
        imu_turn_rate(b_mag_cycle, b_mag_read, pos_l, pos_r);
        imu_classify(speed_cpw);
        imu_write_board(app_bits_get(g_imu_prv_a, PRV_IMU_A_LAST_CLASS_POS,
                                     PRV_IMU_A_LAST_CLASS_BITS));
    }
}

int main(void)
{
    uint32_t idx = 0u;

    HOST_INIT();

    /* 1. Run real target test_imu_run() under APP_TEST=1. */
    test_imu_run();
    HOST_CHECK("S4-U20",
               NULL != strstr(g_imu_log, "[TEST] imu SUMMARY 6 passed, 0 failed"),
               "target test_imu.c passes all 6 checks under APP_TEST=1");

    /* 2. Init + 1000 stationary cycles: verify S4-I09 WCET and S4-I10 mag logs. */
    g_imu_log_len = 0u;
    g_imu_log[0]  = '\0';
    g_sim_acc_rc  = 1;
    g_sim_modes   = 0u;
    terrain_reset(&g_terrain);
    terrain_speed_hist_reset(&g_speed_hist);
    (void) imu_init();
    for (idx = 0u; idx < 1000u; idx++)
    {
        g_sim_exec_us = (300u == idx) ? 610u : 420u;
        sim_imu_cycle();
    }
    HOST_CHECK("S4-U20",
               (NULL != strstr(g_imu_log, "[S4-I09] imu_read 1000-cyc wcet 610 us"))
               && (NULL != strstr(g_imu_log, "[S4-I10] mag raw mx 1000 my 0 mz -500")),
               "S4-I09 WCET and S4-I10 magnetometer logs emitted under APP_TEST=1");

    /* 3. Turn-rate disagreement (S4-I04/I11): encoder turns while mag stays fixed. */
    g_imu_log_len = 0u;
    g_imu_log[0]  = '\0';
    g_sim_modes   = APP_MODE_MOTOR_EN;
    for (idx = 0u; idx < 10u; idx++)
    {
        g_sim_pos_l += 4;
        g_sim_pos_r -= 4;
        sim_imu_cycle();
    }
    HOST_CHECK("S4-U20",
               g_sim_imu_board.b_is_turn_from_enc
               && (NULL != strstr(g_imu_log, "use_enc 1")),
               "S4-I04/I11 turn-rate >10% disagreement selects encoder rate and logs it");

    /* 4. I2C unplug -> 3 errors -> TERRAIN_FAULT + APP_FAULT_IMU; IR lift still stops! */
    g_sim_faults  = 0u;
    g_sim_safety  = 0u;
    g_sim_acc_rc  = -2; /* NACK on both acc and mag addresses */
    g_sim_mag_rc  = -2;
    sim_imu_cycle();
    HOST_CHECK("S4-U20", 0u == (g_sim_faults & APP_FAULT_IMU),
               "1st I2C error cycle does not yet latch APP_FAULT_IMU");
    sim_imu_cycle();
    sim_imu_cycle();
    HOST_CHECK("S4-U20",
               (0u != (g_sim_faults & APP_FAULT_IMU))
               && (TERRAIN_FAULT == imu_terrain_state_get()),
               "3 consecutive I2C errors enter TERRAIN_FAULT and set APP_FAULT_IMU");

    /* While IMU is in TERRAIN_FAULT, IR floor-loss lift-stop still triggers in 30 cycles! */
    g_sim_ir_word = app_bits_put(0u, SH_IR_RAW_L_POS, SH_IR_RAW_L_BITS, 4020u);
    g_sim_ir_word = app_bits_put(g_sim_ir_word, SH_IR_RAW_R_POS, SH_IR_RAW_R_BITS, 4020u);
    for (idx = 0u; idx < IMU_LIFT_IR_CYCLES; idx++)
    {
        sim_imu_cycle();
    }
    HOST_CHECK("S4-U20", 0u != (g_sim_safety & SAFETY_LIFT),
               "IR lift-stop fallback sets SAFETY_LIFT even when IMU bus is faulted");

    /* Reconnect I2C: rewrite registers + 5 good acc reads recover TERRAIN_FLAT. */
    g_sim_acc_rc  = 1;
    g_sim_mag_rc  = 0;
    g_sim_ir_word = 0u;
    for (idx = 0u; idx < (1u + IMU_GOOD_READS_RECOVER); idx++)
    {
        sim_imu_cycle();
    }
    HOST_CHECK("S4-U20", TERRAIN_FLAT == imu_terrain_state_get(),
               "register rewrite + 5 good accelerometer reads recover TERRAIN_FLAT");

    return (0 == HOST_SUMMARY("imu_diag")) ? 0 : 1;
}
