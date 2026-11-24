/*
 *  test_s3_diag_host.c
 *  Host verification of Subsystem 3's TEST=1 bench & timing diagnostics
 *  (S3-U56, S3-I01 WCET/period log, S3-I03 decode latency log, and
 *  S3-I05/I06 1024-sample ADC contrast/noise std-dev log), plus on-target
 *  test_line.c and test_barcode.c compiled with -DAPP_TEST=1 -Wconversion
 *  -Wsign-conversion -Werror.
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

#include "line.c"
#include "barcode.c"
#include "test_line.h"
#include "test_barcode.h"
#include <tm/tmonitor.h>

static char               g_diag_log[8192];
static size_t             g_diag_log_len = 0u;
static uint32_t           g_diag_now_us  = 100000u;
static uint32_t           g_diag_exec_us = 185u;
static bool               gb_in_cycle    = false;
static uint32_t           g_diag_raw_l   = 410u;
static uint32_t           g_diag_raw_r   = 3120u;
static uint32_t           g_diag_modes   = 0u;
static bool               gb_diag_pin    = false;
static status_board_line_t g_diag_board  = {0, 0u, 0u, false};

ID tk_cre_sem(T_CSEM const * pk_csem) { (void) pk_csem; return 1; }
ER tk_sig_sem(ID semid, INT cnt) { (void) semid; (void) cnt; return E_OK; }
ER tk_wai_sem(ID semid, INT cnt, TMO tmout) { (void) semid; (void) cnt; (void) tmout; return E_OK; }
ER tk_slp_tsk(TMO tmout) { (void) tmout; return E_OK; }
ID tk_get_tid(void) { return 1; }

ER tk_dly_tsk(RELTIM dlytim)
{
    /* Advance simulated ADC sample each 10 ms sleep during test_line_run(). */
    (void) dlytim;
    g_diag_raw_l = (410u == g_diag_raw_l) ? 420u : 410u;
    g_diag_raw_r = (3120u == g_diag_raw_r) ? 3130u : 3120u;
    g_diag_now_us += 10000u;
    line_cycle();
    return E_OK;
}

UW in_w(UW port)
{
    (void) port;
    return gb_diag_pin ? (1u << PIN_BAR_DO) : 0u;
}

void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    if (g_diag_log_len < (sizeof(g_diag_log) - 256u))
    {
        int const n = snprintf(&g_diag_log[g_diag_log_len],
                               sizeof(g_diag_log) - g_diag_log_len,
                               p_format, arg_a, arg_b, arg_c);
        if (n > 0)
        {
            g_diag_log_len += (size_t) n;
        }
    }
}

INT tm_printf(UB const * format, ...)
{
    va_list ap;
    int     n = 0;

    va_start(ap, format);
    if (g_diag_log_len < (sizeof(g_diag_log) - 256u))
    {
        n = vsnprintf(&g_diag_log[g_diag_log_len],
                      sizeof(g_diag_log) - g_diag_log_len,
                      (char const *) format, ap);
        if (n > 0)
        {
            printf("%s", &g_diag_log[g_diag_log_len]);
            g_diag_log_len += (size_t) n;
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
    return 1;
}

ER drv_gpio_irq_attach(uint32_t pin, uint32_t edges, drv_gpio_irq_cb_t p_cb)
{
    (void) pin; (void) edges; (void) p_cb;
    return E_OK;
}

void fault_set(uint32_t bit) { (void) bit; }
void drv_ir_adc_init(void) {}

void drv_ir_adc_read(uint32_t * p_left, uint32_t * p_right)
{
    *p_left  = g_diag_raw_l;
    *p_right = g_diag_raw_r;
}

ER sys_tick_register(uint32_t slot, ID task_id)
{
    (void) slot; (void) task_id;
    return E_OK;
}

void sys_tick_cycle_start(uint32_t slot) { (void) slot; }

uint32_t sys_tick_now_us(void)
{
    uint32_t out = g_diag_now_us;

    if (gb_in_cycle)
    {
        out += g_diag_exec_us;
        gb_in_cycle = false;
    }
    else
    {
        gb_in_cycle = true;
    }
    return out;
}

void sys_health_check_in(uint32_t task_bit) { (void) task_bit; }
void status_board_read(status_board_t * p_snap) { (void) p_snap; }
uint32_t status_board_path(status_board_t const * p_snap) { (void) p_snap; return 0u; }
int32_t status_board_pos_l(status_board_t const * p_snap) { (void) p_snap; return 0; }
int32_t status_board_pos_r(status_board_t const * p_snap) { (void) p_snap; return 0; }
int32_t status_board_speed_l(status_board_t const * p_snap) { (void) p_snap; return 36; }
int32_t status_board_speed_r(status_board_t const * p_snap) { (void) p_snap; return 36; }

void status_board_write_line(status_board_line_t const * p_part)
{
    g_diag_board = *p_part;
}

bool mission_mode_is_set(uint32_t mask)
{
    return (0u != (g_diag_modes & mask));
}

bool mission_post_event(uint32_t type, uint32_t value)
{
    (void) type; (void) value;
    return true;
}

uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    return (char_index & 0x3Fu) | ((cmd & 0x3u) << 6u);
}

static void log_reset(void)
{
    g_diag_log_len  = 0u;
    g_diag_log[0]   = '\0';
}

int main(void)
{
    static uint16_t const pat_star = 0x094u;
    static uint16_t const pat_a    = 0x109u;
    uint16_t const        chars[3] = { pat_star, pat_a, pat_star };
    uint32_t              idx      = 0u;

    HOST_INIT();

    (void) line_init();
    (void) barcode_init();
    gb_in_cycle = false;
    line_cycle();

    /* 1. Run real target test_line_run() & test_barcode_run() on host. */
    log_reset();
    test_line_run();
    test_barcode_run();
    HOST_CHECK("S3-U56",
               (NULL != strstr(g_diag_log, "[TEST] S3-U56 PASS"))
               && (NULL != strstr(g_diag_log, "[S3-I05/I06 boot] 16-smp L 410..420 sd 5"))
               && (NULL != strstr(g_diag_log, "[TEST] barcode SUMMARY 4 passed, 0 failed")),
               "target test_line.c and test_barcode.c pass under APP_TEST=1");

    /* 2. Run 1024 stationary cycles: verify S3-I01 WCET and S3-I05/I06 std-dev. */
    (void) line_init();
    log_reset();
    g_diag_modes = 0u;
    for (idx = 0u; idx < 1025u; idx++)
    {
        g_diag_raw_l   = (0u == (idx & 1u)) ? 400u : 420u;   /* sd = 10 */
        g_diag_raw_r   = (0u == (idx & 1u)) ? 3100u : 3106u; /* sd = 3  */
        g_diag_now_us += 10000u;
        g_diag_exec_us = (500u == idx) ? 240u : 180u;
        gb_in_cycle    = false;
        line_cycle();
    }
    HOST_CHECK("S3-U56",
               (NULL != strstr(g_diag_log,
                               "[S3-I05/I06] 1024-smp L: min 400 max 420 sd 10"))
               && (NULL != strstr(g_diag_log,
                                  "[S3-I05/I06] 1024-smp R: min 3100 max 3106 sd 3")),
               "1024-sample bench ADC logger computes exact min, max, and std-dev");
    HOST_CHECK("S3-U56",
               NULL != strstr(g_diag_log,
                              "[S3-I01] line 1000-cyc: dt 10000..10000 us, wcet 240 us"),
               "1000-cycle timing logger reports exact 10 ms period and 240 us WCET");

    /* 3. Feed *A* and verify S3-I03 decode latency log (< 50 ms). */
    (void) barcode_init();
    log_reset();
    g_diag_modes  = APP_MODE_MOTOR_EN;
    g_diag_now_us = 2000000u;
    gb_diag_pin   = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, g_diag_now_us);
    for (uint32_t c_idx = 0u; c_idx < 3u; c_idx++)
    {
        for (uint32_t bit = 0u; bit < 9u; bit++)
        {
            bool const b_w = (0u != (chars[c_idx] & (uint16_t) (1u << (8u - bit))));
            g_diag_now_us += b_w ? 30000u : 12000u;
            gb_diag_pin    = !gb_diag_pin;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, g_diag_now_us);
        }
        if (c_idx < 2u)
        {
            g_diag_now_us += 12000u;
            gb_diag_pin    = !gb_diag_pin;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, g_diag_now_us);
        }
    }
    /* Decode runs 320 us after the 29th bar edge. */
    g_diag_now_us += 320u;
    gb_in_cycle    = true;
    g_diag_exec_us = 0u;
    barcode_cycle();
    HOST_CHECK("S3-U56",
               NULL != strstr(g_diag_log,
                              "[S3-I03] BARCODE decode lat 320 us (char 10, cmd 0)"),
               "S3-I03 logger reports exact 320 us decode latency after 29th edge");

    (void) g_diag_board;
    return (0 == HOST_SUMMARY("s3_diag")) ? 0 : 1;
}
