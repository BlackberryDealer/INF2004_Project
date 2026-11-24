/*
 * test_hmi_health_host.c
 * Host unit tests for packed hmi.c (PRV_MIS_G, S5-U84) and packed
 * sys_health.c (PRV_HLT, S5-U85).
 * Host-only test code, outside BARR-C scope.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "app_program/app.h"
#include "app_program/subsystem5/hmi.c"
#include "app_program/subsystem5/sys_health.c"

static bool     gb_btn_run_low   = false;
static bool     gb_btn_reset_low = false;
static bool     gb_ws2812_ok     = true;
static uint32_t g_led_r          = 255u;
static uint32_t g_led_g          = 255u;
static uint32_t g_led_b          = 255u;
static bool     gb_buzzer_on     = false;

static uint32_t g_now_us         = 10000u;
static UINT     g_flg_ptn        = 0u;
static uint32_t g_wdt_load       = 0u;
static uint32_t g_wdt_loads      = 0u;
static uint32_t g_wdt_ctrl       = 0u;
static uint32_t g_wdt_reason     = 0u;
static uint32_t g_led_toggles    = 0u;

void gpio_init(uint32_t pin) { (void) pin; }
void gpio_set_dir(uint32_t pin, bool out) { (void) pin; (void) out; }
void gpio_pull_up(uint32_t pin) { (void) pin; }

bool gpio_get(uint32_t pin)
{
    if (PIN_BTN_RUN == pin) { return !gb_btn_run_low; }
    if (PIN_BTN_RESET == pin) { return !gb_btn_reset_low; }
    return true;
}

void drv_buzzer_init(void) { gb_buzzer_on = false; }
void drv_buzzer_on(void)   { gb_buzzer_on = true; }
void drv_buzzer_off(void)  { gb_buzzer_on = false; }

bool drv_ws2812_init(void) { return gb_ws2812_ok; }
void drv_ws2812_set(uint32_t r, uint32_t g, uint32_t b)
{
    g_led_r = r;
    g_led_g = g;
    g_led_b = b;
}

void drv_led_alive_init(void)   { g_led_toggles = 0u; }
void drv_led_alive_toggle(void) { g_led_toggles++; }
uint32_t sys_tick_now_us(void)  { return g_now_us; }

ID tk_cre_flg(T_CFLG const * p)
{
    g_flg_ptn = p ? p->iflgptn : 0u;
    return 1;
}

ER tk_set_flg(ID id, UINT p) { (void) id; g_flg_ptn |= p; return E_OK; }
ER tk_clr_flg(ID id, UINT p) { (void) id; g_flg_ptn &= p; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG * r) { (void) id; r->flgptn = g_flg_ptn; return E_OK; }

uint32_t in_w(uint32_t port)
{
    if (SYS_HEALTH_WDT_REASON == port) { return g_wdt_reason; }
    return 0u;
}

void out_w(uint32_t port, uint32_t val)
{
    if (SYS_HEALTH_WDT_LOAD == port)
    {
        g_wdt_load = val;
        g_wdt_loads++;
    }
}

void set_w(uint32_t port, uint32_t val)
{
    if (SYS_HEALTH_WDT_CTRL == port) { g_wdt_ctrl |= val; }
}

void clr_w(uint32_t port, uint32_t val)
{
    if (SYS_HEALTH_WDT_CTRL == port) { g_wdt_ctrl &= ~val; }
}

void app_log(char const * f, int32_t a, int32_t b, int32_t c)
{
    (void) f; (void) a; (void) b; (void) c;
}

static int pass_n, fail_n;
#define CHECK(id, c, d) do { \
    if (c) { pass_n++; printf("[TEST] %s PASS  %s\n", id, d); } \
    else   { fail_n++; printf("[TEST] %s FAIL  %s (line %d)\n", id, d, __LINE__); } \
} while (0)

int main(void)
{
    uint32_t idx = 0u;

    /* ---- S5-U84: packed hmi.c (buttons, RGB colours, beeps/alarms) ---- */
    CHECK("S5-U84", hmi_init(), "hmi_init returns true when WS2812 PIO loads");
    gb_ws2812_ok = false;
    CHECK("S5-U84", !hmi_init(), "hmi_init returns false on PIO failure (for APP_FAULT_PIO_LOAD)");
    gb_ws2812_ok = true;

    /* Short 2-tick bounce on RUN ignored; 3-tick press fires HMI_RUN_PRESS on release. */
    gb_btn_run_low = true;
    (void) hmi_poll();
    (void) hmi_poll();
    gb_btn_run_low = false;
    CHECK("S5-U84", HMI_NONE == hmi_poll(), "2-tick RUN bounce does not trigger HMI_RUN_PRESS");

    gb_btn_run_low = true;
    for (idx = 0u; idx < MISSION_BTN_DEBOUNCE_TICKS; idx++)
    {
        CHECK("S5-U84", HMI_NONE == hmi_poll(), "RUN held: no event before release");
    }
    gb_btn_run_low = false;
    CHECK("S5-U84", HMI_RUN_PRESS == hmi_poll(), "RUN released after 30 ms debounce -> HMI_RUN_PRESS");

    /* RESET: 199 ticks -> none; 200th tick -> HMI_RESET_HOLD once; held further -> none. */
    gb_btn_reset_low = true;
    for (idx = 0u; idx < (MISSION_BTN_RESET_HOLD_TICKS - 1u); idx++)
    {
        (void) hmi_poll();
    }
    CHECK("S5-U84", HMI_RESET_HOLD == hmi_poll(), "RESET held 200 polls (2.0 s) -> HMI_RESET_HOLD");
    CHECK("S5-U84", HMI_NONE == hmi_poll(), "RESET held past 2.0 s does not repeat before release");
    gb_btn_reset_low = false;
    (void) hmi_poll();

    /* State colours and buzzer patterns. */
    hmi_show(APP_SYS_BOOT, 10000u);
    CHECK("S5-U84", (HMI_LEVEL == g_led_r) && (HMI_LEVEL == g_led_g) && (HMI_LEVEL == g_led_b) && gb_buzzer_on,
          "BOOT: white LED + 100 ms boot beep on");
    hmi_show(APP_SYS_BOOT, 120000u);
    CHECK("S5-U84", !gb_buzzer_on, "BOOT after 110 ms: boot beep off");

    hmi_show(APP_SYS_READY, 200000u);
    CHECK("S5-U84", (0u == g_led_r) && (HMI_LEVEL == g_led_g) && (0u == g_led_b) && !gb_buzzer_on,
          "READY: solid green LED, buzzer silent");
    hmi_set_warning(true);
    hmi_show(APP_SYS_READY, 210000u);
    CHECK("S5-U84", (0u == g_led_r) && (0u == g_led_g) && (HMI_LEVEL == g_led_b) && gb_buzzer_on,
          "READY + start warning: flashing blue LED + continuous buzzer");
    hmi_set_warning(false);

    hmi_show(APP_SYS_RUN, 300000u);
    CHECK("S5-U84", (0u == g_led_r) && (0u == g_led_g) && (HMI_LEVEL == g_led_b) && !gb_buzzer_on,
          "RUN: solid blue LED, buzzer silent");

    hmi_show(APP_SYS_FINISHED, 400000u);
    CHECK("S5-U84", (0u == g_led_r) && (HMI_LEVEL == g_led_g) && gb_buzzer_on,
          "FINISHED entry: green blink on + beep 1 on");
    hmi_show(APP_SYS_FINISHED, 550000u);
    CHECK("S5-U84", !gb_buzzer_on, "FINISHED +150 ms: gap between double beeps");
    hmi_show(APP_SYS_FINISHED, 650000u);
    CHECK("S5-U84", gb_buzzer_on, "FINISHED +250 ms: second beep on");
    hmi_show(APP_SYS_FINISHED, 750000u);
    CHECK("S5-U84", !gb_buzzer_on, "FINISHED +350 ms: double beep finished");

    hmi_show(APP_SYS_FAULT, 1000000u);
    CHECK("S5-U84", (HMI_LEVEL == g_led_r) && (0u == g_led_g) && (0u == g_led_b) && gb_buzzer_on,
          "FAULT: flashing red LED + alarm tone");

    /* ---- S5-U85: packed sys_health.c (500 ms start -> 30 ms normal) --- */
    g_now_us = 50000u;
    CHECK("S5-U85", E_OK == sys_health_init(), "sys_health_init creates FLG_ALIVE");
    sys_health_enable_watchdog();
    CHECK("S5-U85", (SYS_HEALTH_LOAD_START == g_wdt_load) && !sys_health_is_started(),
          "watchdog starts on 500 ms window (not yet started)");

    /* Missing OBSTACLE check-in -> no feed, stays unstarted. */
    g_wdt_loads = 0u;
    sys_health_check_in(SYS_HEALTH_BIT_LINE);
    sys_health_check_in(SYS_HEALTH_BIT_MOTION);
    sys_health_check_in(SYS_HEALTH_BIT_IMU);
    sys_health_service();
    CHECK("S5-U85", (0u == g_wdt_loads) && !sys_health_is_started(),
          "without obstacle check-in, startup window is NOT fed");

    /* Obstacle checks in -> switches to 30 ms normal window! */
    sys_health_check_in(SYS_HEALTH_BIT_OBSTACLE);
    sys_health_service();
    CHECK("S5-U85", sys_health_is_started() && (SYS_HEALTH_LOAD_NORMAL == g_wdt_load)
          && (0u == (g_flg_ptn & SYS_HEALTH_FAST_BITS)) && (1u == g_led_toggles),
          "all 4 check-ins switch watchdog to 30 ms normal window and clear fast bits");

    /* Two normal feeds 12 ms then 18 ms apart -> max gap = 18000 us. */
    g_now_us += 12000u;
    sys_health_check_in(SYS_HEALTH_BIT_LINE);
    sys_health_check_in(SYS_HEALTH_BIT_MOTION);
    sys_health_check_in(SYS_HEALTH_BIT_IMU);
    sys_health_service();

    g_now_us += 18000u;
    sys_health_check_in(SYS_HEALTH_BIT_LINE);
    sys_health_check_in(SYS_HEALTH_BIT_MOTION);
    sys_health_check_in(SYS_HEALTH_BIT_IMU);
    sys_health_service();
    CHECK("S5-U85", 18000u == sys_health_feed_gap_max_us(true),
          "feed_gap_max_us records 18 000 us longest normal feed interval");
    CHECK("S5-U85", 0u == sys_health_feed_gap_max_us(false),
          "feed_gap_max_us(true) resets max gap to 0");

    /* Stale obstacle check-in (66 ms > 65 ms limit) -> refuses to feed! */
    g_now_us += 36000u; /* total 66 ms since obstacle's check-in at 50000 us */
    g_wdt_loads = 0u;
    sys_health_check_in(SYS_HEALTH_BIT_LINE);
    sys_health_check_in(SYS_HEALTH_BIT_MOTION);
    sys_health_check_in(SYS_HEALTH_BIT_IMU);
    sys_health_service();
    CHECK("S5-U85", 0u == g_wdt_loads,
          "obstacle check-in 66 ms old (> 65 ms) withholds watchdog feed");

    printf("[TEST] hmi_health SUMMARY %d passed, %d failed\n", pass_n, fail_n);
    return fail_n ? 1 : 0;
}
