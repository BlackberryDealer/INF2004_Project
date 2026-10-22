/*
 * sim_obstacle_host.c
 * Laptop simulation of obstacle.c (echo window and freshness counters,
 * confirmation, scanner, profile and plan) against a simple world: a
 * flat face in front of the robot. Covers S5-U14 (distance conversion)
 * and the host parts of S5-U17, S5-U24, S5-U25, S5-U26 and S5-U30.
 * Host-only test code, outside BARR-C scope. Run: make -C testing/subsystem5/host
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "app_program/app.h"
typedef struct { void *exinf; ATR sematr; INT isemcnt; INT maxsem; } T_CSEM;
ID tk_cre_sem(T_CSEM const *p); ER tk_wai_sem(ID id, INT c, TMO t); ER tk_sig_sem(ID id, INT c);
ER tk_slp_tsk(TMO t); ID tk_get_tid(void);
#include "app_program/subsystem5/obstacle.c"

static int sem; static uint32_t g_now, g_safety, g_fault, g_modes, g_servo = 90;
static bool g_echo_high;           /* echo line already high (stuck) */
ID tk_cre_sem(T_CSEM const *p) { (void) p; return 3; }
ER tk_wai_sem(ID id, INT c, TMO t) { (void) id; (void) c;
  if (sem > 0) { sem--; return E_OK; }
  if (t > 0) { g_now += (uint32_t) t * 1000u; }   /* time passes while waiting */
  return E_TMOUT; }
ER tk_sig_sem(ID id, INT c) { (void) id; (void) c; if (sem >= 1) return -43; sem++; return E_OK; }
ER tk_slp_tsk(TMO t) { (void) t; return E_OK; }
ID tk_get_tid(void) { return 5; }
ID tk_cre_mtx(T_CMTX const *p) { (void) p; return 1; }
ER tk_loc_mtx(ID id, TMO t) { (void) id; (void) t; return E_OK; }
ER tk_unl_mtx(ID id) { (void) id; return E_OK; }
ID app_start_task(FP e, PRI p, uint64_t *s, uint32_t b, char const *n) { (void)e;(void)p;(void)s;(void)b;(void)n; return 1; }
void app_check(ER e, char const *w) { (void) e; (void) w; }
void app_log(char const *f, int32_t a, int32_t b, int32_t c) { printf("   log: "); printf(f, a, b, c); }
ER sys_tick_register(uint32_t s, ID t) { (void) s; (void) t; return E_OK; }
void sys_tick_cycle_start(uint32_t s) { (void) s; }
uint32_t sys_tick_now_us(void) { return g_now; }
void sys_health_check_in(uint32_t b) { (void) b; }
ER drv_gpio_irq_attach(uint32_t p, uint32_t e, drv_gpio_irq_cb_t cb) { (void) p; (void) e; (void) cb; return E_OK; }
void drv_servo_init(void) {}
void drv_servo_set_deg(uint32_t d) { g_servo = d; }
void drv_ultrasonic_init(void) {}
void safety_set(uint32_t b) { g_safety |= b; }
void fault_set(uint32_t b) { g_fault |= b; }
uint32_t odometry_counts_to_mm(uint32_t c) { return (c * 349u) / 1000u; }
bool mission_mode_is_set(uint32_t b) { return 0u != (g_modes & b); }
bool gpio_get(uint32_t pin) { (void) pin; return g_echo_high; }
uint32_t in_w(uint32_t port) { (void) port; return g_echo_high ? (1u << PIN_US_ECHO) : 0u; }
/* world */
static int w_mode;                 /* 0 = fixed distance, 1 = face, 2 = silent */
static uint32_t w_dist = 1000;
static int32_t w_y = 240, w_xl = -60, w_xr = 80;
static uint32_t world_mm(void) {
  if (0 == w_mode) return w_dist;
  { double th = (double) g_servo * 3.14159265 / 180.0, s = sin(th), c = cos(th);
    double x = (double) w_y * c / s;
    if (x >= w_xl && x <= w_xr) return (uint32_t) ((double) w_y / s); }
  return 6000; }
void drv_ultrasonic_trigger(void) {
  uint32_t d, w;
  if (2 == w_mode) return;
  d = world_mm(); w = (uint32_t) ((double) d * 5.8);
  if (w > 38000u) w = 38000u;
  obstacle_echo_isr(DRV_GPIO_IRQ_RISE, g_now + 500u);
  obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 500u + w); }
static uint32_t ev[64], evv[64], evsafety[64], evlegs[64], evservo[64]; static int nev;
bool mission_post_event(uint32_t t, uint32_t v) { status_board_t s; status_board_read(&s);
  ev[nev] = t; evv[nev] = v; evsafety[nev] = g_safety; evlegs[nev] = status_board_leg_lateral(&s); evservo[nev] = g_servo; nev++; return true; }
static int pass_n, fail_n;
#define CHECK(id, c, d) do { if (c) { pass_n++; printf("[TEST] %s PASS  %s\n", id, d); } else { fail_n++; printf("[TEST] %s FAIL  %s (line %d) state=%u servo=%u nev=%d\n", id, d, __LINE__, g_state, g_servo, nev); } } while (0)
static void cyc(void) { g_now += 60000u; obstacle_cycle(); }
static int count_ev(uint32_t t) { int n = 0; for (int i = 0; i < nev; i++) if (ev[i] == t) n++; return n; }
static uint32_t dist(void) { status_board_t s; status_board_read(&s); return status_board_distance(&s); }
int main(void) {
  status_board_t snap;
  status_board_init(); obstacle_init();
  /* S5-U14: conversion */
  CHECK("S5-U14", 100u == obstacle_width_to_mm(580u / 4u), "580 us -> 100 mm");
  CHECK("S5-U14", 1000u == obstacle_width_to_mm(5800u / 4u), "5800 us -> 1000 mm");
  CHECK("S5-U14", APP_DIST_NONE_MM == obstacle_width_to_mm(4095u), "40 000 us (saturated width) -> APP_DIST_NONE_MM");
  CHECK("S5-U14", 0u == (obstacle_width_to_mm(300u) % 4u), "readings stored in 4 mm steps");
  /* S5-U30: power-up grace. The car's HC-SR04P stays silent for a while
     after power-up; that must not latch NO_ECHO, but a dead sensor must. */
  { int ok = 1;
    w_mode = 2; g_fault = 0;
    for (int i = 0; i < 5; i++) { cyc(); if (g_fault & APP_FAULT_NO_ECHO) ok = 0; }
    w_mode = 0; w_dist = 1000; cyc();
    CHECK("S5-U30", ok && 0u == (g_fault & APP_FAULT_NO_ECHO) && 0u == g_warmup_left, "5 silent pings after boot, then an echo: no NO_ECHO, grace ends");
    w_mode = 2; cyc(); cyc(); cyc();
    CHECK("S5-U30", 0u != (g_fault & APP_FAULT_NO_ECHO), "after the first echo, 3 silent pings raise NO_ECHO as before");
    g_fault = 0; g_silent = 0; g_warmup_left = OBS_WARMUP_PINGS; ok = 1;
    for (uint32_t i = 0; i < OBS_WARMUP_PINGS; i++) { cyc(); if (g_fault & APP_FAULT_NO_ECHO) ok = 0; }
    CHECK("S5-U30", ok, "sensor never answers: no NO_ECHO during the 34-ping grace (~2 s)");
    cyc(); cyc(); cyc();
    CHECK("S5-U30", 0u != (g_fault & APP_FAULT_NO_ECHO), "sensor never answers: NO_ECHO 3 pings after the grace");
    g_fault = 0; g_silent = 0; w_mode = 0; }
  w_mode = 0; w_dist = 1000; cyc(); cyc();
  CHECK("S5-U08", APP_SCAN_MONITOR == g_state && 0 == nev && 1000u == dist(), "far object: MONITOR, 1000 mm measured");
  w_dist = 300; cyc(); CHECK("S5-U08", 300u == dist() && APP_SCAN_MONITOR == g_state, "300 mm, not yet confirmed");
  w_dist = 290; cyc(); CHECK("S5-U08", APP_SCAN_APPROACH == g_state && 1 == count_ev(APP_EVT_OBSTACLE_AHEAD), "two agreeing readings < 400 -> OBSTACLE_AHEAD");
  w_dist = 300; cyc(); w_dist = 245; cyc(); CHECK("S5-U08", APP_SCAN_APPROACH == g_state, "245 right after 300 is not confirmed");
  w_dist = 240; cyc();
  CHECK("S5-U08", APP_SCAN_STOPPED == g_state && 1 == count_ev(APP_EVT_OBSTACLE), "confirmed < 251 -> OBSTACLE, STOPPED");
  CHECK("S5-U08", (evsafety[nev - 1] & SAFETY_ESTOP) != 0u, "ESTOP set before OBSTACLE was posted");
  g_modes = APP_MODE_STOPPED | APP_MODE_RESCAN; w_mode = 1;
  cyc(); CHECK("S5-U08", APP_SCAN_COARSE == g_state && 30u == g_servo, "RESCAN rise -> COARSE at 30 deg");
  for (int i = 0; i < 120 && (APP_SCAN_COARSE == g_state || APP_SCAN_FINE == g_state); i++) cyc();
  CHECK("S5-U25", APP_SCAN_WATCH_SETTLE == g_state && 0 == count_ev(APP_EVT_PLAN) && 30u == g_servo, "scan done -> WATCH_SETTLE, servo at watch angle, PLAN not posted yet");
  cyc(); cyc(); cyc(); CHECK("S5-U25", 0 == count_ev(APP_EVT_PLAN), "no PLAN before OBS_WATCH_SETTLE_PINGS cycles");
  cyc(); CHECK("S5-U25", 1 == count_ev(APP_EVT_PLAN) && APP_SIDE_LEFT == evv[nev - 1] && APP_SCAN_BYPASS_WATCH == g_state, "PLAN(left) after exactly 4 cycles -> BYPASS_WATCH");
  CHECK("S5-U08", evlegs[nev - 1] > 0u && 0u == (evlegs[nev - 1] % 4u), "legs on the board (4 mm steps) before PLAN");
  status_board_read(&snap);
  printf("   profile: width %u nearest %u legs %u/%u step %u inferred 0x%x\n", status_board_width(&snap), status_board_nearest(&snap), status_board_leg_lateral(&snap), status_board_leg_forward(&snap), status_board_angle_step(&snap), g_inferred);
  CHECK("S5-U08", status_board_is_profile_valid(&snap) && status_board_width(&snap) >= 100 && status_board_width(&snap) <= 145, "width at 5 deg resolution (face -60..+80)");
  CHECK("S5-U08", 0u != g_inferred && 0u == (g_inferred & g_measured), "unmeasured steps marked inferred");
  g_modes = 0; w_mode = 0; w_dist = 200; g_safety = 0; cyc(); cyc();
  CHECK("S5-U08", 0 == count_ev(APP_EVT_SIDE_TOO_CLOSE), "side 200 mm: fine");
  w_dist = 50; cyc(); cyc();
  CHECK("S5-U08", 1 == count_ev(APP_EVT_SIDE_TOO_CLOSE) && (evsafety[nev - 1] & SAFETY_ESTOP) && APP_SCAN_STOPPED == g_state, "side 50 mm: ESTOP, then SIDE_TOO_CLOSE once, STOPPED");
  g_modes = APP_MODE_RESUME; cyc(); CHECK("S5-U08", APP_SCAN_MONITOR == g_state && 90u == g_servo, "RESUME rise -> MONITOR, servo 90");
  g_modes = 0; w_dist = 1000; cyc(); cyc();
  /* NO_PATH */
  w_dist = 380; cyc(); cyc(); w_dist = 240; cyc(); cyc(); CHECK("S5-U08", APP_SCAN_STOPPED == g_state, "second obstacle -> STOPPED");
  g_modes = APP_MODE_STOPPED | APP_MODE_RESCAN; w_mode = 1; w_y = 200; w_xl = -400; w_xr = 400;
  for (int i = 0; i < 120 && 0 == count_ev(APP_EVT_NO_PATH); i++) cyc();
  CHECK("S5-U08", 1 == count_ev(APP_EVT_NO_PATH) && APP_SCAN_STOPPED == g_state && 90u == g_servo, "wide wall -> NO_PATH, STOPPED, servo 90");
  CHECK("S5-U08", 0u != (g_fault & APP_FAULT_RANGE_CLAMP), "clipped leg sets RANGE_CLAMP");
  /* S5-U26: one good echo, then pings with no edges at all */
  g_modes = APP_MODE_RESUME; w_mode = 0; w_dist = 600; cyc(); g_modes = 0; cyc();
  CHECK("S5-U26", 600u == dist(), "good echo: 600 mm");
  w_mode = 2; g_fault = 0;
  { int ok = 1; for (int i = 0; i < 5; i++) { cyc(); if (APP_DIST_NONE_MM != dist()) ok = 0; } CHECK("S5-U26", ok, "5 silent pings all read APP_DIST_NONE_MM, never the cached 600 mm"); }
  CHECK("S5-U26", 0u != (g_fault & APP_FAULT_NO_ECHO), "silent sensor -> FAULT_NO_ECHO");
  /* S5-U24: echo line stuck high: no trigger, silent */
  w_mode = 0; w_dist = 500; cyc(); cyc(); g_fault = 0; g_echo_high = true;
  { int ok = 1; for (int i = 0; i < 5; i++) { cyc(); if (APP_DIST_NONE_MM != dist()) ok = 0; } CHECK("S5-U24", ok, "ECHO held high: no measurement accepted"); }
  CHECK("S5-U24", 0u != (g_fault & APP_FAULT_NO_ECHO), "stuck sensor reported silent -> NO_ECHO");
  g_echo_high = false;
  /* S5-U17: a falling edge outside the window is never used */
  { uint8_t done_before = g_echo_done_cnt;
    obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 100u);
    CHECK("S5-U17", done_before == g_echo_done_cnt, "fall with the window closed is ignored");
    g_echo_window_open = 1u; obstacle_echo_isr(DRV_GPIO_IRQ_RISE, g_now); g_ping_seq++;
    obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 1000u); g_echo_window_open = 0u;
    CHECK("S5-U17", done_before == g_echo_done_cnt, "fall of an older ping (sequence moved on) is ignored");
    g_echo_window_open = 1u; obstacle_echo_isr(DRV_GPIO_IRQ_RISE, g_now);
    obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 45000u); g_echo_window_open = 0u;
    CHECK("S5-U17", done_before == g_echo_done_cnt, "fall older than the 40 ms timeout is ignored");
    /* Fix 5: both edges in one R1 entry are never measured as 0 mm */
    { uint8_t rise_before = g_echo_rise_cnt;
      g_echo_window_open = 1u; g_echo_high = false;
      obstacle_echo_isr(DRV_GPIO_IRQ_BOTH, g_now + 2000u);
      CHECK("S5-U17", done_before == g_echo_done_cnt && rise_before == g_echo_rise_cnt && !gb_is_rise_armed, "RISE+FALL in one entry, pin low: both discarded, no 0 mm echo");
      g_echo_high = true;
      obstacle_echo_isr(DRV_GPIO_IRQ_BOTH, g_now + 3000u);
      CHECK("S5-U17", done_before == g_echo_done_cnt && (uint8_t) (rise_before + 1u) == g_echo_rise_cnt && gb_is_rise_armed && g_now + 3000u == g_rise_us, "RISE+FALL in one entry, pin high: kept as a rise only");
      g_echo_high = false;
      obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 3000u + 580u);
      CHECK("S5-U17", (uint8_t) (done_before + 1u) == g_echo_done_cnt && g_echo.b_is_valid && 100u == obstacle_width_to_mm(g_echo.width_4us), "... and its real fall then measures normally (100 mm)");
      g_echo_window_open = 0u; gb_is_rise_armed = false; } }
  /* lost echoes in APPROACH stop the robot */
  w_mode = 0; w_dist = 350; g_safety = 0; cyc(); cyc(); CHECK("S5-U08", APP_SCAN_APPROACH == g_state, "approach again");
  w_mode = 2; cyc(); cyc(); CHECK("S5-U08", APP_SCAN_STOPPED == g_state && (g_safety & SAFETY_ESTOP), "2 lost echoes in APPROACH -> ESTOP");
  printf("[TEST] S5-U08 obstacle simulation SUMMARY %d passed, %d failed\n", pass_n, fail_n);
  return fail_n;
}
