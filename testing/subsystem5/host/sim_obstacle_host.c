/*
 * sim_obstacle_host.c
 * Laptop simulation of obstacle.c (echo path, confirmation, scanner,
 * profile and plan) against a simple world: a flat face in front of the
 * robot. Checks the event order (ESTOP before OBSTACLE, board before
 * PLAN), the mm conversion, NO_PATH, RANGE_CLAMP and NO_ECHO.
 * Host-only test code, outside BARR-C scope. Run: make -C testing/subsystem5/host
 */
#include <stdio.h>
#include <string.h>
#include "app_program/app.h"
typedef struct { void *exinf; ATR sematr; INT isemcnt; INT maxsem; } T_CSEM;
ID tk_cre_sem(T_CSEM const *p); ER tk_wai_sem(ID id, INT c, TMO t); ER tk_sig_sem(ID id, INT c);
ER tk_slp_tsk(TMO t); ID tk_get_tid(void);
#include "app_program/subsystem5/obstacle.c"

static int sem; static uint32_t g_now, g_safety, g_fault, g_modes, g_servo = 90;
ID tk_cre_sem(T_CSEM const *p) { (void) p; return 3; }
ER tk_wai_sem(ID id, INT c, TMO t) { (void) id; (void) c; (void) t; if (sem > 0) { sem--; return E_OK; } return E_TMOUT; }
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
/* world */
static int w_mode;                 /* 0 = fixed distance, 1 = face, 2 = silent */
static uint32_t w_dist = 1000;     /* mode 0 */
static int32_t w_y = 240, w_xl = -60, w_xr = 80;
static uint32_t world_mm(void) {
  if (0 == w_mode) return w_dist;
  { double th = (double) g_servo * 3.14159265 / 180.0, s = __builtin_sin(th), c = __builtin_cos(th);
    double x = (double) w_y * c / s;   /* + = right (0 deg = right) */
    if (x >= w_xl && x <= w_xr) return (uint32_t) ((double) w_y / s); }
  return 6000;   /* far: long echo */ }
void drv_ultrasonic_trigger(void) {
  uint32_t d, w;
  if (2 == w_mode) return;
  d = world_mm(); w = (uint32_t) ((double) d * 5.831);
  if (w > 38000u) w = 38000u;
  obstacle_echo_isr(DRV_GPIO_IRQ_RISE, g_now + 500u);
  obstacle_echo_isr(DRV_GPIO_IRQ_FALL, g_now + 500u + w); }
/* events */
static uint32_t ev[64], evv[64], evsafety[64], evlegs[64]; static int nev;
bool mission_post_event(uint32_t t, uint32_t v) { status_board_t s; status_board_read(&s);
  ev[nev] = t; evv[nev] = v; evsafety[nev] = g_safety; evlegs[nev] = status_board_leg_lateral(&s); nev++; return true; }
static int pass_n, fail_n;
#define CHECK(c, d) do { if (c) pass_n++; else { fail_n++; printf("FAIL %s (line %d) state=%u servo=%u nev=%d last=%u\n", d, __LINE__, g_state, g_servo, nev, nev ? ev[nev-1] : 99u); } } while (0)
static void cyc(void) { g_now += 60000u; obstacle_cycle(); }
static int count_ev(uint32_t t) { int n = 0; for (int i = 0; i < nev; i++) if (ev[i] == t) n++; return n; }
int main(void) {
  status_board_t snap;
  status_board_init(); obstacle_init();
  w_mode = 0; w_dist = 1000; cyc(); cyc();
  status_board_read(&snap);
  CHECK(APP_SCAN_MONITOR == g_state && 0 == nev, "far object: stay in MONITOR");
  CHECK(status_board_distance(&snap) >= 995 && status_board_distance(&snap) <= 1000, "1000 mm measured as ~1000 mm (mm conversion)");
  w_dist = 300; cyc(); status_board_read(&snap);
  CHECK(status_board_distance(&snap) >= 298 && status_board_distance(&snap) <= 300, "300 mm echo -> ~300 mm");
  CHECK(APP_SCAN_MONITOR == g_state, "one reading is not confirmed");
  w_dist = 290; cyc(); CHECK(APP_SCAN_APPROACH == g_state && 1 == count_ev(APP_EVT_OBSTACLE_AHEAD), "two agreeing readings < 400 -> OBSTACLE_AHEAD");
  w_dist = 300; cyc(); w_dist = 245; cyc(); CHECK(APP_SCAN_APPROACH == g_state, "245 right after 300 is not confirmed (diff 55)");
  w_dist = 240; cyc();
  CHECK(APP_SCAN_STOPPED == g_state && 1 == count_ev(APP_EVT_OBSTACLE), "confirmed < 251 -> OBSTACLE, STOPPED");
  CHECK((evsafety[nev - 1] & SAFETY_ESTOP) != 0u, "ESTOP was set before OBSTACLE was posted");
  cyc(); cyc(); CHECK(APP_SCAN_STOPPED == g_state, "STOPPED waits for RESCAN");
  g_modes = APP_MODE_STOPPED | APP_MODE_RESCAN; w_mode = 1;
  cyc(); CHECK(APP_SCAN_COARSE == g_state && 30u == g_servo, "RESCAN rise -> COARSE at 30 deg");
  for (int i = 0; i < 120 && (APP_SCAN_COARSE == g_state || APP_SCAN_FINE == g_state); i++) cyc();
  CHECK(APP_SCAN_BYPASS_WATCH == g_state, "scan finished -> BYPASS_WATCH");
  CHECK(1 == count_ev(APP_EVT_PLAN) && APP_SIDE_LEFT == evv[nev - 1], "PLAN(left): obstacle extends further right");
  CHECK(evlegs[nev - 1] > 0u, "legs were on the board before PLAN was posted");
  status_board_read(&snap);
  printf("   profile: width %u nearest %u legs %u/%u clearL %u clearR %u step %u\n", status_board_width(&snap), status_board_nearest(&snap), status_board_leg_lateral(&snap), status_board_leg_forward(&snap), status_board_clear_left(&snap), status_board_clear_right(&snap), status_board_angle_step(&snap));
  CHECK(status_board_is_profile_valid(&snap) && status_board_width(&snap) >= 100 && status_board_width(&snap) <= 145, "width 106 mm at 5 deg resolution (face -60..+80)");
  CHECK(30u == g_servo, "watch angle 30 deg (obstacle on the robot's right)");
  g_modes = 0; w_mode = 0; w_dist = 200; cyc(); cyc();
  CHECK(0 == count_ev(APP_EVT_SIDE_TOO_CLOSE), "side 200 mm: fine");
  w_dist = 50; cyc(); cyc(); cyc(); CHECK(1 == count_ev(APP_EVT_SIDE_TOO_CLOSE), "side 50 mm: SIDE_TOO_CLOSE once");
  g_modes = APP_MODE_RESUME; cyc(); CHECK(APP_SCAN_MONITOR == g_state && 90u == g_servo, "RESUME rise -> MONITOR, servo 90");
  g_modes = 0; w_dist = 1000; cyc(); cyc();
  /* NO_PATH scan: wall everywhere */
  w_dist = 380; cyc(); cyc(); w_dist = 240; cyc(); cyc(); CHECK(APP_SCAN_STOPPED == g_state, "second obstacle -> STOPPED");
  g_modes = APP_MODE_STOPPED | APP_MODE_RESCAN; w_mode = 1; w_y = 200; w_xl = -400; w_xr = 400;
  for (int i = 0; i < 120 && APP_SCAN_BYPASS_WATCH != g_state && (i < 2 || APP_SCAN_STOPPED != g_state); i++) cyc();
  CHECK(1 == count_ev(APP_EVT_NO_PATH) && APP_SCAN_STOPPED == g_state, "wide wall -> NO_PATH, back to STOPPED");
  CHECK(0u != (g_fault & APP_FAULT_RANGE_CLAMP), "clipped leg sets RANGE_CLAMP");
  /* silent sensor */
  g_modes = APP_MODE_RESUME; cyc(); g_modes = 0; w_mode = 2; cyc(); cyc();
  CHECK(0u == (g_fault & APP_FAULT_NO_ECHO), "2 silent pings: no fault yet");
  cyc(); CHECK(0u != (g_fault & APP_FAULT_NO_ECHO), "3 silent pings -> FAULT_NO_ECHO");
  /* lost echoes in APPROACH stop the robot */
  w_mode = 0; w_dist = 350; g_safety = 0; cyc(); cyc(); CHECK(APP_SCAN_APPROACH == g_state, "approach again");
  w_mode = 2; cyc(); cyc(); CHECK(APP_SCAN_STOPPED == g_state && (g_safety & SAFETY_ESTOP), "2 lost echoes in APPROACH -> ESTOP");
  printf("[TEST] S5-U08 obstacle simulation SUMMARY %d passed, %d failed\n", pass_n, fail_n);
  return fail_n;
}
