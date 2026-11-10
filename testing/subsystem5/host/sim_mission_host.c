/*
 * sim_mission_host.c
 * Laptop simulation of mission.c + mission_run.c (the statechart, spec
 * v1.5.1) with mocked neighbours: kernel, motion (tagged commands),
 * safety, fault, telem, odometry, sys_health. Drives whole runs and
 * checks every motion command, mode bit and transition, including the
 * host parts of S5-U15, U16, U20, U21, U22 and U27, plus S5-U04 (K4
 * flood) and S5-U05 (state x event matrix), moved here from the target
 * because they changed live state (fix brief, Fix 2/3).
 * Host-only test code, outside BARR-C scope. Run: make -C testing/subsystem5/host
 */
#include <stdio.h>
#include <string.h>
#include "app_program/subsystem5/mission.c"
#include "app_program/subsystem2/safety.h"

/* ---------------- kernel mocks ---------------- */
static UINT flg[8]; static int nflg = 1;
ID tk_cre_flg(T_CFLG const *p) { flg[nflg] = p->iflgptn; return nflg++; }
ER tk_set_flg(ID id, UINT p) { flg[id] |= p; return E_OK; }
ER tk_clr_flg(ID id, UINT p) { flg[id] &= p; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *r) { r->flgptn = flg[id]; return E_OK; }
/* K4 mock with the kernel's sizing (kernel/tkernel/messagebuf.h): each
   message takes HEADERSZ (4) + ROUNDSZ(msgsz) bytes of bufsz; a send fits
   only if HEADERSZ + msgsz <= free bytes. TMO_POL never waits. */
static mission_evt_t q[64]; static int qn = 0;
static int mbf_bufsz, mbf_used, mbf_waits;
#define MBF_ENTRY(sz) (4 + (((sz) + 3) & ~3))
ID tk_cre_mbf(T_CMBF const *p) { mbf_bufsz = p->bufsz; mbf_used = 0; return 1; }
ER tk_snd_mbf(ID id, void const *m, INT sz, TMO t) { (void) id;
  if (TMO_POL != t) { mbf_waits++; }
  if (4 + sz > mbf_bufsz - mbf_used) { return E_TMOUT; }
  memcpy(&q[qn++], m, sizeof(mission_evt_t)); mbf_used += MBF_ENTRY(sz); return E_OK; }
INT tk_rcv_mbf(ID id, void *m, TMO t) { (void) id; (void) t; if (!qn) return E_TMOUT;
  memcpy(m, &q[0], sizeof(mission_evt_t)); memmove(q, q + 1, (size_t)(--qn) * sizeof(q[0]));
  mbf_used -= MBF_ENTRY(12); return 12; }
ER tk_ref_mbf(ID id, T_RMBF *r) { (void) id; r->frbufsz = mbf_bufsz - mbf_used; return E_OK; }
ID tk_cre_mtx(T_CMTX const *p) { (void) p; return 1; }
ER tk_loc_mtx(ID id, TMO t) { (void) id; (void) t; return E_OK; }
ER tk_unl_mtx(ID id) { (void) id; return E_OK; }
ID app_start_task(FP e, PRI p, uint64_t *s, uint32_t b, char const *n) { (void)e;(void)p;(void)s;(void)b;(void)n; return 1; }
void app_check(ER e, char const *w) { (void) e; (void) w; }
static char clog[4096];   /* console capture */
void app_log(char const *f, int32_t a, int32_t b, int32_t c) { char line[256]; snprintf(line, sizeof(line), f, a, b, c); printf("   log: %s", line);
  if (strlen(clog) + strlen(line) < sizeof(clog)) strcat(clog, line); }
/* ---------------- neighbour mocks ---------------- */
static uint32_t g_safety, g_fault, g_now, g_tag; static int g_refuse;
void safety_set(uint32_t b) { g_safety |= b; }
void safety_clear(uint32_t b) { g_safety &= ~b; }
uint32_t safety_get(void) { return g_safety; }
void fault_set(uint32_t b) { g_fault |= b; }
void fault_clear(uint32_t b) { g_fault &= ~b; }
uint32_t fault_get(void) { return g_fault; }
static char mlog[256], alog[1024];   /* this pass / since last clear */
/* Motion model for the run-start ZERO (S5-U33), as motion_task does it:
   a ZERO received while a safety bit is set (ESTOP state) is only
   latched; it is started, and its MOVE_DONE posted, on a later 10 ms
   cycle with no safety bit set; a STOP drops the latch. */
static bool g_zero_latched;
static bool mcmd(char const *s, int32_t v, bool tagged) {
  if (!strcmp(s, "ZERO") && !(tagged && g_refuse > 0)) { g_zero_latched = true; }
  if (!strcmp(s, "STOP")) { g_zero_latched = false; }
  if (strlen(alog) < 1000) { sprintf(alog + strlen(alog), "%s%d ", s, (int) v); }
  if (tagged && g_refuse > 0) { g_refuse--; sprintf(mlog + strlen(mlog), "REFUSED "); return false; }
  if (tagged) g_tag = (g_tag + 1u) & 3u;
  sprintf(mlog + strlen(mlog), "%s%d ", s, (int) v); return true; }
bool motion_move_forward(int32_t d) { return mcmd("F", d, true); }
bool motion_move_backward(int32_t d) { return mcmd("B", d, true); }
bool motion_turn_left(int32_t a) { return mcmd("L", a, true); }
bool motion_turn_right(int32_t a) { return mcmd("R", a, true); }
bool motion_reset_odometry(void) { return mcmd("ZERO", 0, true); }
uint32_t motion_last_tag(void) { return g_tag; }
bool motion_follow(int32_t s) { return mcmd("FOL", s, false); }
bool motion_stop(void) { return mcmd("STOP", 0, false); }
/* Same constant and rounding as odometry.c (A4: 324 um per count). */
uint32_t odometry_counts_to_mm(uint32_t c) { return ((c * APP_UM_PER_COUNT) + 500u) / 1000u; }
uint32_t odometry_mm_to_counts(uint32_t d) { return ((d * 1000u) + (APP_UM_PER_COUNT / 2u)) / APP_UM_PER_COUNT; }
int32_t odometry_heading_cdeg(int32_t l, int32_t r) { return (l - r) * 16; }
static char tlog[512];
bool telem_post_event(uint32_t t, uint32_t v) { if (strlen(tlog) < 400) sprintf(tlog + strlen(tlog), "E%u:%u ", t, v); return true; }
bool telem_post_profile(telem_bin_profile_t const *p) { sprintf(tlog + strlen(tlog), "PROF%u ", p->side); return true; }
bool telem_post_summary(telem_bin_summary_t const *p) { sprintf(tlog + strlen(tlog), "SUMM(end%u odo%u) ", p->end_reason, p->odometry_mm); return true; }
static telem_bin_snapshot_t g_snapshot; static int g_snapshots;
bool telem_post_snapshot(telem_bin_snapshot_t const *p) { g_snapshot = *p; g_snapshots++; if (strlen(tlog) < 400) sprintf(tlog + strlen(tlog), "SNAP(sys%u odo%u s%u) ", p->sys_state, p->odometry_mm, p->run_time_s); return true; }
static uint32_t g_map_char, g_map_cmd;
static bool g_map_stored = true;
bool barcode_map_set(uint32_t c, uint32_t m) {
  g_map_char = c;
  g_map_cmd = m;
  return g_map_stored;
}
void sys_health_service(void) {}
void imu_diag_service(void) {}
static uint32_t g_barcode_edges, g_barcode_level;
uint32_t barcode_edge_count(void) { return g_barcode_edges; }
uint32_t barcode_do_level(void) { return g_barcode_level; }
uint32_t sys_health_feed_gap_max_us(bool b) { (void) b; return 0u; }
void sys_tick_register_feeder(ID t) { (void) t; }
ER tk_slp_tsk(TMO t) { (void) t; return E_OK; }
ID tk_get_tid(void) { return 1; }
static int g_wdt_enables; static bool g_started;
void sys_health_enable_watchdog(void) { g_wdt_enables++; }
bool sys_health_is_started(void) { return g_started; }
uint32_t sys_tick_now_us(void) { return g_now; }
bool hmi_init(void) { return true; }
uint32_t hmi_poll(void) { return 0u; }
void hmi_show(uint32_t s, uint32_t n) { (void) s; (void) n; }
static bool g_warn; void hmi_set_warning(bool b) { g_warn = b; }
/* S5-U05: every matrix case starts from fresh mocks (Fix 2) */
void mission_host_case_reset(void) {
  memset(flg, 0, sizeof(flg)); qn = 0; mbf_used = 0; g_safety = 0; g_fault = 0;
  g_refuse = 0; g_wdt_enables = 0; g_started = false; g_warn = false;
  mlog[0] = 0; alog[0] = 0; tlog[0] = 0; }
/* ---------------- harness ---------------- */
static int pass_n, fail_n;
#define CHECK(id, c, d) do { if (c) { pass_n++; printf("[TEST] %s PASS  %s\n", id, d); } else { fail_n++; printf("[TEST] %s FAIL  %s (line %d) mlog='%s' sys=%u run=%u av=%u sub=%u pend=%d\n", id, d, __LINE__, mlog, g_ctx.sys_state, g_ctx.run_state, g_ctx.avoid_step, g_ctx.substep, g_ctx.b_move_pending); } } while (0)
static status_board_motion_t mot; static status_board_obs_t obs;
static void step(uint32_t type, uint32_t value, bool has) {
  mission_evt_in_t e = {has, type, value}, b = {false, 0, 0}; status_board_t snap;
  g_now += 10000u; status_board_write_motion(&mot); status_board_write_obstacle(&obs);
  status_board_read(&snap); mlog[0] = 0; mission_pass(&g_ctx, &e, &b, &snap, g_now); }
#define EV(t, v) step((t), (v), true)
#define DONE() step(APP_EVT_MOVE_DONE, g_tag, true)
#define NONE() step(0, 0, false)
#define MODE(b) (0u != (flg[1] & (b)))
static void idle_ms(uint32_t ms) { for (uint32_t i = 0; i < ms / 10u; i++) NONE(); }
static void to_run(void) {          /* READY -> RUN through the run-start handshake */
  EV(APP_EVT_START, 0); idle_ms(1000); NONE(); DONE(); idle_ms(30); }
/* S5-U33: one pass in which the motion model may acknowledge the ZERO.
   Nothing is injected: MOVE_DONE comes only if the model would send it. */
static int g_model_acks;
static void mpass(void) {
  if (g_zero_latched && (0u == g_safety)) { g_zero_latched = false; g_model_acks++; DONE(); }
  else { NONE(); } }
static void mpass_ms(uint32_t ms) { for (uint32_t i = 0; i < ms / 10u; i++) mpass(); }
/* Back to a clean READY (recalibration through the real sweep). */
static void to_ready(void) {
  status_board_t s;
  g_fault = 0; g_safety = 0; g_zero_latched = false; EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  status_board_read(&s); mission_enter(&g_ctx, APP_SYS_CAL_IMU, &s, g_now);
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR); }

/* Track the mock odometry through the 35 mm prefix and 5 mm brake travel. */
static void barcode_test_follow(uint32_t path_mm) {
  status_board_line_t const line = {0, APP_LINE_ON, APP_JCT_NONE, true};
  tlog[0] = 0;
  if (APP_SYS_RUN == g_ctx.sys_state) {
    EV(APP_EVT_STOP, 0u);   /* RESET from FINISHED clears all mode bits */
  }
  to_ready();
  mot.motion_state = APP_MOT_BRAKE;
  mot.speed_l_cpw = mot.speed_r_cpw = 0;
  obs.scanner_state = APP_SCAN_MONITOR;
  obs.distance_mm = APP_DIST_NONE_MM;
  mot.path_counts = 0u;     /* model the run-start ZERO */
  g_barcode_edges = g_barcode_level = 0u;
  to_run();
  status_board_write_line(&line);
  mot.motion_state = APP_MOT_FOLLOW;
  mot.speed_l_cpw = mot.speed_r_cpw = 18;
  mot.path_counts = odometry_mm_to_counts(path_mm);
}

static void barcode_test_brake(void) {
  mot.motion_state = APP_MOT_BRAKE;
  mot.speed_l_cpw = mot.speed_r_cpw = 0;
  mot.path_counts = odometry_mm_to_counts(240u);
  NONE();
}

static void barcode_test_back(uint32_t path_mm, uint32_t level,
                              uint32_t new_edges) {
  mot.motion_state = APP_MOT_MOVE_BACK;
  mot.speed_l_cpw = mot.speed_r_cpw = -15;
  mot.path_counts = odometry_mm_to_counts(path_mm);
  g_barcode_level = level;
  g_barcode_edges += new_edges;
  NONE();
}

static void barcode_test_still(void) {
  mot.motion_state = APP_MOT_BRAKE;
  mot.speed_l_cpw = mot.speed_r_cpw = 0;
  NONE();
}

static void barcode_test_read(void) {
  barcode_test_follow(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  barcode_test_brake();
  barcode_test_back(312u, 1u, 6u);
  barcode_test_back(335u, 0u, 1u);
  barcode_test_back(390u, 0u, 0u);
  DONE();
  barcode_test_still();
}

int main(void) {
  status_board_t snap;
  status_board_init(); mission_init();
  mission_ctx_reset(&g_ctx); status_board_read(&snap);
  flg[1] = APP_MODE_STOPPED;          /* left over: BOOT must clear it */
  mission_enter(&g_ctx, APP_SYS_BOOT, &snap, g_now);
  CHECK("S5-U07", 1 == g_wdt_enables && !MODE(APP_MODE_STOPPED) && MODE(APP_MODE_RESUME), "BOOT: watchdog startup window, modes cleared, RESUME pulse");
  NONE(); NONE(); CHECK("S5-U07", APP_SYS_BOOT == g_ctx.sys_state, "BOOT waits for the first check-ins");
  g_started = true; NONE(); CHECK("S5-U07", APP_SYS_CAL_IMU == g_ctx.sys_state && MODE(APP_MODE_CAL_IMU), "started -> CAL_IMU");
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); CHECK("S5-U07", APP_SYS_CAL_IR == g_ctx.sys_state && MODE(APP_MODE_CAL_IR) && MODE(APP_MODE_MOTOR_EN) && !strcmp(mlog, "L30 "), "CAL_IR: CAL_IR|MOTOR_EN, sweep left 30");
  EV(APP_EVT_MOVE_DONE, (g_tag + 3u) & 3u); CHECK("S5-U07", !strcmp(mlog, "") && 0u == g_ctx.substep, "MOVE_DONE with a stale tag is ignored");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R60 "), "tagged MOVE_DONE -> right 60");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "L30 "), "left 30");
  DONE(); CHECK("S5-U07", !MODE(APP_MODE_CAL_IR) && !MODE(APP_MODE_MOTOR_EN), "sweep done clears CAL_IR and MOTOR_EN");
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR); CHECK("S5-U07", APP_SYS_READY == g_ctx.sys_state, "both calibrations valid -> READY");
  EV(APP_EVT_START, 0); CHECK("S5-U07", g_warn, "START: 1 s warning");
  idle_ms(990); CHECK("S5-U07", APP_SYS_READY == g_ctx.sys_state && !MODE(APP_MODE_RUN_START), "still warning");
  alog[0] = 0; idle_ms(20); CHECK("S5-U20", MODE(APP_MODE_RUN_START) && strstr(alog, "ZERO0") && g_ctx.b_move_pending, "run start: RUN_START set, ZERO sent");
  DONE(); CHECK("S5-U20", APP_SYS_READY == g_ctx.sys_state, "ZERO acked, RUN_START held for 30 ms");
  alog[0] = 0; idle_ms(30); CHECK("S5-U20", APP_SYS_RUN == g_ctx.sys_state && !MODE(APP_MODE_RUN_START) && MODE(APP_MODE_MOTOR_EN) && 0u == g_ctx.run_start_path && !strcmp(alog, "FOL120 "), "-> RUN: RUN_START cleared, MOTOR_EN set, path 0, FOLLOW cruise");
  /* barcode turn */
  EV(APP_EVT_BARCODE, (1u << 6) | 11u); CHECK("S5-U07", APP_RUN_ARMED == g_ctx.run_state && !strcmp(mlog, "FOL120 "), "BARCODE(RIGHT) -> ARMED at cruise");
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS); CHECK("S5-U07", !strcmp(mlog, "STOP0 F60 "), "CROSS -> stop, advance 60 (tracked)");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R90 "), "turn right 90");
  DONE(); CHECK("S5-U07", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "FOL120 ") && !g_ctx.b_move_pending, "turn done -> FOLLOW cruise; FOLLOW is never tracked");
  /* S5-U15: search sends STOP first */
  EV(APP_EVT_LINE_LOST, 0); CHECK("S5-U15", APP_RUN_SEARCH_LINE == g_ctx.run_state && !strcmp(mlog, "STOP0 L60 "), "LINE_LOST -> STOP first, then turn left 60");
  DONE(); CHECK("S5-U15", !strcmp(mlog, "R120 "), "next leg on the tagged MOVE_DONE: right 120");
  EV(APP_EVT_LINE_FOUND, 0); CHECK("S5-U15", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "STOP0 FOL120 ") && !g_ctx.b_move_pending, "LINE_FOUND during a leg -> FOLLOW, pending cleared");
  /* S5-U36 (on-car 2026-11-03): the search sweeps first toward the side the line was last seen (steering < 0 = right) */
  { status_board_line_t ln; memset(&ln, 0, sizeof(ln)); ln.steering = -350; ln.line_status = APP_LINE_LOST; ln.b_is_calibrated = true; status_board_write_line(&ln);
    EV(APP_EVT_LINE_LOST, 0); CHECK("S5-U36", APP_RUN_SEARCH_LINE == g_ctx.run_state && !strcmp(mlog, "STOP0 R60 "), "line last seen right -> STOP, turn right 60 first");
    DONE(); CHECK("S5-U36", !strcmp(mlog, "L120 "), "then left 120");
    DONE(); CHECK("S5-U36", !strcmp(mlog, "R60 "), "then right 60");
    ln.steering = 0; ln.line_status = APP_LINE_ON; status_board_write_line(&ln);
    EV(APP_EVT_LINE_FOUND, 0); CHECK("S5-U36", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "STOP0 FOL120 "), "found -> FOLLOW"); }
  /* obstacle + bypass */
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; mot.pos_l_counts = 1000; mot.pos_r_counts = 900;
  EV(APP_EVT_OBSTACLE, 0); NONE(); CHECK("S5-U07", APP_RUN_AVOID == g_ctx.run_state && MODE(APP_MODE_STOPPED | APP_MODE_RESCAN), "OBSTACLE -> AVOID, standstill -> scan request");
  idle_ms(200); obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 176u; obs.leg_forward_mm = 632u;
  obs.width_mm = 140u; obs.nearest_mm = 380u; clog[0] = 0;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); CHECK("S5-U07", !(g_safety & SAFETY_ESTOP) && !strcmp(mlog, "L90 ") && strstr(tlog, "PROF0"), "PLAN -> ESTOP off, profile, turn left 90");
  CHECK("S5-U40", NULL != strstr(clog, "OBSTACLE width 140 mm, nearest 380 mm ahead, pass side 0"), "scanned width printed on the console with the MQTT profile");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "F176 "), "lateral leg 176");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R90 "), "turn parallel");
  DONE(); CHECK("S5-U27", !strcmp(mlog, "F632 ") && g_ctx.move_limit_ms >= 7000u, "pass leg 632 mm; deadline long enough for it");
  idle_ms(5000); CHECK("S5-U27", APP_SYS_RUN == g_ctx.sys_state, "a healthy 632 mm leg is not faulted at 5 s");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R45 ") && g_ctx.b_fr9_running, "turn 45 toward the line, FR9 starts");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "F297 "), "rejoin move e x 1.2");
  mot.path_counts += odometry_mm_to_counts(250u);
  EV(APP_EVT_LINE_FOUND, 24u); CHECK("S5-U07", !strcmp(mlog, "STOP0 F60 "), "LINE_FOUND in the window -> advance 60");
  mot.pos_l_counts = 1050;
  DONE(); CHECK("S5-U07", !strcmp(mlog, "L8 "), "heading +8 deg -> turn left 8");
  DONE(); CHECK("S5-U07", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "FOL100 ") && MODE(APP_MODE_RESUME), "bypass done -> FOLLOW slow, RESUME");
  idle_ms(160); CHECK("S5-U07", !MODE(APP_MODE_RESUME), "RESUME cleared after 150 ms");
  mot.path_counts += odometry_mm_to_counts(151u); NONE(); CHECK("S5-U07", !MODE(APP_MODE_JUNCTION_INHIBIT) && !strcmp(mlog, "FOL120 "), "inhibits cleared 150 mm later, cruise");
  /* S5-U21: STOP during WAIT_PLAN leaves ESTOP set; recovery clears it */
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP;
  EV(APP_EVT_OBSTACLE, 0); NONE(); EV(APP_EVT_STOP, 0);
  CHECK("S5-U21", APP_SYS_FINISHED == g_ctx.sys_state && !MODE(APP_MODE_MOTOR_EN) && (g_safety & SAFETY_ESTOP), "STOP in WAIT_PLAN -> FINISHED, MOTOR_EN off, ESTOP still set");
  EV(APP_EVT_RESET, 0); CHECK("S5-U21", 0u == g_safety && MODE(APP_MODE_RESUME) && g_ctx.b_recovering, "RESET: common recovery clears ESTOP, pulses RESUME");
  idle_ms(90); CHECK("S5-U21", APP_SYS_FINISHED == g_ctx.sys_state, "recovery waits 100 ms (mission keeps looping)");
  idle_ms(20); CHECK("S5-U21", APP_SYS_READY == g_ctx.sys_state, "-> READY");
  mot.motion_state = APP_MOT_BRAKE; to_run(); CHECK("S5-U21", APP_SYS_RUN == g_ctx.sys_state, "the next run starts");
  /* SIDE_TOO_CLOSE on a leg -> FAULT */
  safety_set(SAFETY_ESTOP); EV(APP_EVT_OBSTACLE, 0); NONE(); idle_ms(200);
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); DONE(); safety_set(SAFETY_ESTOP);
  EV(APP_EVT_SIDE_TOO_CLOSE, 0); CHECK("S5-U07", APP_SYS_FAULT == g_ctx.sys_state && strstr(tlog, "E14:0"), "SIDE_TOO_CLOSE on a leg -> FAULT, forwarded to telemetry");
  /* S5-U16: FAULT RESET goes through BOOT and recalibration */
  EV(APP_EVT_RESET, 0); idle_ms(110); CHECK("S5-U16", APP_SYS_BOOT == g_ctx.sys_state || APP_SYS_CAL_IMU == g_ctx.sys_state, "RESET from FAULT -> BOOT (recalibration), not READY");
  CHECK("S5-U16", 1 == g_wdt_enables, "watchdog startup window only on the first boot");
  NONE(); CHECK("S5-U16", APP_SYS_CAL_IMU == g_ctx.sys_state && !g_ctx.b_imu_valid, "recalibration starts, IMU not valid yet");
  /* S5-U32 (on-car log 2026-10-29): a bump while the car is handled
     latches IMPACT with the motors off; it must not end the next moving
     state in FAULT. mission clears it when the motors are enabled. */
  safety_set(SAFETY_IMPACT);                 /* bumped while being placed */
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); NONE();
  CHECK("S5-U32", APP_SYS_CAL_IR == g_ctx.sys_state && 0u == (g_safety & SAFETY_IMPACT),
        "IMPACT latched in CAL_IMU is cleared on entering CAL_IR: sweep runs, no FAULT");
  DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  CHECK("S5-U16", APP_SYS_READY == g_ctx.sys_state, "recalibrated -> READY");
  /* S5-U32: a second, real impact DURING the sweep still ends in FAULT */
  g_fault = 0; g_safety = 0; status_board_read(&snap);
  mission_enter(&g_ctx, APP_SYS_CAL_IMU, &snap, g_now);
  safety_set(SAFETY_IMPACT);                 /* stale bump, cleared at entry */
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); NONE();
  safety_set(SAFETY_IMPACT); NONE();         /* new impact while sweeping */
  CHECK("S5-U32", APP_SYS_FAULT == g_ctx.sys_state,
        "impact during the CAL_IR sweep (after the stale one was cleared) -> FAULT");
  /* S5-U32: a set ESTOP is never cleared by the motor-enable step */
  g_fault = 0; g_safety = 0; EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  status_board_read(&snap); mission_enter(&g_ctx, APP_SYS_CAL_IMU, &snap, g_now);
  safety_set(SAFETY_ESTOP);
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); NONE();
  CHECK("S5-U32", 0u != (g_safety & SAFETY_ESTOP),
        "ESTOP set before CAL_IR is kept (hazard not proven gone)");
  g_fault = 0; g_safety = 0; EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  status_board_read(&snap); mission_enter(&g_ctx, APP_SYS_CAL_IMU, &snap, g_now);
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  CHECK("S5-U32", APP_SYS_READY == g_ctx.sys_state, "clean recalibration -> READY");
  safety_set(SAFETY_IMPACT);                 /* bumped while RUN is pressed */
  g_model_acks = 0; EV(APP_EVT_START, 0); mpass_ms(1250);
  CHECK("S5-U32", APP_SYS_RUN == g_ctx.sys_state && 0u == (g_safety & SAFETY_IMPACT) && 1 == g_model_acks,
        "IMPACT latched in READY is cleared before the ZERO; the motion model acks it: RUN, no FAULT");
  /* S5-U16: IMPACT during RUN -> FAULT (still stops the car) */
  safety_set(SAFETY_IMPACT); NONE(); CHECK("S5-U16", APP_SYS_FAULT == g_ctx.sys_state, "IMPACT during FOLLOW -> FAULT");
  /* persistent fault comes back during recovery -> stay in FAULT */
  g_safety = 0; EV(APP_EVT_RESET, 0); fault_set(APP_FAULT_PIO_LOAD); idle_ms(110);
  CHECK("S5-U18", APP_SYS_FAULT == g_ctx.sys_state && (g_safety & SAFETY_FAULT), "stop fault set again during recovery -> stay in FAULT");
  fault_clear(APP_FAULT_ALL); EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  CHECK("S5-U07", APP_SYS_READY == g_ctx.sys_state, "clean RESET -> BOOT -> READY");
  /* S5-U22: a refused command is retried once, twice -> FAULT */
  to_run(); g_refuse = 1; EV(APP_EVT_LINE_LOST, 0);
  CHECK("S5-U22", strstr(mlog, "REFUSED") && g_ctx.b_retry_pending, "first refusal: retry pending");
  NONE(); CHECK("S5-U22", !strcmp(mlog, "L60 ") && g_ctx.b_move_pending, "retried on the next pass and accepted");
  EV(APP_EVT_LINE_FOUND, 0); g_refuse = 2; EV(APP_EVT_LINE_LOST, 0); NONE();
  CHECK("S5-U22", APP_SYS_FAULT == g_ctx.sys_state && (g_fault & APP_FAULT_MOTOR_STALL), "refused twice -> MOTOR_STALL, FAULT");
  /* S5-U22: ZERO never acknowledged -> FAULT after 200 ms */
  g_fault = 0; g_safety = 0; EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  EV(APP_EVT_START, 0); idle_ms(1010); CHECK("S5-U22", MODE(APP_MODE_RUN_START), "run start waits for ZERO");
  idle_ms(170); CHECK("S5-U22", APP_SYS_READY == g_ctx.sys_state, "still waiting 180 ms after ZERO");
  idle_ms(30); CHECK("S5-U22", APP_SYS_FAULT == g_ctx.sys_state, "no ZERO acknowledgement -> FAULT after 200 ms");
  /* S5-U33: run start against the motion model (no injected MOVE_DONE) */
  to_ready(); CHECK("S5-U33", APP_SYS_READY == g_ctx.sys_state, "clean READY");
  g_model_acks = 0; EV(APP_EVT_START, 0); mpass_ms(500);
  safety_set(SAFETY_IMPACT);                 /* bump during the 1 s warning */
  alog[0] = 0; mpass_ms(510);
  CHECK("S5-U33", MODE(APP_MODE_RUN_START) && strstr(alog, "ZERO0") && 0u == g_safety && 1 == g_model_acks,
        "IMPACT during the warning: cleared before ZERO, the model acknowledges it");
  mpass_ms(40);
  CHECK("S5-U33", APP_SYS_RUN == g_ctx.sys_state && MODE(APP_MODE_MOTOR_EN), "-> RUN");
  /* a fresh impact while the ZERO is pending -> FAULT IMPACT at once */
  to_ready(); g_model_acks = 0; EV(APP_EVT_START, 0); mpass_ms(1000);
  while (!g_ctx.b_run_starting) { mpass(); }  /* the pass that sent ZERO */
  CHECK("S5-U33", g_zero_latched && g_ctx.b_move_pending, "ZERO sent, not yet acknowledged");
  safety_set(SAFETY_IMPACT); mpass();
  CHECK("S5-U33", APP_SYS_FAULT == g_ctx.sys_state && 0 == g_model_acks && !g_zero_latched,
        "fresh IMPACT with the ZERO pending -> FAULT IMPACT now (not the 200 ms deadline); STOP drops the ZERO");
  CHECK("S5-U33", !MODE(APP_MODE_MOTOR_EN), "motors never enabled");
  /* a fresh impact after the ack, in the 30 ms hold -> FAULT, not cleared */
  to_ready(); g_model_acks = 0; EV(APP_EVT_START, 0);
  while (0 == g_model_acks) { mpass(); }
  CHECK("S5-U33", APP_SYS_READY == g_ctx.sys_state && g_ctx.b_zero_done, "ZERO acknowledged, holding RUN_START");
  safety_set(SAFETY_IMPACT); mpass();
  CHECK("S5-U33", APP_SYS_FAULT == g_ctx.sys_state && !MODE(APP_MODE_MOTOR_EN),
        "fresh IMPACT in the run-start hold -> FAULT, motors never enabled");
  /* S5-U27: deadline helper */
  CHECK("S5-U27", mission_move_deadline_ms(MISSION_MV_FWD, 800) >= 8900u && mission_move_deadline_ms(MISSION_MV_FWD, 800) <= 9100u, "800 mm at 150 mm/s -> ~9 000 ms");
  CHECK("S5-U27", mission_move_deadline_ms(MISSION_MV_FWD, 2047) > 20000u, "2 047 mm move gets > 20 s");
  CHECK("S5-U27", 200u == mission_move_deadline_ms(MISSION_MV_ZERO, 0), "ZERO: 200 ms");
  CHECK("S5-U27", mission_move_deadline_ms(MISSION_MV_BACK, 150) > mission_move_deadline_ms(MISSION_MV_FWD, 150), "reverse (100 mm/s) gets longer than forward");
  /* blocked wheel: deadline passes -> FAULT */
  g_fault = 0; g_safety = 0; EV(APP_EVT_RESET, 0); idle_ms(110); NONE();
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  to_run(); EV(APP_EVT_LINE_LOST, 0); idle_ms(1500);
  CHECK("S5-U27", APP_SYS_RUN == g_ctx.sys_state, "60 deg turn (deadline ~1.65 s) still in time at 1.5 s");
  idle_ms(300); CHECK("S5-U27", APP_SYS_FAULT == g_ctx.sys_state, "a turn that never completes (blocked wheel) ends in FAULT at its deadline");
  /* Wi-Fi loss is not a transition */
  g_fault = 0; g_safety = 0; EV(APP_EVT_RESET, 0); idle_ms(110); fault_set(APP_FAULT_WIFI_LOSS); NONE();
  CHECK("S5-U07", APP_SYS_FAULT != g_ctx.sys_state, "Wi-Fi loss is not a transition");
  /* S1-U41: remote "summary now" (APP_EVT_SNAPSHOT) answers in every state and changes nothing */
  to_ready(); g_snapshots = 0; EV(APP_EVT_SNAPSHOT, 0);
  CHECK("S1-U41", 1 == g_snapshots && APP_SYS_READY == g_snapshot.sys_state && 0u == g_snapshot.odometry_mm && 0u == g_snapshot.run_time_s && APP_SYS_READY == g_ctx.sys_state, "READY, no run yet: snapshot with 0 mm, 0 s; state unchanged");
  mot.path_counts = 0u; to_run(); mot.path_counts += odometry_mm_to_counts(500u); idle_ms(2000);   /* path 0: the run-start ZERO */ mlog[0] = 0; EV(APP_EVT_SNAPSHOT, 0);
  CHECK("S1-U41", 2 == g_snapshots && APP_SYS_RUN == g_snapshot.sys_state && APP_RUN_FOLLOW == g_snapshot.run_state && 500u == g_snapshot.odometry_mm && 2u == g_snapshot.run_time_s && APP_SYS_RUN == g_ctx.sys_state && !strcmp(mlog, ""), "RUN: distance and time so far (500 mm, 2 s), no motion command, still RUN");
  EV(APP_EVT_STOP, 0); mot.path_counts += odometry_mm_to_counts(300u); idle_ms(3000); EV(APP_EVT_SNAPSHOT, 0);
  CHECK("S1-U41", 3 == g_snapshots && APP_SYS_FINISHED == g_snapshot.sys_state && 500u == g_snapshot.odometry_mm && 2u == g_snapshot.run_time_s, "FINISHED: distance and time frozen at the end of the run");
  fault_set(APP_FAULT_RANGE_CLAMP); EV(APP_EVT_SNAPSHOT, 0);
  CHECK("S1-U41", 4 == g_snapshots && 0u != (g_snapshot.faults & APP_FAULT_RANGE_CLAMP) && APP_SYS_FINISHED == g_ctx.sys_state, "faults reported; FINISHED kept");
  /* S5-U39 (Dylan 2026-11-04): FR9 bypass limit 5 s -> 20 s. A rejoin
     that takes ~6.3 s from the end of the pass leg (slower car) must
     finish and resume following, not fault REACQUIRE_FAIL at 5 s.     */
  to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0; tlog[0] = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP;
  EV(APP_EVT_OBSTACLE, 0); NONE(); idle_ms(200);
  obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 176u; obs.leg_forward_mm = 632u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); DONE(); DONE(); DONE(); idle_ms(4000);
  DONE(); CHECK("S5-U39", !strcmp(mlog, "R45 ") && g_ctx.b_fr9_running, "pass leg done: FR9 timer starts, turn 45");
  idle_ms(1200); DONE(); CHECK("S5-U39", !strcmp(mlog, "F297 "), "turn took 1.2 s, rejoin drive");
  idle_ms(3000); mot.path_counts += odometry_mm_to_counts(250u);
  EV(APP_EVT_LINE_FOUND, 24u); CHECK("S5-U39", !strcmp(mlog, "STOP0 F60 ") && APP_SYS_RUN == g_ctx.sys_state, "line found after 3 s more (4.2 s): still RUN");
  idle_ms(1200); mot.pos_l_counts = 1050; DONE(); CHECK("S5-U39", APP_SYS_RUN == g_ctx.sys_state && 0u == (g_fault & APP_FAULT_REACQUIRE_FAIL), "5.4 s after the pass leg: no REACQUIRE_FAIL");
  idle_ms(900); DONE(); CHECK("S5-U39", APP_RUN_FOLLOW == g_ctx.run_state && APP_SYS_RUN == g_ctx.sys_state && 0u == (g_fault & APP_FAULT_REACQUIRE_FAIL), "rejoin finished at ~6.3 s -> FOLLOW (no fault)");
  CHECK("S5-U39", 20000u == MISSION_FR9_LIMIT_MS, "FR9 limit is 20 s");
  /* S5-U41 (on-car 2026-11-04 14:16): the rejoin crossing measured 49 mm
     (both sensors OR-ed at 45 deg) and was rejected (want 25 +- 8), so
     the car drove over its line and faulted REACQUIRE_FAIL.           */
  to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP;
  EV(APP_EVT_OBSTACLE, 0); NONE(); idle_ms(200);
  obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 176u; obs.leg_forward_mm = 632u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); DONE(); DONE(); DONE(); DONE(); DONE();
  CHECK("S5-U41", !strcmp(mlog, "F297 "), "rejoin drive started");
  mot.path_counts += odometry_mm_to_counts(250u);
  EV(APP_EVT_LINE_FOUND, 10u); CHECK("S5-U41", !strcmp(mlog, ""), "10 mm (barcode bar) in the window: rejected, keeps driving");
  EV(APP_EVT_LINE_FOUND, 90u); CHECK("S5-U41", !strcmp(mlog, ""), "90 mm (along the line): rejected");
  EV(APP_EVT_LINE_FOUND, 49u); CHECK("S5-U41", !strcmp(mlog, "STOP0 F60 ") && APP_SYS_RUN == g_ctx.sys_state, "49 mm (the car's real crossing): accepted, advance 60");
  DONE(); DONE(); CHECK("S5-U41", APP_RUN_FOLLOW == g_ctx.run_state && 0u == (g_fault & APP_FAULT_REACQUIRE_FAIL), "back to FOLLOW, no REACQUIRE_FAIL");
  /* S5-U43 (on-car 2026-11-06 12:56): leg 288 mm -> line expected at
     407 mm (accept 284..529); the line came at 221 mm, width 38 mm, and
     was rejected as too early, so the car drove over it and on until
     OBSTACLE SIDE TOO CLOSE. The 50..120 % window still accepts 221 mm. */
  to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP;
  EV(APP_EVT_OBSTACLE, 0); NONE(); idle_ms(200);
  obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 288u; obs.leg_forward_mm = 632u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); DONE(); DONE(); DONE(); DONE(); clog[0] = 0; DONE();
  CHECK("S5-U43", !strcmp(mlog, "F488 "), "rejoin drive e x 1.2 = 488 mm");
  CHECK("S5-U43", NULL != strstr(clog, "REJOIN: line expected at 407 mm (accept 203..488 mm)"), "console shows the 50..120 % window");
  mot.path_counts += odometry_mm_to_counts(202u);
  EV(APP_EVT_LINE_FOUND, 38u); CHECK("S5-U43", !strcmp(mlog, ""), "38 mm wide at 202 mm (< 203): still rejected");
  mot.path_counts += odometry_mm_to_counts(19u);
  EV(APP_EVT_LINE_FOUND, 38u); CHECK("S5-U43", !strcmp(mlog, "STOP0 F60 ") && APP_SYS_RUN == g_ctx.sys_state, "38 mm wide at 221 mm (the car's crossing): accepted, advance 60");
  DONE(); DONE(); CHECK("S5-U43", APP_RUN_FOLLOW == g_ctx.run_state && 0u == (g_fault & APP_FAULT_REACQUIRE_FAIL), "back to FOLLOW, no REACQUIRE_FAIL");
  /* S5-U58: reject the early 116 mm and late 511 mm false rejoins.
     A later normal-width crossing still works, as in the 12:27 run. */
  {
    static const struct {
      uint32_t lateral_mm, distances_mm[4], widths_mm[4], count;
      bool should_rejoin;
    } runs[] = {
      {276u, {89u, 116u}, {12u, 17u}, 2u, false},
      {280u, {53u, 89u, 219u, 511u}, {17u, 20u, 14u, 16u}, 4u, false},
      {292u, {54u, 291u, 333u}, {14u, 13u, 26u}, 3u, true},
      {196u, {319u}, {59u}, 1u, true}
    };
    for (size_t i = 0; i < sizeof(runs) / sizeof(runs[0]); ++i) {
      uint32_t start;
      tlog[0] = 0;
      to_ready();
      mot.motion_state = APP_MOT_BRAKE;
      to_run();
      safety_set(SAFETY_ESTOP);
      mot.motion_state = APP_MOT_ESTOP;
      obs.distance_mm = 244u;
      EV(APP_EVT_OBSTACLE, 0);
      NONE();
      idle_ms(200);
      obs.plan_side = APP_SIDE_RIGHT;
      obs.leg_lateral_mm = runs[i].lateral_mm;
      obs.leg_forward_mm = 578u;
      EV(APP_EVT_PLAN, APP_SIDE_RIGHT);
      DONE(); DONE(); DONE(); DONE(); DONE();

      start = mot.path_counts;
      for (uint32_t j = 0u; j < runs[i].count; ++j) {
        bool const accepted = runs[i].should_rejoin && (j + 1u == runs[i].count);
        mot.path_counts = start + odometry_mm_to_counts(runs[i].distances_mm[j]);
        EV(APP_EVT_LINE_FOUND, runs[i].widths_mm[j]);
        CHECK("S5-U58", !strcmp(mlog, accepted ? "STOP0 F60 " : ""),
              accepted ? "accept the later line crossing"
                       : "reject the early, narrow or late crossing");
      }
      DONE();
      if (runs[i].should_rejoin) {
        DONE();
        CHECK("S5-U58", APP_RUN_FOLLOW == g_ctx.run_state,
              "a valid crossing returns to FOLLOW");
      } else {
        CHECK("S5-U58", APP_SYS_FAULT == g_ctx.sys_state
              && 0u != (g_fault & APP_FAULT_REACQUIRE_FAIL),
              "no accepted crossing by move end, stop on REACQUIRE_FAIL");
      }
    }
  }
  /* S5-U54 (track review 2026-11-09): the car stops at ~40 cm
     (safety), then creeps under sonar control to ~OBS_SCAN_MM and scans
     at standstill; a NO_PATH ends in an explicit stop, never a blind
     reverse.                                                          */
  to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; obs.distance_mm = 400u; clog[0] = 0;
  EV(APP_EVT_OBSTACLE, 0); NONE();
  CHECK("S5-U54", !strcmp(mlog, "F150 ") && MODE(APP_MODE_STOPPED) && !MODE(APP_MODE_RESCAN) && !(g_safety & SAFETY_ESTOP),
        "stopped at 400 mm: ESTOP released, creep 150 mm to the 250 mm scan distance, no scan yet");
  CHECK("S5-U54", NULL != strstr(clog, "CREEP: obstacle at 400 mm, moving 150 mm closer to scan"), "creep logged");
  mot.motion_state = APP_MOT_BRAKE; obs.distance_mm = 252u; DONE();
  CHECK("S5-U54", MODE(APP_MODE_STOPPED | APP_MODE_RESCAN) && (g_safety & SAFETY_ESTOP) && !strcmp(mlog, ""),
        "creep done: ESTOP holds the car, scan requested");
  clog[0] = 0; EV(APP_EVT_NO_PATH, 0);
  CHECK("S5-U54", APP_SYS_FAULT == g_ctx.sys_state && NULL == strstr(alog, "B150") && NULL != strstr(clog, "NO_PATH"),
        "NO_PATH -> stopped (FAULT) with a console line, no reverse");
  /* the sonar guard (obstacle_task) braked the creep early: the scan
     starts from wherever it stopped, the move is cancelled              */
  EV(APP_EVT_RESET, 0); idle_ms(110); to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; obs.distance_mm = 400u;
  EV(APP_EVT_OBSTACLE, 0); NONE(); CHECK("S5-U54", !strcmp(mlog, "F150 "), "creep started again");
  mot.motion_state = APP_MOT_ESTOP; safety_set(SAFETY_ESTOP); NONE();
  CHECK("S5-U54", MODE(APP_MODE_STOPPED | APP_MODE_RESCAN) && !strcmp(mlog, "STOP0 ") && !g_ctx.b_move_pending,
        "guard ESTOP at standstill ends the creep: move cancelled, scan requested");
  obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 280u; obs.leg_forward_mm = 580u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); CHECK("S5-U54", !strcmp(mlog, "L90 ") && !(g_safety & SAFETY_ESTOP), "PLAN after the creep: bypass as before");
  /* already near the scan distance: no creep */
  to_ready(); mot.motion_state = APP_MOT_BRAKE; to_run(); g_fault = 0;
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; obs.distance_mm = 268u;
  EV(APP_EVT_OBSTACLE, 0); NONE();
  CHECK("S5-U54", !strcmp(mlog, "") && MODE(APP_MODE_STOPPED | APP_MODE_RESCAN), "268 mm (within tolerance): scan at once, no creep");
  obs.distance_mm = 0u;
  /* S5-U57: the logged prefix brakes, rewinds once, then reads slowly. */
  barcode_test_follow(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U57", !strcmp(mlog, "STOP0 ") && MODE(APP_MODE_BARCODE_INHIBIT)
        && MODE(APP_MODE_JUNCTION_INHIBIT), "brake before rewinding");
  NONE();
  CHECK("S5-U57", !strcmp(mlog, ""), "wait for both wheels to stop");
  barcode_test_brake();
  CHECK("S5-U60", !strcmp(mlog, "B150 "), "limit rewind to 150 mm");
  EV(APP_EVT_MOVE_DONE, (g_tag + 3u) & 3u);
  CHECK("S5-U57", !strcmp(mlog, "") && MODE(APP_MODE_BARCODE_INHIBIT),
        "a stale completion cannot start the reread");
  barcode_test_back(312u, 1u, 6u);
  CHECK("S5-U60", !strcmp(mlog, "") && MODE(APP_MODE_BARCODE_INHIBIT),
        "old 72 mm endpoint is still black, keep reversing");
  barcode_test_back(320u, 0u, 1u);
  barcode_test_back(334u, 0u, 0u);
  CHECK("S5-U60", !strcmp(mlog, ""), "a 14 mm white gap does not qualify");
  barcode_test_back(335u, 0u, 2u);
  barcode_test_back(355u, 0u, 0u);
  CHECK("S5-U60", !strcmp(mlog, ""), "edges restart the quiet distance");
  barcode_test_back(370u, 0u, 0u);
  CHECK("S5-U63", !strcmp(mlog, "") && MODE(APP_MODE_BARCODE_INHIBIT),
        "white lead-in does not end the bounded reverse move early");
  barcode_test_back(390u, 0u, 0u);
  DONE();
  CHECK("S5-U63", !strcmp(mlog, ""), "wait for the reverse wheels to settle");
  barcode_test_still();
  CHECK("S5-U57", !strcmp(mlog, "FOL70 ") && MODE(APP_MODE_BARCODE_REREAD)
        && !MODE(APP_MODE_BARCODE_INHIBIT | APP_MODE_JUNCTION_INHIBIT),
        "resume line following at 70 mm/s and capture from the lead-in");
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U57", !strcmp(mlog, ""), "no second rewind during the slow pass");
  mot.path_counts += odometry_mm_to_counts(245u);
  EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u));
  CHECK("S5-U57", !strcmp(mlog, "FOL120 ") && APP_RUN_ARMED == g_ctx.run_state
        && 15u == g_ctx.armed_char && !MODE(APP_MODE_BARCODE_REREAD),
        "mapped F is stored and cruise resumes");
  EV(APP_EVT_JUNCTION, APP_JCT_SIDE_L);
  CHECK("S5-U57", !strcmp(mlog, "") && g_ctx.b_is_armed, "ignore a side branch");
  mot.path_counts += odometry_mm_to_counts(1100u);
  NONE();
  CHECK("S5-U57", APP_RUN_ARMED == g_ctx.run_state, "keep the code until CROSS");
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  CHECK("S5-U57", !strcmp(mlog, "STOP0 F60 "), "advance at the cross-junction");
  DONE();
  CHECK("S5-U57", !strcmp(mlog, "R90 "), "take F's mapped right turn");
  DONE();
  CHECK("S5-U57", !g_ctx.b_is_armed && !strcmp(mlog, "FOL120 "),
        "consume the action and resume cruise");

  /* The forum specifies cross-junctions, superseding the S5-U35 dead-end
     U-turn behavior. A loss must keep the stored command through search. */
  EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_UTURN, 13u));
  EV(APP_EVT_LINE_LOST, 0u);
  CHECK("S5-U57", !strcmp(mlog, "STOP0 L60 ") && g_ctx.b_is_armed,
        "line loss searches with a U-turn still armed");
  EV(APP_EVT_LINE_FOUND, 0u);
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  DONE();
  CHECK("S5-U57", !strcmp(mlog, "L180 "), "perform a U-turn at CROSS");
  DONE();
  EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_LEFT, 14u));
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  DONE();
  CHECK("S5-U57", !strcmp(mlog, "L90 "), "take the mapped left at CROSS");
  DONE();
  EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_STRAIGHT, 16u));
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  DONE();
  CHECK("S5-U57", !strcmp(mlog, "FOL120 ") && !g_ctx.b_is_armed,
        "mapped straight crosses and disarms without a spin");

  /* S5-U67: on-car 2026-11-10 22:03, the cross after the barcode was
     reported as T: centred on the line, both sensors read white 30 mm
     past the stripe. Armed, a T must turn like a cross. */
  EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_LEFT, 10u));
  EV(APP_EVT_JUNCTION, APP_JCT_T);
  CHECK("S5-U67", !strcmp(mlog, "STOP0 F60 ") && APP_RUN_EXECUTE_TURN == g_ctx.run_state,
        "armed: a T junction is taken as the cross");
  DONE();
  CHECK("S5-U67", !strcmp(mlog, "L90 "), "and A's mapped left is turned");
  DONE();
  CHECK("S5-U67", !g_ctx.b_is_armed, "the action is used up");

  barcode_test_follow(30u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U57", !strcmp(mlog, ""), "skip a rewind beyond the run start");
  /* S5-U66: on-car 2026-11-10 phantom echoes kept the scanner in
     APPROACH over the barcode; the rewind backs away from the obstacle. */
  obs.scanner_state = APP_SCAN_APPROACH;
  mot.path_counts = odometry_mm_to_counts(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U66", !strcmp(mlog, "STOP0 ") && MODE(APP_MODE_BARCODE_INHIBIT),
        "a rewind may start during obstacle approach");
  barcode_test_brake();
  CHECK("S5-U66", !strcmp(mlog, "B150 "), "and backs up during approach");
  obs.scanner_state = APP_SCAN_PASSING;
  barcode_test_follow(235u);
  obs.scanner_state = APP_SCAN_PASSING;
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U66", !strcmp(mlog, "") && !MODE(APP_MODE_BARCODE_INHIBIT),
        "no rewind while passing an obstacle");
  obs.scanner_state = APP_SCAN_MONITOR;

  /* S5-U65: on-car 2026-11-10 21:37, a misread strip asked for a rewind
     after its quiet zone (286 mm); 150 mm was refused. The rewind is now
     sent as reverse moves of at most 150 mm. */
  barcode_test_follow(600u);
  EV(APP_EVT_BARCODE_PROBE, 286u);
  CHECK("S5-U65", !strcmp(mlog, "STOP0 ") && MODE(APP_MODE_BARCODE_INHIBIT),
        "a whole-strip rewind is accepted");
  mot.motion_state = APP_MOT_BRAKE;
  mot.speed_l_cpw = mot.speed_r_cpw = 0;
  mot.path_counts = odometry_mm_to_counts(610u);
  NONE();
  CHECK("S5-U65", !strcmp(mlog, "B150 ") && 0u != g_ctx.barcode_back_left,
        "first reverse move is capped at 150 mm");
  barcode_test_back(760u, 0u, 0u);
  DONE();
  CHECK("S5-U65", !strcmp(mlog, "B150 "), "second reverse move");
  barcode_test_back(910u, 1u, 20u);
  DONE();
  CHECK("S5-U65", !strcmp(mlog, "B106 ") && 0u == g_ctx.barcode_back_left,
        "last move backs past the strip: 336 + 70 mm in total");
  barcode_test_back(950u, 0u, 1u);
  barcode_test_back(1016u, 0u, 0u);
  DONE();
  barcode_test_still();
  CHECK("S5-U65", !strcmp(mlog, "FOL70 ") && MODE(APP_MODE_BARCODE_REREAD),
        "white lead-in found: slow reread of the whole strip");

  barcode_test_read();
  obs.scanner_state = APP_SCAN_APPROACH;
  EV(APP_EVT_OBSTACLE_AHEAD, 0u);
  CHECK("S5-U57", !strcmp(mlog, ""), "approach does not speed up the reread");
  EV(APP_EVT_INVALID, 28u);
  CHECK("S5-U57", !strcmp(mlog, "FOL100 ") && !g_ctx.b_is_armed
        && !MODE(APP_MODE_BARCODE_REREAD),
        "incomplete reread leaves no route and respects approach speed");
  obs.scanner_state = APP_SCAN_MONITOR;
  EV(APP_EVT_CONTINUE, 0u);
  CHECK("S5-U57", !strcmp(mlog, "FOL120 "), "clear road restores cruise");

  barcode_test_follow(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  barcode_test_brake();
  mot.path_counts = odometry_mm_to_counts(295u);
  DONE();
  NONE();
  CHECK("S5-U60", !strcmp(mlog, "FOL120 ") && !MODE(APP_MODE_BARCODE_REREAD),
        "a short rewind gives up even if its last sample is white");

  barcode_test_follow(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  barcode_test_brake();
  barcode_test_back(380u, 0u, 1u);
  mot.path_counts = odometry_mm_to_counts(390u);
  mot.motion_state = APP_MOT_BRAKE;
  mot.speed_l_cpw = mot.speed_r_cpw = 0;
  DONE();
  NONE();
  CHECK("S5-U60", !strcmp(mlog, "FOL120 ") && !MODE(APP_MODE_BARCODE_REREAD),
        "150 mm without a quiet lead-in gives up without a slow pass");

  barcode_test_follow(235u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  barcode_test_brake();
  barcode_test_back(335u, 0u, 1u);
  barcode_test_back(390u, 0u, 0u);
  DONE();
  g_barcode_level = 1u;
  g_barcode_edges++;
  barcode_test_still();
  CHECK("S5-U60", !strcmp(mlog, "FOL120 ") && !MODE(APP_MODE_BARCODE_REREAD),
        "black during turnaround invalidates the lead-in");

  barcode_test_follow(100u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  mot.path_counts = odometry_mm_to_counts(105u);
  barcode_test_still();
  CHECK("S5-U60", !strcmp(mlog, "B95 "),
        "reverse cannot leave the prior follow segment plus brake margin");

  barcode_test_follow(200u);
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  mot.path_counts = odometry_mm_to_counts(260u);
  EV(APP_EVT_BARCODE_PROBE, 35u);
  CHECK("S5-U63", !strcmp(mlog, ""),
        "a rewind cannot cross a junction behind the car");

  {
    status_board_line_t const white = {0, APP_LINE_LOST, APP_JCT_NONE, true};
    status_board_line_t const seen = {0, APP_LINE_ON, APP_JCT_NONE, true};

    barcode_test_follow(235u);
    status_board_write_line(&white);
    EV(APP_EVT_BARCODE_PROBE, 71u);
    CHECK("S5-U64", !strcmp(mlog, "") && !MODE(APP_MODE_BARCODE_INHIBIT),
          "wait when the guard matches during a short both-white gap");

    mot.path_counts = odometry_mm_to_counts(250u);
    status_board_write_line(&seen);
    NONE();
    CHECK("S5-U64", !strcmp(mlog, "STOP0 ") && MODE(APP_MODE_BARCODE_INHIBIT)
          && odometry_mm_to_counts(124u) <= mot.path_counts - g_ctx.barcode_mark
          && mot.path_counts - g_ctx.barcode_mark <= odometry_mm_to_counts(128u),
          "brake after reacquiring the line and include the waiting distance");

    barcode_test_follow(235u);
    status_board_write_line(&white);
    EV(APP_EVT_BARCODE_PROBE, 71u);
    idle_ms(MISSION_BARCODE_WAIT_MS + 10u);
    status_board_write_line(&seen);
    NONE();
    CHECK("S5-U64", !strcmp(mlog, ""), "ignore the retry once its wait expires");

    barcode_test_follow(235u);
    status_board_write_line(&white);
    EV(APP_EVT_BARCODE_PROBE, 71u);
    /* 124 mm mark: cancel once the rewind needs more than 450 - 10 mm */
    mot.path_counts = odometry_mm_to_counts(566u);
    status_board_write_line(&seen);
    NONE();
    CHECK("S5-U64", !strcmp(mlog, ""), "keep brake room inside the rewind cap");

    barcode_test_follow(235u);
    status_board_write_line(&white);
    EV(APP_EVT_BARCODE_PROBE, 71u);
    EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
    status_board_write_line(&seen);
    NONE();
    CHECK("S5-U64", !strcmp(mlog, ""), "a junction cancels the deferred rewind");

    barcode_test_follow(235u);
    status_board_write_line(&white);
    EV(APP_EVT_BARCODE_PROBE, 71u);
    status_board_write_line(&seen);
    EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u));
    CHECK("S5-U64", !strcmp(mlog, "FOL120 ") && g_ctx.b_is_armed,
          "a successful first-pass decode supersedes the deferred rewind");
  }

  barcode_test_read();
  mot.path_counts += odometry_mm_to_counts(MISSION_BARCODE_READ_MM);
  NONE();
  CHECK("S5-U57", !strcmp(mlog, "FOL120 ") && !g_ctx.b_is_armed
        && !MODE(APP_MODE_BARCODE_REREAD), "unreadable scan has a distance limit");
  barcode_test_read();
  idle_ms(MISSION_BARCODE_READ_MS);
  CHECK("S5-U57", !strcmp(mlog, "FOL120 ") && !MODE(APP_MODE_BARCODE_REREAD),
        "unreadable scan also has a time limit");

  barcode_test_read();
  safety_set(SAFETY_ESTOP);
  mot.motion_state = APP_MOT_ESTOP;
  obs.distance_mm = 244u;
  EV(APP_EVT_OBSTACLE, 0u);
  NONE();
  CHECK("S5-U57", APP_RUN_AVOID == g_ctx.run_state && MODE(APP_MODE_RESCAN)
        && !MODE(APP_MODE_BARCODE_REREAD) && 0u != (g_safety & SAFETY_ESTOP),
        "obstacle stop takes priority over a slow pass");
  barcode_test_read();
  EV(APP_EVT_STOP, 0u);
  CHECK("S5-U57", APP_SYS_FINISHED == g_ctx.sys_state
        && !MODE(APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD),
        "STOP ends the reread with the motors disabled");

  /* S5-U61: a MAP reply confirms the stored letter and action. */
  tlog[0] = 0;
  to_ready();
  tlog[0] = 0;
  EV(APP_EVT_MAP, mission_evt_barcode_value(APP_BAR_CMD_LEFT, 15u));
  CHECK("S5-U61", APP_SYS_READY == g_ctx.sys_state && 15u == g_map_char
        && APP_BAR_CMD_LEFT == g_map_cmd && strstr(tlog, "E23:15 "),
        "READY: the F -> LEFT update sends its MAP confirmation");
  to_run();
  mot.motion_state = APP_MOT_FOLLOW;
  mot.speed_l_cpw = mot.speed_r_cpw = 18;
  tlog[0] = 0;
  EV(APP_EVT_MAP, mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u));
  CHECK("S5-U61", APP_SYS_RUN == g_ctx.sys_state && !strcmp(mlog, "")
        && 15u == g_map_char && APP_BAR_CMD_RIGHT == g_map_cmd
        && strstr(tlog, "E23:79 "),
        "RUN: F -> RIGHT confirms without issuing a motion command");
  g_map_stored = false;
  tlog[0] = 0;
  EV(APP_EVT_MAP, mission_evt_barcode_value(APP_BAR_CMD_LEFT, 16u));
  CHECK("S5-U61", NULL == strstr(tlog, "E23:"),
        "a failed map update has no success event");
  g_map_stored = true;

  tlog[0] = 0;
  clog[0] = 0;
  EV(APP_EVT_JUNCTION, APP_JCT_CROSS);
  CHECK("S5-U62", strstr(tlog, "E3:2 ") && !strcmp(mlog, "")
        && strstr(clog, "JUNCTION detected class 2"),
        "an unarmed cross is reported and the car keeps following");

  /* S5-U40: slower cruise (professor feedback) */
  CHECK("S5-U40", (120 == MOTION_CRUISE_MM_S) && (MOTION_SLOW_MM_S < MOTION_CRUISE_MM_S), "cruise 120 mm/s, 'slow' stays below cruise");
  printf("[TEST] S5-U07 mission simulation SUMMARY %d passed, %d failed\n", pass_n, fail_n);
  { int p0 = pass_n, f0 = fail_n;
    /* S5-U04 (moved from target, Fix 3): flood K4 from one task */
    int sent = 0, refused = 0, waits0 = mbf_waits; uint32_t hw = 0;
    qn = 0; mbf_used = 0; g_queue_high_water = 0; g_fault = 0;
    for (uint32_t i = 0; i < 20u; i++) { if (mission_post_event(APP_EVT_SUMMARY, i)) sent++; else refused++; }
    hw = mission_queue_high_water();
    CHECK("S5-U04", 16 == sent, "K4 holds 16 events (real capacity: 256 B / (4 + 12))");
    CHECK("S5-U04", 4 == refused, "extra posts are refused");
    CHECK("S5-U04", waits0 == mbf_waits, "every post uses TMO_POL: a refused post returns at once");
    CHECK("S5-U04", 0u != (g_fault & APP_FAULT_QUEUE_OVERFLOW), "QUEUE_OVERFLOW fault set");
    CHECK("S5-U04", 256u == hw, "high-water mark = whole buffer");
    CHECK("S5-U04", !mission_post_event(APP_EVT_COUNT, 0), "unknown event type refused");
    qn = 0; mbf_used = 0;
    /* S5-U05 (moved from target, Fix 2): every state x every event type */
    { uint32_t cases = 0u, bad = mission_test_matrix(&cases);
      CHECK("S5-U05", 468u == cases, "468 state/event combinations tried (18 states x 26 types)");
      CHECK("S5-U05", 0u == bad, "every outcome is a defined state"); }
    printf("[TEST] S5-U04/U05 mission host tests SUMMARY %d passed, %d failed\n", pass_n - p0, fail_n - f0); }
  return fail_n;
}
