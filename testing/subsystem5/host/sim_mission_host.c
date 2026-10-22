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
void app_log(char const *f, int32_t a, int32_t b, int32_t c) { printf("   log: "); printf(f, a, b, c); }
/* ---------------- neighbour mocks ---------------- */
static uint32_t g_safety, g_fault, g_now, g_tag; static int g_refuse;
void safety_set(uint32_t b) { g_safety |= b; }
void safety_clear(uint32_t b) { g_safety &= ~b; }
uint32_t safety_get(void) { return g_safety; }
void fault_set(uint32_t b) { g_fault |= b; }
void fault_clear(uint32_t b) { g_fault &= ~b; }
uint32_t fault_get(void) { return g_fault; }
static char mlog[256], alog[1024];   /* this pass / since last clear */
static bool mcmd(char const *s, int32_t v, bool tagged) {
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
uint32_t odometry_counts_to_mm(uint32_t c) { return (c * 349u) / 1000u; }
uint32_t odometry_mm_to_counts(uint32_t d) { return (d * 1000u) / 349u; }
int32_t odometry_heading_cdeg(int32_t l, int32_t r) { return (l - r) * 16; }
static char tlog[512];
bool telem_post_event(uint32_t t, uint32_t v) { if (strlen(tlog) < 400) sprintf(tlog + strlen(tlog), "E%u:%u ", t, v); return true; }
bool telem_post_profile(telem_bin_profile_t const *p) { sprintf(tlog + strlen(tlog), "PROF%u ", p->side); return true; }
bool telem_post_summary(telem_bin_summary_t const *p) { sprintf(tlog + strlen(tlog), "SUMM(end%u odo%u) ", p->end_reason, p->odometry_mm); return true; }
void barcode_map_set(uint32_t c, uint32_t m) { (void) c; (void) m; }
void sys_health_service(void) {}
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
  alog[0] = 0; idle_ms(30); CHECK("S5-U20", APP_SYS_RUN == g_ctx.sys_state && !MODE(APP_MODE_RUN_START) && MODE(APP_MODE_MOTOR_EN) && 0u == g_ctx.run_start_path && !strcmp(alog, "FOL250 "), "-> RUN: RUN_START cleared, MOTOR_EN set, path 0, FOLLOW cruise");
  /* barcode turn */
  EV(APP_EVT_BARCODE, (1u << 6) | 11u); CHECK("S5-U07", APP_RUN_ARMED == g_ctx.run_state && !strcmp(mlog, "FOL150 "), "BARCODE(RIGHT) -> ARMED slow");
  EV(APP_EVT_JUNCTION, 1); CHECK("S5-U07", !strcmp(mlog, "STOP0 F60 "), "JUNCTION -> stop, advance 60 (tracked)");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R90 "), "turn right 90");
  DONE(); CHECK("S5-U07", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "FOL250 ") && !g_ctx.b_move_pending, "turn done -> FOLLOW cruise; FOLLOW is never tracked");
  /* S5-U15: search sends STOP first */
  EV(APP_EVT_LINE_LOST, 0); CHECK("S5-U15", APP_RUN_SEARCH_LINE == g_ctx.run_state && !strcmp(mlog, "STOP0 L60 "), "LINE_LOST -> STOP first, then turn left 60");
  DONE(); CHECK("S5-U15", !strcmp(mlog, "R120 "), "next leg on the tagged MOVE_DONE: right 120");
  EV(APP_EVT_LINE_FOUND, 0); CHECK("S5-U15", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "STOP0 FOL250 ") && !g_ctx.b_move_pending, "LINE_FOUND during a leg -> FOLLOW, pending cleared");
  /* obstacle + bypass */
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; mot.pos_l_counts = 1000; mot.pos_r_counts = 900;
  EV(APP_EVT_OBSTACLE, 0); NONE(); CHECK("S5-U07", APP_RUN_AVOID == g_ctx.run_state && MODE(APP_MODE_STOPPED | APP_MODE_RESCAN), "OBSTACLE -> AVOID, standstill -> scan request");
  idle_ms(200); obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 176u; obs.leg_forward_mm = 632u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); CHECK("S5-U07", !(g_safety & SAFETY_ESTOP) && !strcmp(mlog, "L90 ") && strstr(tlog, "PROF0"), "PLAN -> ESTOP off, profile, turn left 90");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "F176 "), "lateral leg 176");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R90 "), "turn parallel");
  DONE(); CHECK("S5-U27", !strcmp(mlog, "F632 ") && g_ctx.move_limit_ms >= 7000u, "pass leg 632 mm; deadline long enough for it");
  idle_ms(5000); CHECK("S5-U27", APP_SYS_RUN == g_ctx.sys_state, "a healthy 632 mm leg is not faulted at 5 s");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "R45 ") && g_ctx.b_fr9_running, "turn 45 toward the line, FR9 starts");
  DONE(); CHECK("S5-U07", !strcmp(mlog, "F322 "), "rejoin move e x 1.3");
  mot.path_counts += odometry_mm_to_counts(250u);
  EV(APP_EVT_LINE_FOUND, 24u); CHECK("S5-U07", !strcmp(mlog, "STOP0 F60 "), "LINE_FOUND in the window -> advance 60");
  mot.pos_l_counts = 1050;
  DONE(); CHECK("S5-U07", !strcmp(mlog, "L8 "), "heading +8 deg -> turn left 8");
  DONE(); CHECK("S5-U07", APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "FOL150 ") && MODE(APP_MODE_RESUME), "bypass done -> FOLLOW slow, RESUME");
  idle_ms(160); CHECK("S5-U07", !MODE(APP_MODE_RESUME), "RESUME cleared after 150 ms");
  mot.path_counts += odometry_mm_to_counts(151u); NONE(); CHECK("S5-U07", !MODE(APP_MODE_JUNCTION_INHIBIT) && !strcmp(mlog, "FOL250 "), "inhibits cleared 150 mm later, cruise");
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
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); DONE(); DONE(); DONE(); EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR);
  CHECK("S5-U16", APP_SYS_READY == g_ctx.sys_state, "recalibrated -> READY");
  /* S5-U16: IMPACT during RUN -> FAULT */
  to_run(); safety_set(SAFETY_IMPACT); NONE(); CHECK("S5-U16", APP_SYS_FAULT == g_ctx.sys_state, "IMPACT during FOLLOW -> FAULT");
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
      CHECK("S5-U05", 432u == cases, "432 state/event combinations tried");
      CHECK("S5-U05", 0u == bad, "every outcome is a defined state"); }
    printf("[TEST] S5-U04/U05 mission host tests SUMMARY %d passed, %d failed\n", pass_n - p0, fail_n - f0); }
  return fail_n;
}
