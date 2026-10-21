/*
 * sim_mission_host.c
 * Laptop simulation of mission.c + mission_run.c (the statechart) with
 * mocked neighbours: kernel, motion, safety, fault, telem, odometry.
 * Drives a whole run (calibration, barcode turn, bypass with rejoin,
 * NO_PATH, faults, line search) and checks every motion command.
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
static mission_evt_t q[64]; static int qn = 0; static int qcap = 16;
ID tk_cre_mbf(T_CMBF const *p) { (void) p; return 1; }
ER tk_snd_mbf(ID id, void const *m, INT sz, TMO t) { (void) id; (void) sz; (void) t;
  if (qn >= qcap) { return E_TMOUT; }
  memcpy(&q[qn++], m, sizeof(mission_evt_t)); return E_OK; }
INT tk_rcv_mbf(ID id, void *m, TMO t) { (void) id; (void) t; if (!qn) return E_TMOUT;
  memcpy(m, &q[0], sizeof(mission_evt_t)); memmove(q, q + 1, (size_t)(--qn) * sizeof(q[0])); return 12; }
ER tk_ref_mbf(ID id, T_RMBF *r) { (void) id; r->frbufsz = 256 - qn * 16; return E_OK; }
ID tk_cre_mtx(T_CMTX const *p) { (void) p; return 1; }
ER tk_loc_mtx(ID id, TMO t) { (void) id; (void) t; return E_OK; }
ER tk_unl_mtx(ID id) { (void) id; return E_OK; }
ID app_start_task(FP e, PRI p, uint64_t *s, uint32_t b, char const *n) { (void)e;(void)p;(void)s;(void)b;(void)n; return 1; }
void app_check(ER e, char const *w) { (void) e; (void) w; }
void app_log(char const *f, int32_t a, int32_t b, int32_t c) { printf("   log: "); printf(f, a, b, c); }
/* ---------------- neighbour mocks ---------------- */
static uint32_t g_safety, g_fault, g_now;
void safety_set(uint32_t b) { g_safety |= b; }
void safety_clear(uint32_t b) { g_safety &= ~b; }
uint32_t safety_get(void) { return g_safety; }
void fault_set(uint32_t b) { g_fault |= b; }
void fault_clear(uint32_t b) { g_fault &= ~b; }
uint32_t fault_get(void) { return g_fault; }
static char mlog[256];
static void mcmd(char const *s, int32_t v) { sprintf(mlog + strlen(mlog), "%s%d ", s, (int) v); }
bool motion_move_forward(int32_t d) { mcmd("F", d); return true; }
bool motion_move_backward(int32_t d) { mcmd("B", d); return true; }
bool motion_turn_left(int32_t a) { mcmd("L", a); return true; }
bool motion_turn_right(int32_t a) { mcmd("R", a); return true; }
bool motion_follow(int32_t s) { mcmd("FOL", s); return true; }
bool motion_stop(void) { mcmd("STOP", 0); return true; }
uint32_t odometry_counts_to_mm(uint32_t c) { return (c * 349u) / 1000u; }
uint32_t odometry_mm_to_counts(uint32_t d) { return (d * 1000u) / 349u; }
int32_t odometry_heading_cdeg(int32_t l, int32_t r) { return (l - r) * 16; }
static char tlog[512];
bool telem_post_event(uint32_t t, uint32_t v) { sprintf(tlog + strlen(tlog), "E%u:%u ", t, v); return true; }
bool telem_post_profile(telem_bin_profile_t const *p) { sprintf(tlog + strlen(tlog), "PROF%u ", p->side); return true; }
bool telem_post_summary(telem_bin_summary_t const *p) { sprintf(tlog + strlen(tlog), "SUMM(end%u odo%u s%u f%u) ", p->end_reason, p->odometry_mm, p->run_time_s, p->faults); return true; }
void barcode_map_set(uint32_t c, uint32_t m) { printf("   map %u %u\n", c, m); }
void sys_health_service(void) {}
void sys_health_enable_watchdog(void) {}
uint32_t sys_tick_now_us(void) { return g_now; }
void hmi_init(void) {}
uint32_t hmi_poll(void) { return 0u; }
void hmi_show(uint32_t s, uint32_t n) { (void) s; (void) n; }
static bool g_warn; void hmi_set_warning(bool b) { g_warn = b; }
/* ---------------- harness ---------------- */
static int pass_n, fail_n;
#define CHECK(c, d) do { if (c) { pass_n++; } else { fail_n++; printf("FAIL %s  (line %d) mlog='%s' sys=%u run=%u av=%u sub=%u\n", d, __LINE__, mlog, g_ctx.sys_state, g_ctx.run_state, g_ctx.avoid_step, g_ctx.substep); } } while (0)
static status_board_motion_t mot; static status_board_obs_t obs;
static void step(uint32_t type, uint32_t value, bool has) {
  mission_evt_in_t e = {has, type, value}, b = {false, 0, 0}; status_board_t snap;
  g_now += 10000u; status_board_write_motion(&mot); status_board_write_obstacle(&obs);
  status_board_read(&snap); mlog[0] = 0; mission_pass(&g_ctx, &e, &b, &snap, g_now); }
#define EV(t, v) step((t), (v), true)
#define NONE() step(0, 0, false)
static void idle_ms(uint32_t ms) { for (uint32_t i = 0; i < ms / 10u; i++) NONE(); }

int main(void) {
  status_board_t snap;
  status_board_init(); mission_init();   /* flag id 1 = K10 */
  mission_ctx_reset(&g_ctx); status_board_read(&snap); mission_enter(&g_ctx, APP_SYS_BOOT, &snap, g_now);
  #define MODE(b) (0u != (flg[1] & (b)))
  NONE(); CHECK(APP_SYS_CAL_IMU == g_ctx.sys_state && MODE(APP_MODE_CAL_IMU), "BOOT -> CAL_IMU, mode set");
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU); CHECK(APP_SYS_CAL_IR == g_ctx.sys_state && !MODE(APP_MODE_CAL_IMU) && MODE(APP_MODE_CAL_IR) && !strcmp(mlog, "L30 "), "CAL_IMU done -> CAL_IR, sweep left 30");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "R60 "), "sweep right 60");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "L30 "), "sweep left 30");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!MODE(APP_MODE_CAL_IR), "sweep done clears CAL_IR");
  EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IR); CHECK(APP_SYS_READY == g_ctx.sys_state, "CAL_DONE(IR) -> READY");
  EV(APP_EVT_START, 0); CHECK(g_warn && APP_SYS_READY == g_ctx.sys_state, "START: 1 s warning");
  idle_ms(980); CHECK(APP_SYS_READY == g_ctx.sys_state, "still READY before 1 s");
  idle_ms(30); CHECK(APP_SYS_RUN == g_ctx.sys_state && APP_RUN_FOLLOW == g_ctx.run_state && !g_warn, "RUN/FOLLOW after 1 s");
  CHECK(strstr(tlog, "E21:4") != NULL, "STATE_CHANGE RUN|FOLLOW posted");
  /* barcode -> armed -> junction -> right turn */
  EV(APP_EVT_BARCODE, (1u << 6) | 11u); CHECK(APP_RUN_ARMED == g_ctx.run_state && g_ctx.b_is_armed && !strcmp(mlog, "FOL150 "), "BARCODE(RIGHT) -> ARMED slow");
  EV(APP_EVT_OBSTACLE_AHEAD, 0); CHECK(!strcmp(mlog, ""), "ARMED ignores OBSTACLE_AHEAD speed change");
  EV(APP_EVT_JUNCTION, 1); CHECK(APP_RUN_EXECUTE_TURN == g_ctx.run_state && !strcmp(mlog, "STOP0 F60 "), "JUNCTION -> stop, advance 60");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "R90 "), "turn right 90");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(APP_RUN_FOLLOW == g_ctx.run_state && !g_ctx.b_is_armed && !strcmp(mlog, "FOL250 "), "turn done -> FOLLOW cruise, disarmed");
  /* arm expiry */
  EV(APP_EVT_BARCODE, (2u << 6) | 3u); mot.path_counts += odometry_mm_to_counts(1000u) + 1u; NONE();
  CHECK(APP_RUN_FOLLOW == g_ctx.run_state && !g_ctx.b_is_armed, "armed command expires after 1000 mm");
  /* obstacle and bypass */
  EV(APP_EVT_OBSTACLE_AHEAD, 0); CHECK(!strcmp(mlog, "FOL150 "), "OBSTACLE_AHEAD -> slow");
  safety_set(SAFETY_ESTOP); mot.motion_state = APP_MOT_ESTOP; mot.speed_l_cpw = 5; mot.pos_l_counts = 1000; mot.pos_r_counts = 900;
  EV(APP_EVT_OBSTACLE, 0); CHECK(APP_RUN_AVOID == g_ctx.run_state && MODE(APP_MODE_BARCODE_INHIBIT) && MODE(APP_MODE_JUNCTION_INHIBIT) && !strcmp(mlog, "STOP0 "), "OBSTACLE -> AVOID, inhibits");
  CHECK(!MODE(APP_MODE_STOPPED), "no scan request while still rolling");
  mot.speed_l_cpw = 0; NONE(); CHECK(MODE(APP_MODE_STOPPED) && MODE(APP_MODE_RESCAN), "standstill -> STOPPED|RESCAN");
  idle_ms(200);
  obs.plan_side = APP_SIDE_LEFT; obs.leg_lateral_mm = 176u; obs.leg_forward_mm = 630u;
  EV(APP_EVT_PLAN, APP_SIDE_LEFT); CHECK(!MODE(APP_MODE_STOPPED) && !MODE(APP_MODE_RESCAN) && !(g_safety & SAFETY_ESTOP) && !strcmp(mlog, "L90 ") && strstr(tlog, "PROF0"), "PLAN -> clear, ESTOP off, profile, turn left 90");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "F176 ") && APP_AVOID_LEG_OUT == g_ctx.avoid_step, "lateral leg 176");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "R90 "), "turn parallel (right 90)");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "F630 "), "pass leg 630");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "R45 ") && APP_AVOID_REJOIN == g_ctx.avoid_step && g_ctx.b_fr9_running, "turn 45 toward line, FR9 starts");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "F322 ") && 248u == g_ctx.rejoin_e_mm, "rejoin move e x 1.3 (e = 248)");
  mot.path_counts += odometry_mm_to_counts(100u);
  EV(APP_EVT_LINE_FOUND, 25u); CHECK(!strcmp(mlog, "") && 1u == g_ctx.substep, "LINE_FOUND too early (100 mm < 70 %) ignored");
  mot.path_counts += odometry_mm_to_counts(150u);
  EV(APP_EVT_LINE_FOUND, 40u); CHECK(!strcmp(mlog, ""), "LINE_FOUND with wrong width ignored");
  EV(APP_EVT_LINE_FOUND, 24u); CHECK(!strcmp(mlog, "STOP0 F60 ") && 2u == g_ctx.substep, "LINE_FOUND in window -> advance 60");
  mot.pos_l_counts = 1000 + 50; mot.pos_r_counts = 900;    /* heading error = +50 x 16 = 800 cdeg */
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "L8 "), "heading +8 deg -> turn left 8");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "FOL150 ") && MODE(APP_MODE_RESUME), "bypass done -> FOLLOW slow, RESUME set");
  idle_ms(100); CHECK(MODE(APP_MODE_RESUME), "RESUME still set at 100 ms");
  idle_ms(60); CHECK(!MODE(APP_MODE_RESUME), "RESUME cleared after 150 ms");
  CHECK(MODE(APP_MODE_JUNCTION_INHIBIT), "inhibits kept until 150 mm");
  mot.path_counts += odometry_mm_to_counts(151u); NONE();
  CHECK(!MODE(APP_MODE_JUNCTION_INHIBIT) && !MODE(APP_MODE_BARCODE_INHIBIT) && !strcmp(mlog, "FOL250 "), "inhibits cleared, back to cruise");
  /* stop and reset */
  tlog[0] = 0; EV(APP_EVT_STOP, 0); CHECK(APP_SYS_FINISHED == g_ctx.sys_state && strstr(tlog, "SUMM(end0"), "STOP -> FINISHED + summary");
  EV(APP_EVT_RESET, 0); CHECK(APP_SYS_READY == g_ctx.sys_state && MODE(APP_MODE_RESUME), "RESET -> READY + RESUME pulse");
  idle_ms(200);
  /* NO_PATH twice -> FAULT */
  EV(APP_EVT_START, 0); idle_ms(1010); mot.motion_state = APP_MOT_BRAKE; mot.speed_l_cpw = 0;
  EV(APP_EVT_OBSTACLE, 0); NONE(); CHECK(MODE(APP_MODE_STOPPED) && MODE(APP_MODE_RESCAN), "AVOID scan request");
  idle_ms(200); EV(APP_EVT_NO_PATH, 0); CHECK(!strcmp(mlog, "B150 ") && MODE(APP_MODE_STOPPED) && !MODE(APP_MODE_RESCAN) && APP_AVOID_REVERSE == g_ctx.avoid_step, "first NO_PATH -> reverse 150, keep STOPPED");
  idle_ms(200); EV(APP_EVT_MOVE_DONE, 0); CHECK(MODE(APP_MODE_RESCAN) && APP_AVOID_WAIT_PLAN == g_ctx.avoid_step, "reverse done -> RESCAN again");
  EV(APP_EVT_NO_PATH, 0); CHECK(APP_SYS_FAULT == g_ctx.sys_state && (g_safety & SAFETY_FAULT), "second NO_PATH -> FAULT");
  EV(APP_EVT_RESET, 0); CHECK(APP_SYS_FAULT == g_ctx.sys_state && 0u == g_safety, "RESET in FAULT clears, waits a pass");
  NONE(); CHECK(APP_SYS_READY == g_ctx.sys_state, "-> READY when no stop fault");
  /* stall fault stays */
  fault_set(APP_FAULT_MOTOR_STALL); NONE(); CHECK(APP_SYS_FAULT == g_ctx.sys_state, "MOTOR_STALL fault -> FAULT");
  EV(APP_EVT_RESET, 0); fault_set(APP_FAULT_MOTOR_STALL); NONE(); CHECK(APP_SYS_FAULT == g_ctx.sys_state && (g_safety & SAFETY_FAULT), "stop fault back after RESET -> stay FAULT");
  fault_clear(APP_FAULT_ALL); EV(APP_EVT_RESET, 0); NONE(); CHECK(APP_SYS_READY == g_ctx.sys_state, "clean RESET -> READY");
  /* wifi loss is not a transition */
  fault_set(APP_FAULT_WIFI_LOSS); NONE(); CHECK(APP_SYS_READY == g_ctx.sys_state, "Wi-Fi loss is not a transition");
  /* line search timeout */
  idle_ms(200); EV(APP_EVT_START, 0); idle_ms(1010); EV(APP_EVT_LINE_LOST, 0);
  CHECK(APP_RUN_SEARCH_LINE == g_ctx.run_state && !strcmp(mlog, "L60 "), "LINE_LOST -> SEARCH sweep left 60");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "R120 "), "sweep right 120");
  EV(APP_EVT_MOVE_DONE, 0); CHECK(!strcmp(mlog, "L60 "), "sweep left 60");
  idle_ms(4950); CHECK(APP_RUN_SEARCH_LINE == g_ctx.run_state, "still searching at 4.97 s"); idle_ms(40); CHECK(APP_SYS_LINE_LOST_STOP == g_ctx.sys_state, "5 s without line -> LINE_LOST_STOP");
  /* line found during search */
  EV(APP_EVT_RESET, 0); idle_ms(200); EV(APP_EVT_START, 0); idle_ms(1010); EV(APP_EVT_LINE_LOST, 0); EV(APP_EVT_LINE_FOUND, 0);
  CHECK(APP_RUN_FOLLOW == g_ctx.run_state && !strcmp(mlog, "STOP0 FOL250 "), "LINE_FOUND in search -> FOLLOW");
  /* CAL timeout */
  mission_enter(&g_ctx, APP_SYS_CAL_IMU, &snap, g_now); idle_ms(10010); CHECK(APP_SYS_FAULT == g_ctx.sys_state && (g_fault & APP_FAULT_CAL_TIMEOUT), "CAL_IMU 10 s -> CAL_TIMEOUT FAULT");
  printf("[TEST] S5-U07 mission simulation SUMMARY %d passed, %d failed\n", pass_n, fail_n);
  return fail_n;
}
