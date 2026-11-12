/*
 * test_status_board_host.c
 * S5-U74 (Tribunal 1 change, min-bit item 6): the status board K6 is
 * stored as the 11 packed SB_* words of tools/layout.csv. Every field
 * written by its task reads back unchanged over its whole valid range,
 * including the sentinels, and the initial board reads "nothing ahead",
 * "clear" and zero. Host-only test code, outside BARR-C scope.
 * Run: make -C testing/subsystem5/host
 */
#include <stdio.h>
#include <string.h>
#include "app_program/subsystem5/status_board.h"
#include "app_program/app_types.h"

ID tk_cre_mtx(T_CMTX const *p) { (void) p; return 1; }
ER tk_loc_mtx(ID id, TMO t) { (void) id; (void) t; return E_OK; }
ER tk_unl_mtx(ID id) { (void) id; return E_OK; }

static int pass_n, fail_n;
#define CHECK(id, c, d) do { if (c) { pass_n++; printf("[TEST] %s PASS  %s\n", id, d); } \
  else { fail_n++; printf("[TEST] %s FAIL  %s (line %d)\n", id, d, __LINE__); } } while (0)

static status_board_t snap;

static bool line_ok(int32_t steer, uint32_t st, uint32_t jct, bool cal) {
  status_board_line_t p = {steer, st, jct, cal};
  status_board_write_line(&p); status_board_read(&snap);
  return status_board_steering(&snap) == steer && status_board_line_status(&snap) == st
         && status_board_junction(&snap) == jct && status_board_is_calibrated(&snap) == cal; }

static bool mot_ok(int32_t sl, int32_t sr, uint32_t ms, int32_t pl, int32_t pr, uint32_t path) {
  status_board_motion_t p = {sl, sr, ms, pl, pr, path};
  status_board_write_motion(&p); status_board_read(&snap);
  return status_board_speed_l(&snap) == sl && status_board_speed_r(&snap) == sr
         && status_board_motion_state(&snap) == ms && status_board_pos_l(&snap) == pl
         && status_board_pos_r(&snap) == pr && status_board_path(&snap) == path; }

static bool imu_ok(int32_t pitch, uint32_t hn, uint32_t hp, uint32_t mc, bool enc, int32_t tr) {
  status_board_imu_t p = {pitch, hn, hp, mc, enc, tr};
  status_board_write_imu(&p); status_board_read(&snap);
  return status_board_pitch(&snap) == pitch && status_board_hump_now(&snap) == hn
         && status_board_hump_peak(&snap) == hp && status_board_motion_class(&snap) == mc
         && status_board_is_turn_from_enc(&snap) == enc && status_board_turn_rate(&snap) == tr; }

static bool obs_ok(uint32_t d, uint32_t ss, uint32_t step, uint32_t side, bool v,
                   uint32_t w, uint32_t n, uint32_t cl, uint32_t cr, uint32_t ll, uint32_t lf) {
  status_board_obs_t p = {d, ss, step, side, v, w, n, cl, cr, ll, lf};
  status_board_write_obstacle(&p); status_board_read(&snap);
  return status_board_distance(&snap) == d && status_board_scanner_state(&snap) == ss
         && status_board_angle_step(&snap) == step && status_board_plan_side(&snap) == side
         && status_board_is_profile_valid(&snap) == v && status_board_width(&snap) == w
         && status_board_nearest(&snap) == n && status_board_clear_left(&snap) == cl
         && status_board_clear_right(&snap) == cr && status_board_leg_lateral(&snap) == ll
         && status_board_leg_forward(&snap) == lf; }

static bool mis_ok(uint32_t sys, uint32_t run, uint32_t av, uint32_t end, bool arm, uint32_t cmd) {
  status_board_mission_t p = {sys, run, av, end, arm, cmd};
  status_board_write_mission(&p); status_board_read(&snap);
  return status_board_sys_state(&snap) == sys && status_board_run_state(&snap) == run
         && status_board_avoid_step(&snap) == av && status_board_end_reason(&snap) == end
         && status_board_is_armed(&snap) == arm && status_board_armed_cmd(&snap) == cmd; }

int main(void) {
  status_board_init();
  status_board_read(&snap);
  CHECK("S5-U74", status_board_distance(&snap) == APP_DIST_NONE_MM
        && status_board_width(&snap) == APP_PROFILE_CLEAR_MM
        && status_board_nearest(&snap) == APP_PROFILE_CLEAR_MM
        && status_board_clear_left(&snap) == APP_PROFILE_CLEAR_MM
        && status_board_clear_right(&snap) == APP_PROFILE_CLEAR_MM
        && status_board_steering(&snap) == 0 && status_board_speed_l(&snap) == 0
        && status_board_speed_r(&snap) == 0 && status_board_pos_l(&snap) == 0
        && status_board_pos_r(&snap) == 0 && status_board_pitch(&snap) == 0
        && status_board_turn_rate(&snap) == 0 && status_board_path(&snap) == 0
        && status_board_sys_state(&snap) == 0,
        "initial board: nothing ahead, profile clear, every other field 0");
  CHECK("S5-U74", sizeof(status_board_t) == 44u,
        "snapshot is the 11 packed 32-bit words (44 B)");
  CHECK("S5-U74", line_ok(-1000, 3u, 4u, true) && line_ok(1000, 0u, 0u, false)
        && line_ok(0, 1u, 1u, true) && line_ok(-1, 2u, 3u, false),
        "line part: steering -1000..1000, status, junction, calibrated");
  CHECK("S5-U74", mot_ok(-127, 127, 7u, -131071, 131071, 131071)
        && mot_ok(127, -127, 0u, 131071, -131071, 0u) && mot_ok(18, 17, 1u, 3086, -3086, 3086),
        "motion part: speeds +-127, state, positions +-131071, path 0..131071");
  CHECK("S5-U74", imu_ok(-450, 127u, 127u, 6u, true, -360)
        && imu_ok(450, 0u, 0u, 0u, false, 360) && imu_ok(25, 11u, 12u, 3u, true, -90),
        "IMU part: pitch +-450, hump 0..127, class, source, turn +-360");
  CHECK("S5-U74", obs_ok(APP_DIST_NONE_MM, 7u, 24u, 1u, true, 1020u, 1020u, 1020u, 1020u, 1020u, 1020u)
        && obs_ok(0u, 0u, 0u, 0u, false, 0u, 0u, 0u, 0u, 0u, 0u)
        && obs_ok(252u, 2u, 12u, 0u, true, 200u, 248u, 120u, 1020u, 352u, 800u),
        "obstacle part: 4 mm steps, sentinels 2044 / 1020, legs 0..1020");
  CHECK("S5-U74", mis_ok(7u, 4u, 6u, 3u, true, 3u) && mis_ok(0u, 0u, 0u, 0u, false, 0u)
        && mis_ok(4u, 1u, 0u, 2u, true, 1u),
        "mission part: every state, step, end reason and armed command");
  {
    status_board_imu_t p = {900, 0u, 0u, 0u, false, 0};
    status_board_write_imu(&p); status_board_read(&snap);
    CHECK("S5-U74", status_board_pitch(&snap) == 450,
          "pitch beyond +-45 deg is stored as 45 deg (ramp check uses +-2 deg)");
  }
  printf("[TEST] S5-U74 status board host tests SUMMARY %d passed, %d failed\n", pass_n, fail_n);
  return (0 == fail_n) ? 0 : 1;
}
