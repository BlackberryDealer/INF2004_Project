/*
 * S5-U83: barcode bench image (MISSION_BAR_BENCH=1, make BAR_BENCH=1).
 * The mission statechart built as the bench: CAL_IMU goes straight to
 * READY (no IR sweep), each pass is a tracked FWD MISSION_BAR_BENCH_MM,
 * a pass without a decode backs up in <= 150 mm steps and retries
 * (MISSION_BAR_BENCH_RETRIES), a decode ends the run, OBSTACLE events
 * are ignored, and a move that never finishes ends the run once as
 * FINISHED (on-car 2026-11-19: STOP + FAULT summaries after a sonar
 * stop). Reuses sim_mission_host.c's mocks; its main() is not run.
 */
#define main sim_mission_main_unused
#include "sim_mission_host.c"
#undef main

#if (MISSION_BAR_BENCH != 1u)
#error "S5-U83 must be built with MISSION_BAR_BENCH=1u"
#endif

static int summaries(void)
{
    int n = 0;
    for (char const *p = tlog; NULL != (p = strstr(p, "SUMM(")); p++) { n++; }
    return n;
}

static void bench_pass_fail(uint32_t * p_path_mm)
{
    /* drive the pass, no decode, then four 150 mm steps back */
    *p_path_mm += 600u; mot.path_counts = odometry_mm_to_counts(*p_path_mm);
    DONE();
}

int main(void)
{
    status_board_t snap;
    uint32_t       path_mm = 0u;

    status_board_init(); mission_init();
    mission_ctx_reset(&g_ctx); status_board_read(&snap);
    mission_enter(&g_ctx, APP_SYS_BOOT, &snap, g_now);
    g_started = true; NONE();
    CHECK("S5-U83", APP_SYS_CAL_IMU == g_ctx.sys_state, "bench boots to CAL_IMU");
    clog[0] = 0;
    EV(APP_EVT_CAL_DONE, APP_CAL_SRC_IMU);
    CHECK("S5-U83", APP_SYS_READY == g_ctx.sys_state && !strcmp(mlog, "")
          && strstr(clog, "IR line calibration skipped"),
          "IMU calibrated -> READY, no IR sweep (no line tiles needed)");

    obs.scanner_state = APP_SCAN_MONITOR; obs.distance_mm = APP_DIST_NONE_MM;
    mot.path_counts = 0u; alog[0] = 0; tlog[0] = 0; clog[0] = 0;
    to_run();
    CHECK("S5-U83", APP_SYS_RUN == g_ctx.sys_state && strstr(alog, "F600 ")
          && strstr(clog, "sonar does not brake") && g_ctx.b_move_pending,
          "RUN: pass 1 is a tracked forward 600 mm; sonar note printed");

    EV(APP_EVT_OBSTACLE, 0u);
    CHECK("S5-U83", APP_SYS_RUN == g_ctx.sys_state && !strcmp(mlog, ""),
          "OBSTACLE during a pass is ignored (no stop, no avoid)");

    bench_pass_fail(&path_mm);
    CHECK("S5-U83", !strcmp(mlog, "B150 ") && strstr(clog, "retry 1 of 3"),
          "pass 1 ends with no read: back up 150 mm, retry 1 of 3");
    DONE(); DONE(); DONE();
    CHECK("S5-U83", !strcmp(mlog, "B150 "), "four 150 mm steps back to the pass start");
    DONE();
    CHECK("S5-U83", !strcmp(mlog, "F600 "), "back at the start: pass 2 forward 600 mm");

    EV(APP_EVT_BARCODE, mission_evt_barcode_value(APP_BAR_CMD_LEFT, 10u));
    CHECK("S5-U83", APP_SYS_RUN == g_ctx.sys_state && strstr(clog, "'A' decoded"),
          "decode during pass 2 is logged as 'A'");
    path_mm += 600u; mot.path_counts = odometry_mm_to_counts(path_mm);
    DONE();
    CHECK("S5-U83", APP_SYS_FINISHED == g_ctx.sys_state && strstr(clog, "read OK on pass 2")
          && 1 == summaries(), "pass 2 done with a read -> FINISHED, one summary");

    /* all four passes fail */
    EV(APP_EVT_RESET, 0u); idle_ms(110); NONE();
    CHECK("S5-U83", APP_SYS_READY == g_ctx.sys_state, "RESET -> READY");
    alog[0] = 0; tlog[0] = 0; clog[0] = 0;
    to_run();
    for (uint32_t pass = 0u; pass < 3u; pass++)
    {
        bench_pass_fail(&path_mm);
        DONE(); DONE(); DONE(); DONE();
    }
    CHECK("S5-U83", !strcmp(mlog, "F600 ") && strstr(clog, "retry 3 of 3"),
          "three retries: pass 4 starts");
    bench_pass_fail(&path_mm);
    CHECK("S5-U83", APP_SYS_FINISHED == g_ctx.sys_state && !strcmp(mlog, "STOP0 ")
          && strstr(clog, "no read after 4 passes") && 1 == summaries(),
          "pass 4 without a read -> FINISHED (stop), one summary");

    /* a move that never finishes (blocked wheel): FINISHED once, no FAULT */
    EV(APP_EVT_RESET, 0u); idle_ms(110); NONE();
    alog[0] = 0; tlog[0] = 0; clog[0] = 0;
    to_run();
    idle_ms(17000u);
    CHECK("S5-U83", APP_SYS_FINISHED == g_ctx.sys_state && 1 == summaries()
          && NULL == strstr(tlog, "SUMM(end3"),
          "no MOVE_DONE within the pass deadline -> one FINISHED summary, no FAULT");

    printf("[TEST] sim_mission_bench SUMMARY %d passed, %d failed\n", pass_n, fail_n);
    return (0 == fail_n) ? 0 : 1;
}
