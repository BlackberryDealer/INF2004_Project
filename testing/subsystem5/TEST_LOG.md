# Subsystem 5 — test log

**Subsystem:** Shared services, obstacles and the mission (Buddy 5)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U04 | Flooding K4 never blocks a producer | K4 `mission_post_event`, min-bit item 7 | auto (host) | `sim_mission_host.c` (moved from target, fix brief Fix 3): 20 posts against a K4 mock with the kernel's sizing (4 B header + 12 B message, 256 B buffer) | 16 accepted, 4 refused, every post uses `TMO_POL`, `APP_FAULT_QUEUE_OVERFLOW` set, high-water = 256 B |
| S5-U06 | Release-latency counter moves under load | `sys_tick_cycle_start`, NFR17 | auto | `test_mission.c`: a priority-11 task busy-loops 2 ms at tick boundaries for 40 rounds | `sys_tick_latency_max_us()` rises and reaches ≥ 1500 µs |
| S5-I05 | A12: R1 handler time | R1 `gpio_bank0_isr`, NFR5a | manual | With Buddy 3: signal generator on GP7 (barcode DO), build `PROBE=1`, scope GP14 (high inside R1) | R1 < 20 µs per entry |
| S5-I06 | A21: sensor-to-front offset | `OBS_FRONT_OFFSET_MM` | manual | After assembly, measure from the HC-SR04P face to the robot's front-most point; photo with ruler | Value written into `cfg_obstacle.h` (placeholder 60 mm) |
| S5-I07 | A8: HC-SR04P at 3.3 V, range and timeout | `OBS_ECHO_TIMEOUT_MS`, `OBS_RANGE_MAX_MM` | manual | Photo the module markings; range at 10 distances 20 mm to 1.5 m against a tape, motors off and at cruise PWM; time the no-echo pulse | Reading vs tape recorded at each distance; timeout set from the measured no-echo pulse |
| S5-I08 | A9: servo settle and supply | `OBS_SETTLE_PINGS` | manual | Sweep 500–2500 µs at 50 Hz; film the settle for 15° and 5° steps; watch VSYS during sweeps | Settle ms measured; `OBS_SETTLE_PINGS = ceil(settle ms / 60)` |
| S5-U01 | Profile of a centred obstacle; plan picks the smaller shift | `obs_profile.c`, FR7, FR8 | auto (host) | `make -C testing/subsystem5/host` | width, nearest and extent within a few mm; PLAN LEFT; legs = shift and nearest + depth + length; falls back to RIGHT when the left gap is too small |
| S5-U02 | Obstacle clear of the path | `obs_profile.c`, FR8 | auto (host) | as above | empty scan and an object outside the 125 mm corridor give CONTINUE |
| S5-U03 | Both sides blocked; a leg over its limit | `obs_profile.c`, NFR16 | auto (host) | as above | NO_PATH; a 334 mm shift sets `b_was_clamped`, reports 300 mm, outcome NO_PATH |
| S5-U02b | Only a run that blocks the corridor is the obstacle (review R02) | `obs_profile.c`, FR8 | auto (host) | side object at 30°/300 mm + central object at 90°/200 mm | the central object is chosen; plan is not CONTINUE |
| S5-U23 | Leg minimum (review V13-04) | `obs_profile.c`, `OBS_LEG_MIN_MM` | auto (host) | a run touching the corridor edge (shift 0) | `leg_lateral = OBS_LEG_MIN_MM`, never 0 |
| S5-U14 | Echo width → distance (review R01) | `obstacle.c` | auto (host, in S5-U08) | `obstacle_width_to_mm()` | 580 µs → 100 mm, 5800 µs → 1000 mm, saturated width → `APP_DIST_NONE_MM`, 4 mm steps |
| S5-U05 | Every state × every event type | `mission.c` statechart, B5.2.6 | auto (host) | `sim_mission_host.c` → `mission_test_matrix()` (built only with `-DMISSION_HOST_TEST`, fix brief Fix 2), fresh mocks per case | 468 combinations (18 × 26 event types since APP_EVT_BARCODE_PROBE), every outcome a defined state, no crash |
| S5-U07 | Statechart simulation (spec v1.5.1) | `mission.c`, `mission_run.c`, FR3, FR7–FR9, FR13, FR14, FR17 | auto (host) | `sim_mission_host.c` with mocked kernel, tagged motion and neighbours | all checks pass: BOOT waits for the startup check-ins, stale MOVE_DONE tags ignored, CAL_IR with MOTOR_EN, run-start ZERO handshake, barcode turn, bypass, SIDE_TOO_CLOSE → FAULT, common recovery, FAULT → BOOT, IMPACT → FAULT, retry/deadline rules (host parts of S5-U15, U16, U18, U20, U21, U22, U27) |
| S5-U08 | Obstacle task simulation (spec v1.5.1) | `obstacle.c`, NFR6, FR6–FR8 | auto (host) | `sim_obstacle_host.c` against a flat-face world | all checks pass: S5-U14 conversion, confirmation, ESTOP before OBSTACLE, measured/inferred masks, WATCH_SETTLE then PLAN, board before PLAN, NO_PATH, SIDE_TOO_CLOSE → ESTOP → STOPPED, NO_ECHO, host parts of S5-U17 (late/stale falls), S5-U24 (stuck high), S5-U25, S5-U26 (stale record) |
| S5-I01 | Stop distance at 300 mm/s | NFR6 | manual (track) | Obstacle on the line, run at 300 mm/s, video + serial log | ≥ 100 mm clearance; braking ≤ 150 ms after the first reading |
| S5-I02 | Watchdog on a hung task (superseded by S5-I02r) | NFR10b | manual (target) | Test build hangs one periodic task | Reset ≤ 100 ms |
| S5-I03 | Bypass and reacquire | FR9 | manual (track) | Obstacle on a straight; log `FR9 %d ms` | Line reacquired ≤ 5 ± 0.5 s after the end of the pass leg |
| S5-I04 | CPU, RAM, stacks | NFR11 | auto (target) | `app_stack_free_pct_min()`, map file, GP14 | CPU < 60 %, RAM < 75 %, every stack ≥ 25 % free |
| S5-U15 | Search really turns | `mission_run.c` | auto (target) | FOLLOW → LINE_LOST with the car on the floor | STOP first, then each leg on its tagged MOVE_DONE; LINE_FOUND during a leg returns to FOLLOW |
| S5-U16 | IMPACT and FAULT recovery | `mission.c`, FR13 | manual (target) | Tap the car during FOLLOW, a MOVE and a TURN | FAULT each time; RESET goes through BOOT and recalibration |
| S5-U17 | Late echo | `obstacle.c` R1 path | manual (target) + auto (host, in S5-U08) | Signal generator: a fall after the timeout, across a new trigger and across sequence wrap. Host: RISE and FALL in one R1 entry (fix brief Fix 5) | never used for the next ping; both edges in one entry never give a 0 mm echo (pin low: discarded; pin high: kept as a rise) |
| S5-U18 | Persistent fault | `mission.c` | manual (target) | Force a PIO-load failure, then RESET | stays in FAULT (bit set again every cycle) |
| S5-U19 | Minimum object width (A29) | `obstacle.c`, `obs_profile.c` | manual (track) | 120 mm and 80 mm wide objects at 3 positions | the 120 mm one is always detected |
| S5-U20 | Second run starts from zero | run-start handshake | manual (target) | Two runs in a row | odometry, run peak and summary of run 2 start from 0 |
| S5-U21 | STOP mid-bypass, then a new run | common recovery | manual (target) | STOP during WAIT_PLAN, TURN_OUT, LEG_PASS, REJOIN; RESET; START | every time the next run starts (ESTOP cleared) |
| S5-U22 | Refused command / missing ZERO ack | `mission_send_move` | manual (target) | Test hook in motion refuses twice; ZERO never acked | FAULT, never an endless wait; ZERO case after 200 ms |
| S5-U24 | Echo held high | `obstacle.c` | manual (target) | Signal generator holds ECHO high for 5 pings | no measurement accepted; NO_ECHO |
| S5-U25 | Watch-angle settle | `obstacle.c` | manual (target) | Probe PWM and motor pins at a PLAN | PLAN only after `OBS_WATCH_SETTLE_PINGS`; motors braked until then |
| S5-U26 | Sensor unplugged after a good echo | `obstacle.c` | manual (target) | Unplug the HC-SR04P | every ping reads `APP_DIST_NONE_MM` and counts as silent |
| S5-U27 | Move deadlines | `mission_move_deadline_ms` | auto (host, in S5-U07) | 800 mm at 150 mm/s, 2 047 mm, ZERO, blocked turn | ≈ 9 000 ms, > 20 s, 200 ms; blocked turn → FAULT at its deadline; FOLLOW never tracked |
| S5-U28 | Watchdog startup window | `sys_health.c`, NFR10b | manual (target) | Normal boot; then a test build with obstacle_task not started | 30 ms window only after all four first check-ins; without obstacle the board resets after 500 ms; reset cause printed at boot |
| S5-I02b | Obstacle hang | `sys_health.c` | manual (target) | Hang obstacle_task right after its check-in at several phases of mission's pass | reset ≤ 100 ms after the hang every time |
| S5-U29 | 1 µs time base runs | `sys_tick_init` (watchdog tick), `sys_tick_now_us` | auto (target) | `test_mission.c` (TEST=1): `sys_tick_now_us()`, `tk_dly_tsk(1000)`, again (fix brief Fix 4) | difference 990 000 to 1 020 000 µs, value printed |
| S5-U30 | Power-up grace for the ultrasonic | `obstacle.c`, `OBS_WARMUP_PINGS`, T10 | auto (host, in S5-U08) + manual (target) | Host: silent pings right after boot, then an echo; a sensor that never answers. Target: power up with the team firmware, no FAULT from NO_ECHO | No NO_ECHO before the first echo or within 34 pings (~2 s); after that 3 silent pings raise NO_ECHO as before; a dead sensor raises it 3 pings after the grace |
| S5-I02r | Every task hangs (revised S5-I02) | `sys_health.c`, NFR10b | manual (target) | Hang line, motion, imu, obstacle and mission right after a check-in, at 6 phases of the 60 ms period | reset ≤ 100 ms after the hang every time (normal bound 95 ms) |
| S5-U56 | Early rejoin crossing after a right bypass | FR9, APP_EVT_LINE_FOUND | auto (host) | Replay the 53/89 mm and 64/97 mm crossings in the 9 November car logs | Reject each first crossing and take the second, then return to FOLLOW |
| S5-U57 | Bounded barcode rewind, slow reread, and cross routing | FR2, FR3 | auto (host) | Start with the logged 35 mm prefix, inject tagged completion and decoder outcomes, then junction, obstacle, and STOP events | Brake, back up once, follow at 70 mm/s, restore cruise when clear, and use the mapped action at CROSS. A line end leaves the code pending (supersedes S5-U35). |
| S5-U58 | Rejoin distance and width window | FR9 | auto (host) | Replay the early 116 mm, late 511 mm and valid later crossings | No turn at the early or late mark. Valid crossings within 50–120% accepted. Supersedes S5-U56. |
| S5-U59 | Extended bypass cap with outer-echo and clearance checks | FR8, NFR16 | auto (host) | Replay the 334 mm scan, add a limiting return, and test above the cap | Allow the clear 334 mm plan. The normal cap remains 300 mm and the wall fixture stops. |
| S5-U60 | Verified white lead-in for the slow reread | FR2, FR3 | auto (host) | Replay a black sensor at 72 mm, short white gaps, a quiet white stretch, and travel exhaustion | Brake and reread only after 35 mm of continuous white behind the first reported edge. The reverse command is limited to 150 mm and the previously followed segment. |
| S5-U61 | Confirm MQTT barcode mapping after storage | FR3, APP_EVT_MAP | auto (host) | Remap F in READY and RUN, then simulate a failed store | Confirm the applied mapping in both states and only report success after storage. |
| S5-U62 | Report an unarmed cross independently of a barcode | FR3, FR19 | auto (host) | Inject CROSS with no mapped action armed | Console and dashboard record the class and the mission keeps following. |
| S5-U63 | Finish the bounded rewind before checking the white lead-in | FR2, FR3 | auto (host) | Present white before reverse completion and place a probe near a passed cross | Wait for the bounded move and both wheels to stop. Do not reverse across the previous cross. |
| S5-U64 | Defer a barcode retry during a short gap in line tracking | FR2, FR3 | auto (host) | Match the 71 mm start guard while both line sensors are white, then restore tracking or exceed a bound | Brake only when safe. Account for additional travel, and cancel on a junction or a valid first-pass decode. |
| S5-U65 | Rewind the whole strip after a failed capture | FR2, FR3 | auto (host) | Probe with 286 mm of travel (on-car 10 Nov 21:37) after 600 mm of following | Brake, then reverse 150 + 150 + 106 mm (needed distance + 70 mm), then reread at 70 mm/s after a quiet lead-in. U64's cap check now uses the 450 mm cap. |
| S5-U66 | A rewind may start while an obstacle is ahead (APPROACH) but not while passing one | FR2, FR3, FR6 | auto (host) | Probe with the scanner in APPROACH, then in PASSING | APPROACH brakes and backs up; PASSING does nothing. Replaces the S5-U57 check that skipped the rewind in APPROACH. |
| S5-U67 | While armed, the cross-junction is taken even though the sensors see it as a T shape | FR3, FR19 | auto (host) | Arm A (LEFT), then the JUNCTION event (on-car 10 Nov 22:03: centred on the line, both sensors read white past the cross-junction's stripe, as the arm ahead runs between them) | Advance 60 mm, turn left 90, disarm. |
| S5-U68 | A failed slow pass rewinds to its start and reads again, up to 2 more times | FR2, FR3 | auto (host) | Slow pass ends INVALID, then a pass ends at the cross undecoded, then a third pass ends INVALID (on-car 10 Nov 22:39: the failed pass carried on) | Each of the first two failures brakes and backs up (need + 70 mm in 150 mm moves) and reads at 60 mm/s again; the third gives up and follows at the normal speed. |
| S5-U69 | The junction log names the one junction type | FR3, FR19 | auto (host) | Inject the cross-junction, then a one-sided mark, with nothing armed | Console shows `CROSS-JUNCTION detected` (no class number) and `SIDE MARK detected (2 …)`; no turn for either. |
| S5-U74 | Status board K6 stored as the 11 packed `SB_*` words (Tribunal 1 minimum-bit change, item 6: data shared between tasks); writers and getters unchanged | Min-bit item 6, NFR11 | auto (host) | `test_status_board_host.c`: write every part at its minimum, maximum and typical values (sentinels 2044 / 1020 mm), read back through the getters; initial board; snapshot size | Every field reads back unchanged; initial board = nothing ahead, profile clear, other fields 0; snapshot 44 B; pitch beyond ±45° stored as ±45° (the ramp check compares with ±2°) |
| S5-U75 | K4 mission event is one 16-bit `MSG_EVT` word (11-bit value, 5-bit type), Tribunal 1 minimum-bit change (post-tribunal gap 3); a value above 2047 is clamped and flagged, never wrapped | Min-bit items 6, 7; K4 | auto (host) | `sim_mission_host.c` with a K4 mock using the kernel cost (4 B header + size rounded to 4): message size and buffer, six event types with values 0..2047 posted and read back, one value of 5000 | Messages 2 B, 16 entries in 128 B (S5-U04 high-water is now 128 B, was 256 B); every type and value back unchanged in FIFO order; 5000 arrives as 2047 with `APP_FAULT_RANGE_CLAMP` |
| S5-U76 | A barcode slow pass may take up to 15 s (`MISSION_BARCODE_READ_MS`) before it ends without a decode | FR2, FR3 | auto (host sim) | `sim_mission_host.c`: a slow pass with no decoder result for 10 s, then to 15 s (on-car 12 Nov 14:23: a pass through a bend decoded after 8.4 s, 0.6 s before the old 9 s limit) | Still reading after 10 s; ends at 15 s with the S5-U68 retry. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S5-U01 | `e74d623` | AfifLotfi | PASS | `logs/2026-10-21_host_tests.txt` | host run (gcc); target and track tests not run yet |
| 2026-10-21 | S5-U02 | `e74d623` | AfifLotfi | PASS | `logs/2026-10-21_host_tests.txt` | host run (gcc); target and track tests not run yet |
| 2026-10-21 | S5-U03 | `e74d623` | AfifLotfi | PASS | `logs/2026-10-21_host_tests.txt` | host run (gcc); target and track tests not run yet |
| 2026-10-21 | S5-U07 | `e74d623` | AfifLotfi | PASS | `logs/2026-10-21_host_tests.txt` | host run (gcc); target and track tests not run yet |
| 2026-10-21 | S5-U08 | `e74d623` | AfifLotfi | PASS | `logs/2026-10-21_host_tests.txt` | host run (gcc); target and track tests not run yet |
| 2026-10-21 | S5-U01 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U02 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U02b | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U03 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U23 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U07 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U08 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U14 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-21 | S5-U27 | `b0e8196` | AfifLotfi | PASS | `logs/2026-10-21_host_tests_v151.txt` | host run (gcc), spec v1.5.1; target and track tests not run yet |
| 2026-10-22 | S5-U01 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U02 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U02b | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U03 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U23 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U14 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U07 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U27 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U08 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 |
| 2026-10-22 | S5-U04 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5; now a host test (Fix 3) |
| 2026-10-22 | S5-U05 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5; now a host test (Fix 2), 432 cases |
| 2026-10-22 | S5-U17 | `7d4af51` | AfifLotfi | PASS | `logs/2026-10-22_host_tests_fix1.txt` | host run (gcc) after merging main and fix brief 1-5 (host part only: Fix 5 cases); signal-generator part not run yet |
| 2026-10-22 | S5-U08 | `e123850` + diag | AfifLotfi | FAIL | on-car diagnostic build (LED-coded fault bit) | NO_ECHO latched at every boot: echo edges reach R1 (all in the window, none skipped), but the first pings after power-up get no echo at all (sensor not up yet) |
| 2026-10-22 | S5-U30 | (this branch) | AfifLotfi | PASS | `make -C testing/subsystem5/host` | host: 4 new checks pass, S5-U24/U26 unchanged; on-car confirmation pending |
| 2026-11-09 | S5-U56 | before fix (`6d303b8`) | Dylan | FAIL | Mission host simulation | The 89 mm crossing was rejected |
| 2026-11-09 | S5-U56 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | Both second crossings accepted and FOLLOW restored |
| 2026-11-09 | S5-U57 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | Bounded rewind and slow reread, all four mapped actions at CROSS, invalid and timeout exits, obstacle stop and STOP. |
| 2026-11-09 | S5-U58 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | Rejected the logged 116 and 511 mm marks. Accepted the later in-range crossings. |
| 2026-11-09 | S5-U59 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Planner host test and 11 logged scans | All scans planned with the guarded cap. The flat wall and over-limit fixtures stayed stopped. |
| 2026-11-09 | S5-U60 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | White gap, edge activity, maximum rewind, and direction-change checks passed. Track verification pending. |
| 2026-11-09 | S5-U61 | before fix (`6d303b8`) | Dylan | FAIL | Mission host simulation | The map was sent to storage without a confirmation in READY or RUN. |
| 2026-11-09 | S5-U61 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | Both states confirmed a stored update. A failed store produced no success event. |
| 2026-11-09 | S5-U62 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | The unarmed cross emits its class without a turn. |
| 2026-11-09 | S5-U63 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | The reread waits for bounded rewind completion and a quiet lead-in. A probe too close to the previous cross cannot start a rewind. |
| 2026-11-09 | S5-U64 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Mission host simulation | A 71 mm guard can wait for the line to be tracked again within the time and rewind limits. Cross and complete decode cancel the wait. |
| 2026-11-10 | S5-U65, S5-U66 | `barcode-transition-settle` | Hanz | FAIL | Mission host simulation | The 286 mm probe and the probe during APPROACH were both ignored (7 checks failed). |
| 2026-11-10 | S5-U64, S5-U65, S5-U66 | `barcode-reread-full-strip` | Hanz | PASS | Mission host simulation | Whole-strip rewind in three reverse moves, rewind allowed in APPROACH. Full subsystem 5 run: 55 + 174 + 8 + 77 pass. |
| 2026-11-10 | S5-U67 | before fix | Hanz | FAIL | Mission host simulation | The armed car ignored the T, so no advance and no turn (3 checks failed). |
| 2026-11-10 | S5-U67 | `barcode-reread-full-strip` | Hanz | PASS | Mission host simulation | T while armed advances and turns the mapped way. Full subsystem 5 run: 55 + 176 + 8 + 77 pass. |
| 2026-11-10 | S5-U68 | before fix | Hanz | FAIL | Mission host simulation | With no retries the failed pass resumed following. |
| 2026-11-10 | S5-U68 | `barcode-reread-full-strip` | Hanz | PASS | Mission host simulation | Two retries, the cross does not block the rewind. Full subsystem 5 run: 55 + 187 + 8 + 77 pass. |
| 2026-11-11 | S5-U62, S5-U69 | before fix (`5efabbd`) | Dylan | FAIL | Mission host simulation | Console showed `JUNCTION detected class 2`; telemetry code 2 for the cross (3 checks failed). |
| 2026-11-11 | S5-U57, S5-U62, S5-U67, S5-U68, S5-U69 | `one-cross-junction-type` | Dylan | PASS | Mission host simulation | Full subsystem 5 run: 55 + 5 + 189 + 8 + 77 pass. |
| 2026-11-12 | S5-U74 | before fix (`745abc0`) | Dylan | FAIL | Status board host test | Snapshot was five natural-type structs, not 44 B of packed words; pitch stored unbounded (2 checks failed). |
| 2026-11-12 | S5-U07, S5-U08, S5-U74 | `pack-status-board-words` | Dylan | PASS | Status board host test, mission and obstacle simulations | Packed board: 8 + 189 + 8 + 77 pass, same totals as before the change. |
| 2026-11-12 | S5-U75 | before fix (`745abc0`) | Dylan | FAIL | Mission host simulation | 12 B messages in a 256 B buffer, values not limited (3 checks failed); S5-U04 and the 189 simulation checks pass on both. |
| 2026-11-12 | S5-U04, S5-U07, S5-U75 | `pack-mission-event-word` | Dylan | PASS | Mission host simulation | 189 + 11 pass; K4 high-water = whole 128 B buffer. |
| 2026-11-12 | S5-U76 | before fix (`745abc0`) | Dylan | FAIL | Mission host simulation | The pass ended at 9 s (1 check failed). |
| 2026-11-12 | S5-U57, S5-U68, S5-U76 | `barcode-reread-time-limit` | Dylan | PASS | Mission host simulation | Mission simulation 191 pass. |

## SPEC-QUESTIONs raised (2026-10-22, fix brief from Buddy 2)

- **S5-U04 and S5-U05 moved from target to host: live side effects.** On the
  robot the TEST=1 suites run beside the real mission_task. The S5-U05 matrix
  sent a real 360° CAL_IR turn with MOTOR_EN, re-armed the watchdog and its
  "undo" cleared a real ESTOP, faults and modes. S5-U04 assumed usermain at
  priority 1, but D.15 step 4 raises it to 21, so mission drained K4 during
  the flood and the fake SUMMARY events reached the real mission. Both now run
  in `sim_mission_host.c`; the spec's test list should say "auto (host)".
- **Q1 answered: include order cannot fix the SDK/kernel clash.** `size_t`
  (kernel `long`, SDK `unsigned int`) and the peripheral base macros clash in
  either order. Every B5 `.c` file is now kernel side (register access with
  `in_w`/`out_w`/`set_w`/`clr_w`) or SDK side (no kernel header), per the
  build rule in `docs/BARR_C_DEVIATIONS.md`. `hmi_init()` now returns `bool`
  so mission_task raises `APP_FAULT_PIO_LOAD` (B5.1).
- **New test S5-U29 (timer tick, Fix 4).** Nothing in the template starts the
  RP2040 watchdog tick that drives the TIMER, so `sys_tick_init()` now starts
  it (12 cycles of the 12 MHz clk_ref = 1 µs). This affects the whole template
  (the radio driver uses the same timer). Measurement on the board pending.
- **NO_ECHO at power-up (S5-U30, 2026-10-22).** On the car the HC-SR04P stays
  silent for the first pings after power-up, which latched NO_ECHO and sent
  the car to FAULT at every boot. `obstacle.c` now ignores silence until the
  first echo edge or `OBS_WARMUP_PINGS` (34 pings, ~2 s), whichever comes
  first. This adds one rule to B5.2.3 / T10, and one private counter (a
  candidate PRV_OBS field for Phase 4; 6 bits).

