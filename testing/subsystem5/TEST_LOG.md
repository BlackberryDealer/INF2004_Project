# Subsystem 5 — test log

**Subsystem:** Shared services, obstacles and the mission (Buddy 5)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U04 | Flooding K4 never blocks a producer | K4 `mission_post_event`, min-bit item 7 | auto | `test_mission.c`: usermain (priority 1) posts 20 events while mission_task (16) cannot drain | 16 accepted, 4 refused, each refusal < 200 µs, `APP_FAULT_QUEUE_OVERFLOW` set, high-water = 256 B |
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
| S5-U05 | Every state × every event type | `mission.c` statechart, B5.2.6 | auto (target) | `test_mission.c` → `mission_test_matrix()` on a scratch context, **wheels lifted** | 432 combinations, every outcome a defined state, no crash |
| S5-U07 | Statechart simulation (spec v1.5.1) | `mission.c`, `mission_run.c`, FR3, FR7–FR9, FR13, FR14, FR17 | auto (host) | `sim_mission_host.c` with mocked kernel, tagged motion and neighbours | all checks pass: BOOT waits for the startup check-ins, stale MOVE_DONE tags ignored, CAL_IR with MOTOR_EN, run-start ZERO handshake, barcode turn, bypass, SIDE_TOO_CLOSE → FAULT, common recovery, FAULT → BOOT, IMPACT → FAULT, retry/deadline rules (host parts of S5-U15, U16, U18, U20, U21, U22, U27) |
| S5-U08 | Obstacle task simulation (spec v1.5.1) | `obstacle.c`, NFR6, FR6–FR8 | auto (host) | `sim_obstacle_host.c` against a flat-face world | all checks pass: S5-U14 conversion, confirmation, ESTOP before OBSTACLE, measured/inferred masks, WATCH_SETTLE then PLAN, board before PLAN, NO_PATH, SIDE_TOO_CLOSE → ESTOP → STOPPED, NO_ECHO, host parts of S5-U17 (late/stale falls), S5-U24 (stuck high), S5-U25, S5-U26 (stale record) |
| S5-I01 | Stop distance at 300 mm/s | NFR6 | manual (track) | Obstacle on the line, run at 300 mm/s, video + serial log | ≥ 100 mm clearance; braking ≤ 150 ms after the first reading |
| S5-I02 | Watchdog on a hung task (superseded by S5-I02r) | NFR10b | manual (target) | Test build hangs one periodic task | Reset ≤ 100 ms |
| S5-I03 | Bypass and reacquire | FR9 | manual (track) | Obstacle on a straight; log `FR9 %d ms` | Line reacquired ≤ 5 ± 0.5 s after the end of the pass leg |
| S5-I04 | CPU, RAM, stacks | NFR11 | auto (target) | `app_stack_free_pct_min()`, map file, GP14 | CPU < 60 %, RAM < 75 %, every stack ≥ 25 % free |
| S5-U15 | Search really turns | `mission_run.c` | auto (target) | FOLLOW → LINE_LOST with the car on the floor | STOP first, then each leg on its tagged MOVE_DONE; LINE_FOUND during a leg returns to FOLLOW |
| S5-U16 | IMPACT and FAULT recovery | `mission.c`, FR13 | manual (target) | Tap the car during FOLLOW, a MOVE and a TURN | FAULT each time; RESET goes through BOOT and recalibration |
| S5-U17 | Late echo | `obstacle.c` R1 path | manual (target) | Signal generator: a fall after the timeout, across a new trigger and across sequence wrap | never used for the next ping |
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
| S5-I02r | Every task hangs (revised S5-I02) | `sys_health.c`, NFR10b | manual (target) | Hang line, motion, imu, obstacle and mission right after a check-in, at 6 phases of the 60 ms period | reset ≤ 100 ms after the hang every time (normal bound 95 ms) |

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
