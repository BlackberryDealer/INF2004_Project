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
| S5-U05 | Every state × every event type | `mission.c` statechart, B5.2.6 | auto (target) | `test_mission.c` → `mission_test_matrix()` on a scratch context, **wheels lifted** | 432 combinations, every outcome a defined state, no crash |
| S5-U07 | Statechart simulation (whole run) | `mission.c`, `mission_run.c`, FR3, FR7–FR9, FR14 | auto (host) | `sim_mission_host.c` with mocked kernel and neighbours | all 54 checks: calibration sweep, barcode turn, arm expiry, bypass legs, rejoin window and width, heading fix, RESUME 150 ms, inhibits after 150 mm, NO_PATH reverse then FAULT, fault/RESET rules, line search timeout |
| S5-U08 | Obstacle task simulation | `obstacle.c`, NFR6, FR6–FR8 | auto (host) | `sim_obstacle_host.c` against a flat-face world | all 25 checks: mm conversion, confirmation, ESTOP before OBSTACLE, board before PLAN, scan → PLAN/NO_PATH, watch angle, SIDE_TOO_CLOSE once, RESUME, NO_ECHO after 3 silent pings, 2 lost echoes → ESTOP |
| S5-I01 | Stop distance at 300 mm/s | NFR6 | manual (track) | Obstacle on the line, run at 300 mm/s, video + serial log | ≥ 100 mm clearance; braking ≤ 150 ms after the first reading |
| S5-I02 | Watchdog on a hung task | NFR10b | manual (target) | Test build hangs one periodic task | Reset ≤ 100 ms |
| S5-I03 | Bypass and reacquire | FR9 | manual (track) | Obstacle on a straight; log `FR9 %d ms` | Line reacquired ≤ 5 ± 0.5 s after the end of the pass leg |
| S5-I04 | CPU, RAM, stacks | NFR11 | auto (target) | `app_stack_free_pct_min()`, map file, GP14 | CPU < 60 %, RAM < 75 %, every stack ≥ 25 % free |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
