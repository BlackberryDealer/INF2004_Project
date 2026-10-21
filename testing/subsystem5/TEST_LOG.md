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
| S5-I07 | A8: HC-SR04P at 3.3 V, range and timeout | `OBS_ECHO_TIMEOUT_MS`, `OBS_RANGE_MAX_MM` | manual | Photo the module markings; range at 10 distances 20 mm to 1.5 m against a tape, motors off and at cruise PWM; time the no-echo pulse | Error within ±10 mm up to 1.5 m; timeout value set from the measured no-echo pulse |
| S5-I08 | A9: servo settle and supply | `OBS_SETTLE_PINGS` | manual | Sweep 500–2500 µs at 50 Hz; film the settle for 15° and 5° steps; watch VSYS during sweeps | Settle ms measured; `OBS_SETTLE_PINGS = ceil(settle ms / 60)` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
