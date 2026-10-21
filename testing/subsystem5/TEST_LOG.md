# Subsystem 5 — test log

**Subsystem:** Shared services, obstacles and the mission (Buddy 5)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U04 | Flooding K4 never blocks a producer | K4 `mission_post_event`, min-bit item 7 | auto | `test_mission.c`: usermain (priority 1) posts 20 events while mission_task (16) cannot drain | 16 accepted, 4 refused, each refusal < 200 µs, `APP_FAULT_QUEUE_OVERFLOW` set, high-water = 256 B |
| S5-U06 | Release-latency counter moves under load | `sys_tick_cycle_start`, NFR17 | auto | `test_mission.c`: a priority-11 task busy-loops 2 ms at tick boundaries for 40 rounds | `sys_tick_latency_max_us()` rises and reaches ≥ 1500 µs |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
