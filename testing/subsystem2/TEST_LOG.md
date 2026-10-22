# Subsystem 2 — test log

**Subsystem:** Motion control, safety interlock, fault bitmask  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Every odometry conversion matches a hand calculation; heading (+313, −313) ≈ +10 009 cdeg and (−313, +313) ≈ −10 009 cdeg (±1 %); heading (0, 0) = 0; a negative wheel difference stays negative | NFR3, D.6 | auto (host) | `make -C testing/subsystem2/host` | `[TEST] S2-U01 PASS` (×17) |
| S2-U02 | Step on the first-order model `speed += (duty/14 − speed)/4`: duty within ±1000, integrator within ±`MOTION_I_CLAMP`, change per step ≤ ramp, within 5 % of the set-point from tick 50 | NFR2 | auto (host) | `make -C testing/subsystem2/host` | `[TEST] S2-U02 PASS` (×4: 36, 21, −14, 43 cpw) |
| S2-U11 | Speed window: ±63 stored, ±64 clamped; normal window = sum; 3 late ticks (span 140 ms) rescale to 50 ms; empty = 0; dt clamped to 63 ms; index wraps without `%` | NFR2 | auto (host) | `make -C testing/subsystem2/host` | `[TEST] S2-U11 PASS` (×7) |
| S2-U13 | Fault mask K12: set ENCODER and STALL, clear STALL → only ENCODER; out-of-mask bits ignored; clear all → 0 | min-bit item 5, K12 | auto (host, isolated flags) | `make -C testing/subsystem2/host` (`test_flags_host.c`: real `fault.c` on a mock event flag with the kernel's OR/AND semantics) | `[TEST] S2-U13 PASS` (×7) |
| S2-U14 | Safety flag K3: set from a task → active; ISR set adds IMPACT; clear → not active; out-of-mask bits ignored | FR13, K3 | auto (host, isolated flags) | `make -C testing/subsystem2/host` (`test_flags_host.c`) | `[TEST] S2-U14 PASS` (×7) |
| S2-I07 | `motion_move_backward(151)` returns false (NFR16); every other out-of-range argument is refused, nothing sent | NFR16, D.6 | auto (target) | `make … TEST=1` | `[TEST] S2-I07 PASS` (×2) |
| S2-U19 | **Proposed (spec change).** One impossible encoder read (5000 counts in 10 ms) does not advance progress, path or position and does not complete a MOVE; two in a row raise `APP_FAULT_ENCODER` and resync without the jump entering the position; after a late (30 ms) glitched read, the valid recovery read is judged over the 40 ms since the last trusted read: accepted, no fault, no movement lost | NFR3, NFR16 | auto (host sim) | `make -C testing/subsystem2/host` (`test_motion_task_host.c`) | `[TEST] S2-U19 PASS` (×10) |
| S2-I09 | A4 counts per rev: raw counts printed each second, turn each wheel 10 times by hand | A4 | manual (bench) | Phase 1 measurement | value → `APP_COUNTS_PER_REV` (SHARED PR) |
| S2-I05 | A5 / NFR10c: meter on GP8–GP11 through reset until motion_task runs: all low; Robo Pico motor buttons: M1 = left | A5, NFR10c | manual (bench) | Phase 1 measurement | all low; `MOTION_L/R_REVERSED` set |
| S2-I08 | A20 braking: open loop to steady 250 and 300 mm/s, brake, count until 5 zero-delta cycles; 10 runs each | A20, NFR6 | manual (bench) | Phase 1 measurement | `MOTION_BRAKE_DECEL_MM_S2 = v²/(2d)`, worst case → Buddy 5 at once |
| — | Wheel diameter and wheel centre-to-centre distance | A4 | manual (ruler/calliper) | Phase 1 measurement | `APP_WHEEL_CIRC_UM`, `APP_TRACK_UM` (SHARED PR) |
| S2-U03 | Command sent while MOVING waits in the latch; STOP brakes at once | FR15 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-U12 | STOP during FOLLOW and during MOVE: braked within one cycle | FR15, NFR10 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-U15 | `path2` within 1000 half-counts of saturation, then `motion_move_forward(1000)` completes with MOVE_DONE | R03 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-U16 | Two runs: `motion_reset_odometry()` in BRAKE zeroes positions and path and posts MOVE_DONE with its tag; during FOLLOW it waits in the latch | R03 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-U17 | `APP_MODE_MOTOR_EN` clear: `motion_follow(250)` never drives; setting it later does not start the dropped command | FR17 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-U18 | STOP cancels a MOVE: no MOVE_DONE for it; the next MOVE's MOVE_DONE carries the new tag | R06 | auto (host sim) + target | host: `test_motion_task_host.c`; target: Phase 3 (needs B5 `sys_tick` and `APP_MODE_MOTOR_EN`) | PASS |
| S2-I01 | Speed settles ±5 % within 0.5 s, plot logged | NFR2b | manual (bench) | PID tuning log | PASS |
| S2-I02 | 1 m straight × 10: ±3 % | NFR3a | manual (floor) | motion-accuracy table | PASS |
| S2-I03 | 90° and 180° × 10: ±5° and ±8° | NFR3b, c | manual (floor) | motion-accuracy table | PASS |
| S2-I04 | Safety flag → motor inputs low ≤ 15 ms | NFR10a | probe (`PROBE=1`, GP14) | scope | ≤ 15 ms |
| S2-I06 | Blocked wheel → brake + STALL after 300 ms | FR15 | manual (bench) | hold a wheel | STALL event, fault bit |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Design note: PI integral gain (S2-U02)

The spec's starting value `MOTION_KI_Q8 = 256` fails S2-U02 on the spec's own
model: while the duty is limited by the 50 permille/tick ramp the integrator
keeps growing and reaches its ±300 permille clamp in about 9 ticks, so the
speed overshoots by about 30 % and is still 8 % high at 0.5 s. A sweep of the gains
on the same model (KP 0…4096, KI 0…256; all four set-points must pass):

| KP (Q8) | KI = 8 | KI = 16 | KI = 32 | KI ≥ 48 |
|---|---|---|---|---|
| 2048 – 3072 | pass, worst 0.7 % | pass, worst 2.3 % | pass, worst 4.5 % (marginal) | fail |

Chosen: KP 2560 (spec value), KI 16: strong integral action with a clear
margin (KI 32 passes only by 0.5 %), so steady-state drift on the real robot
is still removed. Both stay
`TUNE` values: retune on the bench in S2-I01.

## Design note: untrusted encoder reads (S2-U19, review finding)

The spec (B2.2.4 step 2) pushes every delta into the window, path and
progress, and raises `APP_FAULT_ENCODER` only after two impossible reads in a
row. A review reproduced the gap: a single read of 5000 counts per wheel
completed a 1 m MOVE (`MOVE_DONE`) while the fault stayed clear. Fixed in
`motion_sense_wheel()`: an impossible read counts as zero movement and the
last trusted count is kept, so the next read is judged against it. On the
second impossible read in a row the fault is raised and the counts are
resynchronised, with `raw0` moved by the same jump, so position stays
continuous and a real counter jump cannot lock the wheel out for ever. The
host simulation fails 5 of the 6 S2-U19 checks on the previous code and
passes on the fixed code.

Follow-up review: after a rejected read the kept count was compared with
only the last cycle's time, so a valid recovery read 10 ms after a late
(30 ms) glitched read held 40 ms of movement (~34 counts at 300 mm/s)
against a 10 ms limit (30 counts): false ENCODER fault and lost distance.
Fixed: each wheel keeps the time of its last trusted count, and both are
updated together on acceptance and on resync. The new late-cycle checks
(S2-U19, 4 more) fail 3 of 4 on the previous code and pass on the fix.

Raised to the team as a SPEC-QUESTION: B2.2.4 step 2 needs the rejection
rule (zero movement for an untrusted read, count and time baselines kept
as a pair per wheel, resync with ENCODER on the second read) and the test
list needs S2-U19.

## Design note: why S2-U13 / S2-U14 are host tests

On the robot the K3/K12 flags are live system state, and the TEST=1 suites
run while every task, including mission, is running. Injecting ENCODER/STALL
there erased a pre-existing STALL fault (reproduced in review) and, once
mission runs, would drive it into FAULT, which restoring the bits cannot
undo. The tests therefore run the real `safety.c` and `fault.c` on isolated
flag objects (`mock_kernel.c`, the kernel's OR/AND semantics; a broken
`fault_clear` without `~` fails 3 checks). The on-target suite keeps only
S2-I07, which cannot change system state.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S2-U01, S2-U11 | `<commit>` | | PASS | `logs/2026-10-22_host_odometry_speed_pid.txt` | 24 checks, placeholder MEASURE constants |
| 2026-10-22 | S2-U02 | `<commit>` | | PASS | `logs/2026-10-22_host_odometry_speed_pid.txt` | 4 set-points, KI = 16 |
| 2026-10-22 | S2-U01, U02, U11, U13, U14, U03, U12, U15–U19 | `<commit>` | | PASS | `logs/2026-10-22_host_all_v2.txt` | 64 host checks after the review fixes |
| 2026-10-22 | S2-U01, U02, U11, U13, U14, U03, U12, U15–U19 | `<commit>` | | PASS | `logs/2026-10-22_host_all_v3.txt` | 68 host checks after the follow-up fix (late-cycle recovery) |
