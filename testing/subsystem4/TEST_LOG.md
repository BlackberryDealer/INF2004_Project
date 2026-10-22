# Subsystem 4 — test log

**Subsystem:** IMU, terrain and the RAM report (Buddy 4)
**Student:** Faris Zharfan, 2500841
**Code:** `app_program/subsystem4/` · **Host tests:** `testing/subsystem4/host/`
**Branch:** `b4-imu` · commit prefix `S4:`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change (spec B4).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Terrain Monitor replay of a recorded hump log | FR4, FR5, NFR7 | auto (host) | `make -C testing/subsystem4/host terrain` | One hump; peak within ±20 % or ±5 mm of the ruler value |
| S4-U02 | Ramp longer than `TERRAIN_MAX_CLIMB_MM` | FR4 | auto (host) | host replay, synthetic ramp | No hump reported |
| S4-U03 | Integer `atan2` vs `math.h` on a 1 deg grid | B4.2.1 | auto (host) | host grid comparison (may use double) | Error ≤ 0.2 deg |
| S4-U06 | Long hump at 150 mm/s (synthetic) | FR4 (distance, not time, decides ramp) | auto (host) | host replay, synthetic 150 mm/s log | Hump is not rejected as a ramp |
| S4-U07 | Synthetic slip on one wheel | NFR7 | auto (host) | host replay, one wheel reads double travel | Height changes ≤ 10 % |
| S4-U12 | `a_fwd` on a logged speed trace | B4.2.2 step 5 | auto (host) | speed history replay | Matches the hand calculation |
| S4-U13 | Run start: peak reset and odometry rebase | R03 / B4.2.2 step 4 | auto (target) | `test_imu.c`, `APP_MODE_RUN_START` 30 ms | Run peak returns to 0; `ds = 0`, no false climb |
| S4-U14 | FIFO overrun and backlog handling | R14 / B4.1.1 | auto (host) | `drv_lsm303_fifo_plan` on FIFO_SRC images | Overrun: no samples used, counter +1; backlog of 6 → 4 now and 2 next cycle |
| S4-U10 | Packed pitch pack/unpack (Phase 4) | SB_IMU0_PITCH | auto (host) | `app_bits_encode`/`decode` at −45, 0, +45 deg, clamp beyond | Exact stored values; clamp sets the flag |
| S4-U11 | Hump 127/128 clamp sets RANGE_CLAMP (Phase 4) | SB_IMU0_HUMP_* | auto (target) | `test_imu.c` with the packed status board | 127 passes; 128 sets `APP_FAULT_RANGE_CLAMP` |
| S4-I01 | Flat cruise, no false hump | NFR7 | track | 3 runs at cruise | No hump event |
| S4-I02 | Humps 15/25/40 mm, short and long, 150 and 250 mm/s | FR4, FR5, NFR7 | track | 5 runs each, CSVs committed | All detected within ±20 % or ±5 mm |
| S4-I03 | Impact taps from front/back/left/right | FR13 | bench | 5 taps each side; log `INT1_SRC_A`; with Buddy 2 | INT1 fires every time; motors low ≤ 15 ms |
| S4-I04 | 10 on-the-spot turns: IMU vs encoder rate | A7 | floor | turn left/right, compare rates | Within 10 % or flagged `b_is_turn_from_enc` |
| S4-I05 | Unplug SDA | B4.2.2 step 2 | bench | pull SDA during RUN | Fault after 3 errors; recovers on reconnect |
| S4-I06 | FIFO burst read equals single reads; identity | B4.1.1 verify wrap | bench | burst of 4 vs 4 single reads; read IRA/IRB/IRC | Equal samples; identity `H43` |
| S4-I07 | A18 pull-ups on SDA/SCL | A18 | bench | meter to 3.3 V, power off | 2–10 kΩ |
| S4-I08 | Axis signs | B4.1.2 | bench | tilt nose up; flat | `ax` rises → `IMU_X_FORWARD_SIGN`; flat `az` ≈ +1000 mg |
| S4-I09 | I2C time on GP14 | B.3 budgets | bench | GP14 around FIFO read | Record µs for 4 samples and magnetometer; ≤ budget |
| S4-I10 | Magnetometer hard-iron offsets | R15 | bench | level, 360 deg slow turn, log mx/my | Offsets into `cfg_imu.h` |
| S4-I11 | Mag vs encoder rate after S4-I10 | A7 | bench | 10 on-the-spot turns | Within 10 % |

*Method:* **auto** = `TEST_CHECK` / `HOST_CHECK` in code, result printed;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S4-U01, U02, U03, U06, U07, U12, U13, U14 | Phase 2 commit | Faris | PASS (host) | `logs/2026-10-22_host_phase2.txt` | terrain 18/18, fifo planner 6/6; S4-U13 pure parts (mode-driven half needs B5's mission) |
| 2026-10-22 | S4-U13, normal+TEST=1 builds | Phase 2 commit | Faris | PASS (target build) | `logs/2026-10-22_host_phase2.txt` | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.elf` and `_test.elf` build with zero warnings |
| 2026-10-22 | S4-U01, U02, U03, U06, U07, U12, U13, U10 | Phase 4 commit | Faris | PASS (host, packed) | `logs/2026-10-22_host_phase4_packed.txt` | terrain 19/19 (incl. S4-U10 pitch pack), fifo 6/6; packed terrain.c/imu.c build into normal and TEST=1 images with zero warnings |
| 2026-10-22 | S4-U11 | — | Faris | pending | — | hump 127/128 → RANGE_CLAMP lives in Buddy 5's packed status-board writer (and imu.c raises it when the run peak reaches 127); runs once B5 merges |
| 2026-10-22 | RAM report (Phase 5) | Phase 5 commit | Faris | PASS (script) | `docs/ram_comparison.md` | real baseline-vs-packed ELFs from this branch: B4 statics net −63 B; section 3 stays pending until on-car `app_stack_report()` logs exist |
| 2026-10-22 | S4-I01, I02, I03, I04, I09 (Phase 3) | — | Faris | pending | — | integration on the car: INT1→motors low ≤15 ms with Buddy 2 (GP14), course humps vs ruler, imu_task WCET. Blocked on hardware; plan below |
| 2026-10-22 | S4-I01–I11 | — | Faris | pending | — | bench/track: needs the physical car (placeholders kept, spec rule A.3) |

## SPEC-QUESTION (to the team chat before merging Phase 4)

**SQ-B4-1: E_IMU_SAMPLE group.** B4.4 lists the three E_IMU rows in one
group with mixed `group_bits` (32, 32, 16). Buddy 1's `gen_layout.py`
rejects mixed `group_bits` inside a group *before* its class-E exemption,
so the 16-bit `RAW_SAMPLE` row now sits in its own documentation group
`E_IMU_SAMPLE`. No macros are generated for either group, so nothing
functional changes. Please decide: relax the generator's group check for
class E rows (Buddy 1), or keep the split group as committed.

## Phase 3 integration plan (pending hardware)

- **INT1 → motors low ≤ 15 ms:** with Buddy 2's motion merged, tap the
  chassis; probe GP14 and the motor pins. The path is
  R1 → `imu_int1_isr` → `safety_set_from_isr(SAFETY_IMPACT)` → motion's
  step 3 brake, inside one 10 ms cycle (B4.2.2 step 3 timing note).
- **Humps on the course vs a ruler:** run S4-I02 profiles; the terrain
  monitor's integrated peak was host-validated at 20 mm ± 5 mm.
- **imu_task WCET:** GP14 around the three paths (2 samples, magnetometer
  cycle, 4-sample backlog) plus the I2C-failure path; record in the WCET
  table for Phase 5.
- The mode-driven half of S4-U13 (mission sets `APP_MODE_RUN_START` for
  30 ms) validates once Buddy 5's mission.c is merged; imu_task's
  run-start handler and the `terrain_ds_counts` rebase guard are already
  in place.

