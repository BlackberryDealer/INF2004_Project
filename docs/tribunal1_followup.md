# Tribunal 1 follow-up: where the code differs from the document

Status on 2026-11-28 (first written 2026-11-16, updated 2026-11-18 and
2026-11-28) of the Tribunal 1 decisions (minimum-bit data representation,
`CSC28_PostTri1.docx`) against the submitted firmware. Each item records
its implementation, evidence, and any hardware check not completed before
the car was dismantled on 2026-11-25.

## 1. MAP only outside RUN (fixed 2026-11-23)

**Tribunal text (Version C, gap 7):** "MAP is 2 bytes and accepted only
outside RUN. Only mission_task writes the table, one byte per update, so
barcode_task never reads a half-written entry."

**Before:** `mission_forward()` (`app_program/subsystem5/mission.c`)
stored a MAP in every state, RUN included, and S5-U61 expected the update
to be confirmed in RUN.

**Now:** `mission_forward()` refuses a MAP while the system state is RUN.
The table is left unchanged, no MAP confirmation is sent, and the console
shows `MAP refused in RUN`. In every other state the update is stored and
confirmed as before. S5-U61 checks both (failing on `3c5daac`, passing
after the fix).

**Operating rule:** send the MAP commands for the letters announced on
the day while the car is in READY, before START. The decoder still
identifies every letter A to Z during the run, mapped or not.

**Status (2026-11-28):** Verified on host in `S5-U61` (PASS in the
2026-11-28 host run). On the car on 25 Nov, `CMD_MAP` in `READY` was
exercised 12 times in the course captures; the on-car `MAP`-while-`RUN`
refusal trial was not run before the car was dismantled on 2026-11-25.

## 2. Changes made on 2026-11-16 that close Tribunal items

| Item | Change | Evidence |
|---|---|---|
| Gap 2, "still to add": reject a clamped gap | A 29-element frame that holds a gap (after the first bar) at the 15-bit clamp (about 524 ms) is not decoded. The capture is not dropped, so rereads and rewinds behave as on the proven code (14c3dc1) | S3-U48. The earlier drop-the-capture version (S3-U47, branch `barcode-clamped-gap-reject`) changed reread behaviour in traces and is not used |
| No text values over MQTT | The console-line (LOG) and SENSORS debug payloads are sent only by a `DEBUG_MQTT=1` image; the normal image sends only binary payloads | S1-U38 run for both builds; `docs/message_sizes.md` |
| Kernel object table | The console mirror buffer is K13 in `TEAM.md` (it was also called K12, the ID of `FLG_FAULT`) | `TEAM.md` section 5 |
| Only the bits needed | Junction code is 2 bits (codes 0..3 since `b3c4505`), in `SB_LINE0` and `EVT_JUNCTION` | `tools/layout.csv`; build check in `line.c` |
| Every event value has a defined width (gap 6) | `EVT_INVALID` (6 bits) and `EVT_BARCODE_PROBE` (11 bits) rows; the INVALID element count now reaches the laptop instead of being sent as 0 | `tools/layout.csv`, `telem.c` |

## 2a. Changes made on 2026-11-18

| Item | Change | Evidence |
|---|---|---|
| Only the bits needed: Buddy 2 motion state | `motion_task.c` statics and the speed window packed (`PRV_MOT_*`, `PRV_WIN_*`), commit `69dccb1` | host tests; car runs of 18 Nov on `tribunal1-car-test` |
| Only the bits needed: Buddy 3 late additions | `barcode.c`: FIX_CNT, SETTLE, START_PATH and the REREAD_FIRST / PROBE_OK / QUIET_KNOWN flags in the new word `PRV_BAR_C`; the flicker CARRY in the free 15 bits of `PRV_BAR_B`; the raw edge log indices in `PRV_BAR_LOG`; the ISR edge counter is the 16-bit `SH_BAR_EDGES`. `line.c`: REJOIN_WHITE, CROSS_STRT, BRANCH_SEEN, LAST_JCT and RAW_LOG_CNT in the free 11 bits of `PRV_LINE_A`; WHITE_MARK and CROSS_MARK in `PRV_LINE_E` / `PRV_LINE_F` | The line + barcode simulation (all its scenarios) prints the same console lines and posts the same events, byte for byte, before and after; host suites, layout and style checks pass |
| Only the bits needed: Buddy 4 late additions | `imu.c`: the per-cycle values (calibrated averages, forward acceleration, pitch, hump now, turn rate, encoder heading, magnetometer images) and the calibration / turn-source flags in `PRV_IMU_G`..`PRV_IMU_M`. Each row states the range the arithmetic can produce, so storing never clamps | The old and new `imu.c` were driven with the same random sensor, encoder, IR and mode streams (23 runs of 20 000-30 000 cycles, with calibration, lifts, humps, I2C failures and full-scale values): every board write, event, safety stop and console line was identical |
| Shared data marked as shared | The barcode map rows `PRV_BAR_MAP` are class S: mission_task writes one byte per MAP and barcode_task reads whole bytes | `tools/layout.csv` |
| Generated layout, not hand packing | `mission_evt_barcode_value()` packs with the `EVT_BARCODE` macros; a build check keeps `EVT_MAP` identical | `mission.c` |
| Change Requirement 2 document | The IR level-lift path (S4-U17) is described with its rule, threshold history and the 18 Nov car results | `docs/change_requirement_2_lift_stop.md` section 3.1 |
| RAM evidence | Section 5 of `docs/ram_comparison.md`: car image `ae78cf5` -> `9dcc102` -> this change, with the symbol deltas | `tools/ram_report.sh`, `tools/ram_symbols.txt` |
| Echo record uses `MSG_ECHO` (Buddy 5) | `obstacle_echo_isr` packs WIDTH / SEQ / VALID into the 16-bit `MSG_ECHO` word and stores it at once; obstacle_task reads it with one load (the read-twice loop is gone). 12 B -> 2 B | Obstacle and mission host simulations print identical output (logs included) before and after; build check ties the ISR limits to the row |
| Only the bits needed: obstacle_task (Buddy 5) | About 25 statics in `PRV_OBS_M/I/A/B/C`; scan table and samples as 8-bit 4 mm codes; the board part kept in the board's own `SB_OBS0..2` layout (12 B instead of 44 B). Rows for the ISR-shared bytes (one writer per byte) and E5 timers | Obstacle and mission simulations identical; 20 randomized runs x 30 000 cycles of old vs new `obstacle.c` identical (board, events, ESTOP, faults, servo, logs) |
| Only the bits needed: HMI and health (Buddy 5) | `PRV_MIS_G` (buttons, LED, buzzer, last state, beep), `PRV_HLT` (watchdog window as 1 bit, enabled, started) | Randomized old-vs-new runs of `hmi.c` (10 x 100 000 polls) and `sys_health.c` (8 x 50 000 passes) identical |
| Only the bits needed: mission state (Buddy 5) | `mission_ctx_t` stored packed between passes (`PRV_MIS_A..R`, 15 words, 390 bits, plus 12 E5 timers); each pass works on an unpacked local copy, like the status board part structs | Mission simulation identical with every pass round-tripped; check S5-PACK: 20 000 random states over every row range round-trip unchanged |

## 2b. Integration, car verification and submission status (25–28 Nov 2026)

| Item | Change / milestone | Evidence |
|---|---|---|
| Packed image merged and run on car | `tribunal1-packing` merged to `main`; `SMP=0` course image built at `f4e94ad` (SHA-256 `072aa8b78f9cf6e8…`, adding the 4-byte `g_imu_prv_r` IMU calibration-retry word). Subsequent unit-constant (`ee23a2e`) and BARR-C:2018 (`6899aa0`) merges reproduce the exact same UF2 binary byte-for-byte | `testing/TEST_SUMMARY_2026-11-25.md`; `testing/system/logs/2026-11-25_*_mqtt.csv` |
| On-car functional check of packed firmware | Full-track run (13:20–13:22, 10.5 m in 119 s, 0 faults), Code 39 barcode decodes (`S3-I02`), obstacle stop/scan/bypass (`S5-I01`, `S5-I03`, `S5-I09`), lift-stop 3/3 (`S4-I12`/`SYS-06`), hump events (`S4-I02`/`SYS-04`), wheel stall (`S2-I06`), IMU bus fault (`S4-I05`), dashboard control (`SYS-02`), and 5 Hz binary MQTT with 0 debug/text payloads (`S1-I01`, 24 211 live messages on `f4e94ad`) | `testing/TEST_SUMMARY_2026-11-25.md`, `videos/README.md`, `docs/message_sizes.md` |
| Final host and static verification | Full host suite re-run on `22a73b4` (firmware and test sources identical to `6899aa0`): 955 PASS, 0 FAIL (including `S5-PACK`, `S5-U61`, `S5-U85`, `S1-U38`). `tools/gen_layout.py --check tools/layout.csv` (397 rows OK) and `tools/barr_check.sh` (101 files, no findings) | `testing/integration/logs/2026-11-28_host_all_22a73b4.txt`; per-subsystem `2026-11-28_host_22a73b4.txt` logs |
| Final RAM and message size records | Final `SMP=0` course ELF section/symbol sizes (`94272 B` static SRAM; `1669 B` application state/buffers across all 5 subsystems) and 25 Nov live broker payload counts recorded | `docs/ram_comparison.md` section 7; `docs/message_sizes.md` |

## 3. Open items at submission (2026-11-28)

The state of every task follows the minimum-bit rule (natural types only
for the E1–E8 exceptions and per-pass working copies). Items left open at
submission:

**Driver handles (Buddy 5, minor).** Set once at init and never changed:
the buzzer and servo PWM slice / channel (`uint32_t`, 3 + 1 bits needed),
the WS2812 state machine number and ready flag, the alive-LED flag and
the GPIO IRQ table. Left unpacked as init-only driver handles.

**Buddy 3 test switch.** `gb_bar_reread_always` in `barcode.c` stays a
`bool`: it is a build setting (`BAR_REREAD_ALWAYS`) that only the host
simulation changes, never written by the firmware.

**Hardware measurements not completed before car dismantling (2026-11-25).**
The car was dismantled at 13:30 on 25 Nov after the demo runs, so the
following hardware-only checks remain unrecorded:
- On-car `MAP`-refused-in-`RUN` trial (section 1; host-verified in `S5-U61`).
- `app_stack_report()` baseline vs packed console logs (`docs/ram_comparison.md` section 3).
- `S1-I10` `w10-baseline` broker size capture, Wireshark frame captures, and GP14 `PROBE=1` scope timing (`docs/message_sizes.md`).
- Hang-injection watchdog tests (`S5-I02`/`INT-08`) and open demo-day hardware defects indexed in `testing/TEST_SUMMARY_2026-11-25.md`.

Natural types kept on purpose are not in this list: they are the
exceptions E1-E8 in `docs/BARR_C_DEVIATIONS.md` (D5), pending the
professor.
