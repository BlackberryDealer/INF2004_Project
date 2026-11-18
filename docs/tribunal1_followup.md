# Tribunal 1 follow-up: where the code differs from the document

Status on 2026-11-18 (first written 2026-11-16) of the Tribunal 1 decisions (minimum-bit data
representation, `CSC28_PostTri1.docx`) that the code does not yet follow
word for word. Each item says why, and what is still to do.

## 1. MAP is accepted during RUN (kept on purpose)

**Tribunal text (Version C, gap 7):** "MAP is 2 bytes and accepted only
outside RUN. Only mission_task writes the table, one byte per update, so
barcode_task never reads a half-written entry."

**Code:** `mission_forward()` (`app_program/subsystem5/mission.c`) stores a
MAP in every state, RUN included. Test S5-U61 (added 2026-11-09) expects
the update to be confirmed in READY and in RUN.

**Why it is kept:**

- The reason the Tribunal gave still holds in RUN. `barcode_map_set()`
  writes one byte of `g_bar_map` from mission_task, and barcode_task reads
  whole bytes, so a half-written entry cannot be read.
- A remap does not change a turn that is already armed: `run_arm()` copies
  the command when the barcode is decoded, so a MAP only affects barcodes
  read after it.
- MAP in RUN was used in the car run of 2026-11-12 (subsystem 5 test log).
  Refusing it would change tested behaviour (S5-U61) without a car run to
  confirm it, so the change is left for the team to decide.

**To do:** the team confirms this deviation, or changes `mission_forward()`
to ignore MAP while `APP_SYS_RUN` and updates S5-U61, then re-tests on the
car.

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

## 3. Still open (left until the car can be tested)

These change the code that drives the car, so they wait for a car run.

**Buddy 5 (mission, obstacle, HMI).** `mission_ctx_t` (about 70 fields),
the obstacle-task statics and the `g_obs_scan` array, and the HMI statics
are in natural types. (The ultrasonic echo record now uses the `MSG_ECHO`
word: section 2a.)

**Buddy 3 test switch.** `gb_bar_reread_always` in `barcode.c` stays a
`bool`: it is a build setting (`BAR_REREAD_ALWAYS`) that only the host
simulation changes, never written by the firmware.

**Evidence.** `docs/ram_comparison.md` section 3 still needs the stack logs
of the baseline and packed images (tests T4, T6).

**Car check of the 18 Nov packing.** The B3/B4 packing is shown equal to the
car-tested code on the host (section 2a), but has not been on the car yet:
one barcode + turn, one obstacle, one quick and one level lift, and one
ramp run on the new image before merging to `main`.

Natural types kept on purpose are not in this list: they are the
exceptions E1-E8 in `docs/BARR_C_DEVIATIONS.md` (D5), pending the
professor.
