# Tribunal 1 follow-up: where the code differs from the document

Status on 2026-11-16 of the Tribunal 1 decisions (minimum-bit data
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

## 3. Still open (left until the car can be tested)

These change the code that drives the car, so they wait for a car run.

**Buddy 2 (motion).** Private state in `motion_task.c` (about 40 statics)
and the speed window `odometry_window_t` (15 x 32-bit values, 64 B) are in
natural types. The Tribunal decision was a small signed encoder delta per
tick. The layout rows `PRV_MOT_*` / `PRV_WIN_*` named in the file headers
do not exist yet.

**Buddy 5 (mission, obstacle, HMI).** `mission_ctx_t` (about 70 fields),
the obstacle-task statics and the `g_obs_scan` array, and the HMI statics
are in natural types. The ultrasonic echo record still uses a three-field
struct instead of the `MSG_ECHO` word that `tools/layout.csv` defines.

**Buddy 3 and Buddy 4 (later additions).** Statics added after the Phase 4
packing are natural width and have no layout row, for example
`g_bar_carry_16us`, `g_bar_settle_16us`, `g_bar_start_path` and several
`bool` flags in `barcode.c`, `g_white_mark` in `line.c`, and
`g_pitch_ddeg`, `g_turn_dps` in `imu.c`.

**Evidence.** `docs/ram_comparison.md` section 3 still needs the stack logs
of the baseline and packed images (tests T4, T6).

Natural types kept on purpose are not in this list: they are the
exceptions E1-E8 in `docs/BARR_C_DEVIATIONS.md` (D5), pending the
professor.
