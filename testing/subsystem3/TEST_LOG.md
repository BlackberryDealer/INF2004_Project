# Subsystem 3 — test log

**Subsystem:** Line following, junctions and barcodes  ·  **Owner:** Buddy 3 (Lucas)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-I05 | A1 IR contrast: raw ADC over black and white at 3, 5, 8 mm height, room light and torch | A1, FR1 | manual (bench) | Phase 1 measurement (`test_line.c`) | Difference ≥ 1241 codes (1.0 V); sets `LINE_BLACK_IS_HIGH` |
| S3-I06 | A19 ADC noise: 1000 samples on one surface with ultrasonic pinging | A19, NFR1 | manual (bench) | Phase 1 measurement (`test_line.c`) | Std-dev < 1 % of black-white span |
| S3-I07 | A3 / A12 barcode DO edges: edge times over sample barcodes at 2–4 mm height; 20 powered passes per scale at 150 and 250 mm/s | A3, A12, FR2, NFR5 | manual (bench) | Phase 1 measurement (`drv_gpio_irq`) | Sets `BAR_DO_BLACK_IS_HIGH` and sensor height |
| S3-I08 | A22 junction traces: log `nL, nR, path, DO level` every 10 ms at 150 mm/s over every junction type of the course | A22, FR19 | manual (track) | Phase 1 measurement (`logs/junction_<type>_<n>.csv`) | Sets `LINE_JUNCTION_MIN_MM`, `LINE_JUNCTION_CONFIRM_MM`, `LINE_BRANCH_EARLY_MM` |
| S3-U01 | `line_track_normalise` & `line_track_steering`: exact endpoints 0 and 1000, midpoint 500, clamping, and `+1000` (left) / `-1000` (right) steering sign convention | FR1, NFR1, NFR4 | auto | Host (`test_line_track_host.c`) & target (`test_line.c`) | All assertions pass |
| S3-U02 | `line_track_jct_step`: Table J1 classification for `APP_JCT_SIDE_L`, `APP_JCT_SIDE_R`, `APP_JCT_T`, `APP_JCT_CROSS`, and short-patch rejection | FR19, NFR4 | auto | Host (`test_line_track_host.c`) & target (`test_line.c`) | All assertions pass |
| S3-U03 | `code39_decode`: all 43 valid `*<char>*` characters in forward and reverse scan order | FR2, NFR5 | auto | Host (`test_code39_host.c`) & target (`test_barcode.c`) | Returns `CODE39_OK` and exact index `0..42` |
| S3-U04 | `code39_decode`: half-scale widths decode all 43 valid `*<char>*` characters in forward and reverse scan order | FR2, NFR5 | auto | Host (`test_code39_host.c`) | Returns `CODE39_OK` and exact index `0..42` |
| S3-U05 | `code39_decode`: rejection of `< 29` elements, missing `*` delimiters, wide inter-char gap, and collapsed ratio (`wide/narrow < 1.8`); table uniqueness and 3-bit checks | FR2, NFR5 | auto | Host (`test_code39_host.c`) & target (`test_barcode.c`) | Returns `CODE39_INVALID`; table checks pass |
| S3-U06 | `line_track_cal_finish` & `line_track_normalise`: polarity-agnostic calibration (`min_l < max_l` and `min_r > max_r`), span `< 400` rejection | FR1, NFR1 | auto | Host (`test_line_track_host.c`) & target (`test_line.c`) | Both polarities accepted; span `< 400` rejected |
| S3-U07 | Immediate decode on the 29th element (`n == 29`) without waiting for the 30 mm post-barcode quiet zone | FR2, R08 | auto | Host (`test_line_barcode_sim_host.c`) | `APP_EVT_BARCODE` posted on first `barcode_cycle()` after 29th edge |
| S3-U08 | `barcode_map_set`: out-of-range `char_index >= 43` and `cmd >= 4` ignored; valid remap and new slot visible to next decode | FR3 | auto | Host (`test_line_barcode_sim_host.c`) & target (`test_barcode.c`) | Invalid calls ignored; valid mappings applied |
| S3-U09 | Rejoin mode (`APP_MODE_JUNCTION_INHIBIT` without `LINE_LOST`): reports both a rejected 10 mm stray mark and a subsequent 45 mm real line | FR1, F-02 | auto | Host (`test_line_barcode_sim_host.c`) | Two `APP_EVT_LINE_FOUND` events (~10 mm then ~45 mm) |
| S3-U10 | Packed `MSG_EDGE` 16-bit ring entry (`g_bar_ring`, 128 B): pack/unpack at gaps `0`, `1`, `0x7FFE`, `0x7FFF`, and saturation (`> 0x7FFF`) | B3.4, T1 | auto | Host (`test_line_barcode_sim_host.c`) & target (`test_barcode.c`) | Exact round-trip and saturation to `0x7FFF` |
| S3-U11 | Dropped edge detection from the packed `MSG_EDGE_LEVEL` bit: consecutive identical levels beyond `BAR_LEVEL_FIX_MAX` discard capture to `IDLE` | B3.4, FR2 | auto | Host (`test_line_barcode_sim_host.c`) | Capture discarded (`STATE == IDLE`, `N_ELEM == 0`) |
| S3-U14 | Both barcode-branch arrival orders (`b_branch_recent` early before `CANDIDATE`, and late `branch_seq` change during `CONFIRM`) | FR19, F-03 | auto | Host (`test_line_track_host.c`, `test_line_barcode_sim_host.c`) & target (`test_line.c`) | Both early and late branch arrivals set barcode-side saturation |
| S3-U15 | Rejoin at 45° (`gb_rejoin_white`): white cycles followed by grey edge cycles then black crossing still post `APP_EVT_LINE_FOUND` | FR1, F-02 | auto | Host (`test_line_barcode_sim_host.c`) | `APP_EVT_LINE_FOUND` (~25 mm) posted |
| S3-U16 | `line_track_steer_step`: low-pass smoothing, derivative damping, and last-seen-side recovery on white | FR1, NFR4 | auto | Host (`test_line_track_host.c`) | All 8 smoothing, damping, and recovery assertions pass |
| S3-U17 | LM393 comparator chatter filter in `barcode_task`: `< 0.4 mm` flicker merged into next element and up to 3 mid-flicker level fixes per capture | FR2, NFR5 | auto | Host (`test_line_barcode_sim_host.c`) | Both mid-flicker level and 1.5 ms bounce captures decode `'D'` |
| S3-U18 | Small & large barcode robustness: per-character separate bar/space thresholding (`code39_decode`) + 29-element sliding window (`barcode.c`) decode all 43 characters under optical bar/space width bias, 40% speed change, and leading stray edges | FR2, NFR5b | auto | Host (`test_code39_host.c`, `test_line_barcode_sim_host.c`) | All 43 characters decode forward & reverse; sliding window recovers after stray edges |
| S3-I01 | Line sample period 10 ± 1 ms | NFR1 | manual (probe) | Probe GP14 around `line_task` | Period 10 ± 1 ms, WCET ≤ 300 µs |
| S3-I02 | Barcode accuracy: 20 passes per print scale at 150 and 250 mm/s | FR2, NFR5b | manual (track) | 20 passes full-scale + 20 passes half-scale | ≥ 95 % correct |
| S3-I03 | Barcode decode latency ≤ 50 ms after final bar | NFR5c | manual (probe) | Probe GP7 final edge to `APP_EVT_BARCODE` | ≤ 50 ms |
| S3-I04 | Cruise line-following stability with Buddy 2: 3 runs at 250 mm/s | FR1, NFR4 | manual (track) | 3 track runs with `FOLLOW` | No line loss > 200 ms |
| S3-I10 | R1 GPIO ISR WCET on GP7 while ultrasonic pings (both tags) | B3.3, B3.4 | manual (probe) | Signal generator on GP7 with ultrasonic active | R1 total WCET < 20 µs |
| S3-U23 | Barcode prefix and bounded reread handshake | FR2, FR3 | auto (host) | Replay the 11:41 widths in the IRQ and task at cruise and reread speeds | A guarded prefix requests one rewind, reverse edges are ignored, complete slow reverse F decodes with its runtime map, and 28 elements are rejected |
| S3-U24 | Reject short barcode noise with no width contrast | FR2 | auto (host) | Send four similar 1.1–1.3 mm elements at cruise | No rewind request |
| S3-U25 | Preserve barcode IRQ direction and the first reread edge | FR2 | auto (host) | Inject a pin bounce after a latched edge. Replay the full F widths starting low after a verified rewind. | Capture starts fresh and the decoder receives all 29 elements. |
| S3-U26 | Gentler recovery when both sensors go white after a curve | FR1 | auto (host) | Hold one channel black, then give 120 ms of white samples and a visible edge | Full steering on visible tape and a short coast while line-loss tracking remains active. |
| S3-U27 | Require a partial Code 39 guard for an early rewind | FR2, FR3 | auto (host) | Replay the 14:09 tape capture, mixed-width noise, and a forward guard missing its first bar | Only the partial guard triggers a rewind before the capture fails. |
| S3-U28 | Clear steering recovery at the cross stripe | FR1, FR19 | auto (host) | Drive over a cross stripe with a staggered trailing edge, then resume following tape | Straight over the stripe, no white-floor recovery from its edge, CROSS classification and normal steering afterwards. |
| S3-U29 | Retry on an optically compressed start guard | FR2, FR3 | auto (host) | Replay the measured 14:34 F guard at cruise and its 28-element capture at reread speed | The guard can request a rewind within the distance limit. The incomplete frame is rejected. |
| S3-U30 | Filter short comparator pulses before parsing the barcode | FR2 | auto (host) | Insert six short pulses ahead of real transitions in a complete F frame and replay a short leading pulse across task wakes | The full frame decodes and the short leading pulse cannot start a capture. |
| S3-U31 | Let a barcode with an intact start guard finish at cruise | FR2, FR3 | auto (host) | Replay the 11:41 F widths with a synthetic narrow first bar at cruise in both directions | Exactly one mapped F event per pass and no rewind request. |
| S3-U32 | Keep a barcode intact when the ISR filters a chatter return | FR2 | auto (host) | Insert six 400 µs pulses 5 ms before real transitions in a complete F frame | All measured elements still decode as F. |
| S3-U33 | Suppress steering from the trailing edge of a dark cross stripe | FR1, FR19 | auto (host) | Hold one side dark across two task wakes after a cross, and replay a brief band below saturation | The trailing stripe cannot steer the car. Normal steering resumes once the band clears or the hold distance ends. |
| S3-U34 | Request a bounded reread after a capture with a known start fails | FR2, FR3 | auto (host) | Replay an intact guard, then an edge loss, quiet timeout or invalid short frame. Also replay the logged eleven-element fragment and a failed slow reread. | The intact guard keeps scanning. A confirmed failure requests one rewind. The unrecognized fragment and the slow reread request none. |
| S3-U35 | Soften steering when one sensor grazes the tape | FR1 | auto (host) | Give a half-scale visible line error on each side | The steering target is a quarter of full scale on both sides. |
| S3-U36 | Print the raw barcode edges after a failed capture | FR2 | auto (host) | Drop a six-element capture at the quiet zone and run the task for 2 s | A header and one line per raw edge (task cycle, signed gap, speed used); the first bar shows its gap and speed. |
| S3-U37 | Read bars and spaces from transition timing, not from the DO level read during comparator chatter | FR2, FR3 | auto (host) | Replay the 32 logged raw edges of the 10 Nov 21:09 pass, then complete `*A*` and `*F*` frames with echo edges 1.05 and 2.1 ms after transitions and wrong DO levels on every third edge, in both directions | The logged pass keeps 21 elements starting with the `*` guard and is not dropped; every synthetic frame decodes to its mapped command. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| PENDING | S3-I05 | — | | PENDING | `logs/YYYY-MM-DD_s3_i05_contrast.txt` | Sets `LINE_BLACK_IS_HIGH` |
| PENDING | S3-I06 | — | | PENDING | `logs/YYYY-MM-DD_s3_i06_noise.txt` | Verifies ADC stability |
| PENDING | S3-I07 | — | | PENDING | `logs/YYYY-MM-DD_s3_i07_barcode_edges.txt` | Sets `BAR_DO_BLACK_IS_HIGH` |
| PENDING | S3-I08 | — | | PENDING | `logs/junction_<type>_<n>.csv` | Sets `LINE_JUNCTION_MIN_MM`, `LINE_JUNCTION_CONFIRM_MM`, `LINE_BRANCH_EARLY_MM` |
| 2026-10-22 | S3-U01 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | 7/7 normalisation & steering checks passed |
| 2026-10-22 | S3-U02 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | 6/6 junction trace classifications passed |
| 2026-10-22 | S3-U03 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | All 43 Code 39 chars forward & reverse at full scale |
| 2026-10-22 | S3-U04 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | All 43 Code 39 chars forward & reverse at half scale |
| 2026-10-22 | S3-U05 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | 7/7 pattern table & invalid frame rejections passed |
| 2026-10-22 | S3-U06 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | 8/8 endpoint & polarity-agnostic calibration checks passed |
| 2026-10-22 | S3-U07 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | Immediate decode on 29th element without quiet-zone wait |
| 2026-10-22 | S3-U08 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | `barcode_map_set` boundary & remap checks passed |
| 2026-10-22 | S3-U09 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | Rejoin mode reports both 10 mm stray mark and 45 mm line |
| 2026-10-22 | S3-U14 | Phase 2 | Lucas | PASS | `logs/2026-10-22_host_phase2.txt` | Early (`b_branch_recent`) and late (`CONFIRM`) branch orders passed |
| 2026-11-05 | S3-U01..U11, U14..U18 | Phase 4 | Lucas | PASS | `logs/2026-11-05_host_phase4_packed.txt` | 62/62 host assertions pass on packed `line.c` + `barcode.c` (128 B `MSG_EDGE` ring, `PRV_CAL_*`, `PRV_LINE_*`, `PRV_BAR_*`, split bar/space Code 39 decoder, 29-element sliding window) |
| 2026-11-09 | S3-U23 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Line and barcode host simulation | Logged prefix prompts one rewind. A complete simulated F frame uses the runtime map and incomplete or invalid frames are rejected. |
| 2026-11-09 | S3-U24 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Barcode host simulation | Four similarly narrow elements never requested a rewind. |
| 2026-11-09 | S3-U25 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Line and barcode host simulation | Latched edge direction preserved and the 29 measured elements of F decode after a falling first edge. |
| 2026-11-09 | S3-U26 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Line tracking host simulation | Full visible-curve response and gentler recovery after both channels go white. |
| 2026-11-09 | S3-U27 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Barcode host simulation | The tape capture and mixed-width noise produced no rewind. Both guard orientations and the mapped F reread passed. |
| 2026-11-09 | S3-U28 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Line task host simulation | Staggered stripe edges kept the car straight, CROSS still fired, and normal steering resumed. Track verification pending. |
| 2026-11-09 | S3-U29 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Barcode host simulation | The recorded weak F guard requests a retry. The 28-element frame remains invalid. |
| 2026-11-09 | S3-U30 | `obstacle-rejoin-barcode-reread-steering-cap` | Dylan | PASS | Barcode host simulation | Six simulated false pulses did not corrupt the full F frame. Short leading pulses were rejected across task wakes. |
| 2026-11-10 | S3-U31 | before fix (`5b07d79`) | Dylan | FAIL | Barcode host simulation | Both complete cruise passes posted an early rewind request. |
| 2026-11-10 | S3-U27, S3-U31 | `barcode-rewind-partial-guard-only` | Dylan | PASS | Barcode host simulation | Both cruise passes posted only the mapped F. A partial guard still requested a rewind. All 114 subsystem 3 checks passed. |
| 2026-11-10 | S3-U32, S3-U33 | Before pulse and stripe fixes | Dylan | FAIL | Line and barcode host simulation | The injected pulses broke the F capture. A trailing stripe steered the car and left recovery bias. |
| 2026-11-10 | S3-U28, S3-U32, S3-U33 | Subsystem 3 candidate | Dylan | PASS | Line and barcode host simulation | F decoded with the injected pulses. Both stripe cases held steering straight, and ordinary steering and CROSS still worked. All 118 subsystem 3 checks passed. |
| 2026-11-10 | S3-U34 | Before failed-capture retry | Dylan | FAIL | Barcode host simulation | Three failures after an intact start guard produced no rewind request. |
| 2026-11-10 | S3-U34 | Failed-capture retry candidate | Dylan | PASS | Barcode host simulation | A failed capture with a known start requested a rewind. Healthy scans, unrecognized fragments, and slow rereads avoided duplicate retries. All 124 subsystem 3 checks passed. |
| 2026-11-10 | S3-U35 | Before steering adjustment | Dylan | FAIL | Line tracking host simulation | Both half-scale readings targeted half of full steering. |
| 2026-11-10 | S3-U35 | Steering adjustment candidate | Dylan | PASS | Line tracking host simulation | Both half-scale readings targeted a quarter of full steering. Full steering at full contrast and line recovery still passed. All 126 subsystem 3 checks passed. |
| 2026-11-10 | S3-U36 | Before edge log | Dylan | FAIL | Barcode host simulation | No edge dump after the dropped capture. |
| 2026-11-10 | S3-U36 | `barcode-edge-log` | Dylan | PASS | Barcode host simulation | Seven raw edges printed after the drop, with the expected gap and speed. Full subsystem 3 run: 46 + 16 + 67 pass. |
| 2026-11-10 | S3-U37 | Before transition settle | Dylan | FAIL | Barcode host simulation | The logged pass was dropped (`edge missed`) and no synthetic frame decoded. |
| 2026-11-10 | S3-U37 | `barcode-transition-settle` | Dylan | PASS | Barcode host simulation | Logged pass: 21 elements, `*` guard first, no drop. `*A*` and `*F*` decode in both directions. Full subsystem 3 run: 46 + 16 + 70 pass. |
| 2026-11-10 | S3-U11, S3-U14, S3-U18, S3-U30, S3-U32, S3-U34 | `barcode-transition-settle` | Dylan | PASS | Barcode host simulation | Re-run after the capture stopped using the DO level. U11 and U34 now expect repeated levels to keep the capture. U18 runs its 0.5 mm bars at cruise speed. U30 and U32 place the false pulse 3 ms before the real edge, inside the settle window. U14 no longer sends a white edge before the branch. |

## SPEC-QUESTION

**SQ-B3-1: `E_BAR` mixed `group_bits` (8 vs 32) and 32-bit `LAST_US` width in `tools/layout.csv`:**
Section B3.4 lists the two `E_BAR` documentation rows as:
- `E_BAR,8,HEAD_TAIL,6,1,index,0,0,0,RAW,E,B3,E6 ring head (ISR) and tail (task): one writer per byte`
- `E_BAR,32,LAST_US,32,1,us,0,0,0,RAW,E,B3,E5 ISR-private last accepted edge time`

`tools/gen_layout.py` validates that all rows in a group share one `group_bits` value and that `bits` is in `1..31` *before* applying the class-`E` exemption (the same generator rule documented in Buddy 4's resolved `SQ-B4-1`). Accordingly, the second class-`E` documentation row is recorded as `E_BAR_US,32,LAST_US,31,1,us,0,0,0,RAW,E,B3,...`. Class-`E` rows emit no C macros and use `group_bits` (`32`) as their stored width in `docs/data_representation.md`, so generated code and ABI are unaffected.
