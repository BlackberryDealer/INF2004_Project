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
| S3-U04 | `code39_decode`: speed variation at 150 mm/s and 250 mm/s with ±15 % element timing jitter | FR2, NFR5 | auto | Host (`test_code39_host.c`) | Returns `CODE39_OK` and correct index |
| S3-U05 | `code39_decode`: rejection of `< 29` elements, missing `*` delimiters, wide inter-char gap, `**` data char, and collapsed ratio (`wide/narrow < 1.8`) | FR2, NFR5 | auto | Host (`test_code39_host.c`) & target (`test_barcode.c`) | Returns `CODE39_INVALID` |
| S3-U06 | `line_track_cal_finish` & `line_track_normalise`: polarity-agnostic calibration (`min_l < max_l` and `min_r > max_r`), span `< 400` rejection | FR1, NFR1 | auto | Host (`test_line_track_host.c`) & target (`test_line.c`) | Both polarities accepted; span `< 400` rejected |
| S3-U07 | Immediate decode on the 29th element (`n == 29`) without waiting for the 30 mm post-barcode quiet zone | FR2, R08 | auto | Host (`test_line_barcode_sim_host.c`) | `APP_EVT_BARCODE` posted on first `barcode_cycle()` after 29th edge |
| S3-U08 | `barcode_map_set`: out-of-range `char_index >= 43` and `cmd >= 4` ignored; valid remap and new slot visible to next decode | FR3 | auto | Host (`test_line_barcode_sim_host.c`) & target (`test_barcode.c`) | Invalid calls ignored; valid mappings applied |
| S3-U09 | Rejoin mode (`APP_MODE_JUNCTION_INHIBIT` without `LINE_LOST`): reports both a rejected 10 mm stray mark and a subsequent 45 mm real line | FR1, F-02 | auto | Host (`test_line_barcode_sim_host.c`) | Two `APP_EVT_LINE_FOUND` events (~10 mm then ~45 mm) |
| S3-U14 | Both barcode-branch arrival orders (`b_branch_recent` early before `CANDIDATE`, and late `branch_seq` change during `CONFIRM`) | FR19, F-03 | auto | Host (`test_line_track_host.c`, `test_line_barcode_sim_host.c`) & target (`test_line.c`) | Both early and late branch arrivals set barcode-side saturation |

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
