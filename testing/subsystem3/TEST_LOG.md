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
