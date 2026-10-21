# Subsystem 1 — test log

**Subsystem:** Communications, telemetry and packing tools  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U20 | `app_bits_put`/`get` round-trip every SB_/MSG_ field at min, max and 0; neighbouring fields unchanged | D.4, D.11 | auto (host) | `make -C testing/subsystem1/host` | `[TEST] S1-U20 PASS` |
| S1-U21 | `app_bits_encode` clamps −1001 → 24 (steering) and sets the flag; 1000 → 2024 no flag; NULL flag pointer accepted | D.4 | auto (host) | `make -C testing/subsystem1/host` | `[TEST] S1-U21 PASS` |
| S1-U22 | Writer/reader round-trip of 3+5+18+11 bits at maximum; state header byte 0 = `1 \| (0 << 3)` | D.4 | auto (host) | `make -C testing/subsystem1/host` | `[TEST] S1-U22 PASS` |
| S1-U23 | Writing past the buffer sets overflow and writes nothing | D.4 | auto (host) | `make -C testing/subsystem1/host` | `[TEST] S1-U23 PASS` |
| S1-U24 | `gen_layout.py` on a broken copy of the CSV (field too wide) exits 1 | B1.0.2 | auto (host) | `make -C testing/subsystem1/host` | `[TEST] S1-U24 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S1-U20..U24 | `<commit>` | Hanz | PASS | `logs/2026-10-21_host_app_bits.txt` | 15 checks + S1-U24 |
