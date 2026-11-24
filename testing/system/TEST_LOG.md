# System — test log

Tests of the complete PicoCar against its functional and non-functional requirements on the final hardware set-up. To minimise recording overhead per the module coordinator's guideline (*videos only where a `.txt` log alone cannot prove physical behaviour*), the manual track/bench checks are grouped into **3 consolidated short clips** alongside saved `.txt`/`.csv` logs.

See `testing/README.md` and `videos/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Boot, stationary IMU calibration, 360° IR sweep calibration, and `READY` indication | FR1, FR16, FR17, NFR10b | `.txt` log + Clip 1 start | Power on PicoCar on the track line; observe white `BOOT` -> blinking yellow `CAL_IMU`/`CAL_IR` -> solid green `READY` | Motors stay off in `BOOT`/`CAL_IMU`; `CAL_IR` spans >= 400 codes; enters `READY` with 0 stop-faults |
| SYS-02 | Remote dashboard `START`/`STOP`/`RESET`/`SUMMARY` & 5 Hz live telemetry | FR10, FR11, FR12, NFR8 | Clip 1 + broker `.csv` log (`S1-I01`, `S1-I12`) | Click `START` on `tools/dashboard/index.html` (1 s blue-blink buzzer warning -> `RUN`), `SUMMARY` mid-run, `STOP` at end -> `FINISHED` -> `RESET` -> `READY` | Commands take effect within 1 s; `c28/s` steady at 5 Hz ± 10 %; `SUMMARY` frozen after `STOP` |
| SYS-03 | Cruise line following + full-scale & half-scale Code 39 barcode reading + cross-junction routing | FR1, FR2, FR3, FR19, NFR4, NFR5 | Clip 1 + serial/MQTT `.txt` (`S3-I02`, `S3-I04`, `S3-U55`) | In one continuous track run, cross a full-scale barcode -> cross-junction #1 and a half-scale barcode (with automatic rewind/reread if triggered) -> cross-junction #2 | Both barcodes appear in dashboard `Barcode result` (`Run reads (2)`) and car executes each mapped turn at its cross-junction |
| SYS-04 | Hump height measurement vs ruler & ramp rejection (no false hump, no false lift-stop) | FR4, FR5, FR13, NFR7 | Clip 1 + serial/MQTT `.txt` (`S4-I01`, `S4-I02`, `S4-I13`) | Place ruler beside track hump; drive across flat, hump, and > 400 mm ramp in Clip 1 | Dashboard `Hump height` within ±20 % or ±5 mm of ruler; ramp reports 0 humps; 0 false `LIFT STOP` |
| SYS-05 | Obstacle stop clearance (>= 100 mm), servo profile width (`cm`/`mm`), collision-free bypass & line rejoin (`FR9 <= 5.0 ± 0.5 s`) | FR6, FR7, FR8, FR9, NFR6, NFR16 | Clip 1 + serial/MQTT `.txt` (`S5-I01`, `S5-I03`) | Place box on straight track with ruler/mark; car stops, scans, bypasses, and rejoins line | Stops >= 100 mm from box; top card shows width (`cm` & `mm`) and `FR9` rejoin <= 5.5 s; zero contact |
| SYS-06 | Lift-stop safety response (`RUN -> FINISHED` in <= 100 ms, green blink, `RESET -> READY`) | FR13 (Change Req 2), FR16, NFR10a | Clip 2 (`S4-I12`) + serial `.txt` | During `RUN`, lift car straight up and tilted; observe immediate wheel stop & green blink; hold `RESET` 2 s (or click `RESET`) | Wheels brake within 100 ms (`LIFT STOP` logged); enters `FINISHED` (not `FAULT`); `RESET` returns to `READY` |
| SYS-07 | Fault & link-loss resilience: Wi-Fi AP off mid-run continues driving (`S1-I05`), wheel stall (`S2-I06`) & I2C unplug (`S4-I05`) enter `FAULT` and recover on `RESET` | FR13, FR16, NFR9c | Clip 3 + serial `.txt` | 1) Turn hotspot off during `RUN`: car keeps following line (`WIFI_LOSS` info bit only). 2) Hold wheel 300 ms -> `STALL` (`FAULT`, red blink + alarm) -> `RESET`. 3) Pull SDA in `RUN` -> `FAULT` -> reconnect & `RESET` | Car unaffected by Wi-Fi loss; both physical faults cut motor PWM immediately, raise red alarm, and recover cleanly via `RESET -> BOOT` |
| SYS-08 | Timing, memory, coding-standard & `TEST=1` bench verification (WCET, ADC noise, stack margin, RAM packing, BARR-C) | NFR1, NFR5c, NFR11, NFR13, NFR17 | auto `.txt` logs (`TEST=1`, host suites, `barr_check`, `gen_layout`) | Run `TEST=1` image on Pico (`S3-I01`, `S3-I03`, `S3-I05/I06`, `S4-I09/I10`, `S5-I04`, `S5-U06/U29`), `tools/gen_layout.py --check`, `tools/barr_check.sh`, and all S1–S5 host suites | `line_task` 10 ± 1 ms (`WCET <= 300 us`), decode `<= 50 ms`, IR noise `< 1 %`, stacks `>= 25 %` free, 0 BARR-C findings |

*Method:* **auto** = `TEST_CHECK` / script assertion in `.txt` log; **Clip 1–3** = 3 short videos (`videos/`) covering all physical motion/clearance/LED checks.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | SYS-08 (host, layout, BARR-C & `TEST=1` diag suites) | `c96481d` | Lucas | PASS | [`2026-11-24_host_all_suites.txt`](../subsystem1/logs/2026-11-24_host_all_suites.txt) | All S1–S5 host unit/simulation suites, dashboard UI ledger checks (`S1-U50`), `TEST=1` diagnostic harnesses (`S3-U56`, `S4-U20`), layout check (386 rows), and BARR-C check (95 files) pass with 0 findings. |
