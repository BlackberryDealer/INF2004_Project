# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | K1 echo ISR → obstacle | K1 | auto (host) + manual (car) | S5-U08/U17 simulate valid and delayed echoes. On the car: compare the scanner output against measured boxes in SYS-05. | Valid range reaches the obstacle state machine and late echoes are discarded. |
| INT-02 | K2 barcode ISR → decoder | K2 | auto (host) + manual (car) | S3-U07/U46/U55 exercise edge capture, overflow and both print scales. On the car: match the printed letters with received events in SYS-03. | An overflow must discard the frame. |
| INT-03 | K3 safety → motion | K3 | auto (host) + manual (car) | S5-U78 and S2 motion tests exercise LIFT and braking separately. On the car: SYS-06; measure the delay. | SYS-06 must show lift, wheel stop, FINISHED and RESET. |
| INT-04 | K4 mission event queue | K4 | auto (host) | S5-U04 fills the 16-event queue. Save the corresponding host assertions. | The next post returns immediately and sets the overflow fault. |
| INT-05 | K5 mission → motion | K5 | auto (host) + manual (car) | S5-U07/U27 exercise mission commands and completion tags. On the car: compare the commanded and actual turn at a cross in SYS-03. | `[TEST] S5-U07 PASS` and `[TEST] S5-U27 PASS` |
| INT-06 | K6 shared status | K6 | auto (host) + manual (car) | S5-U74 round-trips packed subsystem fields and sentinels. On the car: compare a live snapshot with the matching dashboard state. | `[TEST] S5-U74 PASS` |
| INT-07 | K7 mission → telemetry | K7 | auto (host) + manual (car) | S1-U02/U42 check packed events, profiles and summaries. On the car: SYS-02 and SYS-05. | `[TEST] S1-U02 PASS` and `[TEST] S1-U42 PASS`; SYS-02 and SYS-05 must receive the run summary and box profile at the broker. |
| INT-08 | K8 task check-ins → watchdog | K8 | manual (bench) | In the documented S5-I02r hang-injection setup, stop each task checking in. Record the reset cause and elapsed time. Ordinary boot alone covers startup only. | Verify the watchdog resets within 100 ms for each injection. |
| INT-09 | K9 PIO encoder → motion | K9 | auto (host) + manual (car) | S2-U01/U31 exercise odometry and wrap using simulated counts. On the car: drive a measured distance and save actual wheel counts, direction and odometer. | `[TEST] S2-U01 PASS` and `[TEST] S2-U31 PASS` |
| INT-10 | K10 mission mode flags | K10 | auto (host) + manual (car) | S3-U44 and S5-U07 exercise reread and motion mode transitions. On the car: observe calibration, RUN and STOP in SYS-01/02. | `[TEST] S3-U44 PASS` and `[TEST] S5-U07 PASS` |
| INT-11 | K11 telemetry → radio | K11 | auto (target) + manual (broker) | S1-U03 on the test image exercises queue capacity. S1-I01 measures delivery at the broker. Collect 120 s of broker data at 5 Hz and exercise link loss in SYS-07. | `[TEST] S1-U03 PASS`; S1-I01 as defined in the subsystem 1 log. |
| INT-12 | K12 shared faults | K12 | auto (host) + manual (car) | S4-U21 injects an IMU read failure. S5-U78 exercises the separate lift-stop path. On the car: record fault state and RESET recovery in SYS-07. | `[TEST] S4-U21 PASS` and `[TEST] S5-U78 PASS` |
| INT-13 | K13 diagnostic console → radio | K13 | manual (bench) | Build the separate debug profile with `DEBUG_MQTT=1`. Compare a distinctive USB message against its decoded `c28/l` copy. Save both captures and the matching line. The normal course image has the console mirror disabled. | The USB message and its decoded `c28/l` copy match. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | INT-01–13, historical host coverage | `c96481d` | Lucas | Host suite passed; partial integration evidence | [Branch host capture](../subsystem1/logs/2026-11-24_host_all_suites.txt) | Branch diagnostics used different S4/S5 IDs. Evidence applies to that revision. Target, physical checks and both owners' sign-off remain separate. |
| 2026-11-25 | INT-01 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S5-U08, S5-U17. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-02 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S3-U07, S3-U46, S3-U55. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-03 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S5-U78, S2-U03. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-04 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S5-U04. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-05 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S5-U07, S5-U27. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-06 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S5-U74. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-07 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S1-U02, S1-U42. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-09 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S2-U01, S2-U31. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-10 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S3-U44, S5-U07. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-11 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S1-U11, S1-U12. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-12 | `72482f3` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt) | S4-U21, S5-U78. See case table for any remaining car work. Owner sign-off pending. |
| 2026-11-25 | INT-01 | `14ed65a` (debug UF2) | Dylan (owners' sign-off pending) | PARTIAL (on car) | [S5-I07 / S5-I01 rows](../subsystem5/TEST_LOG.md) | Echo readings match the ruler at 200/300/500 mm and the obstacle task stopped the car at 224 mm. |
| 2026-11-25 | INT-09 | `14ed65a` (debug UF2) | Dylan (owners' sign-off pending) | PASS (on car, counts) | [S2-I09 rows](../subsystem2/TEST_LOG.md) | PIO encoder counts reach motion: 619 and 626 counts/rev for 10 hand turns. |
| 2026-11-25 | INT-03, INT-05 | `14ed65a` (debug UF2) | Dylan (owners' sign-off pending) | PARTIAL (on car) | [S5-I01](../subsystem5/TEST_LOG.md), [SYS-06/07](../system/TEST_LOG.md) | Obstacle and stall stop the motors; lift-stop unreliable (IMU defect). Delay not measured. |
| 2026-11-25 | INT-02, INT-08, INT-13 | — | Dylan | NOT RUN | — | No full track, IMU fix in progress and no scope/meter on demo morning (30 cm track section only). |
| 2026-11-25 | INT-12 | `f4e94ad` (course UF2) | Dylan (owners' sign-off pending) | PARTIAL (on car) | [S4-I05 row](../subsystem4/TEST_LOG.md) | IMU bus fault reaches the shared fault word (0x004) and mission enters FAULT; power-cycle recovers. RESET-only recovery not tried. |
| 2026-11-25 | Closing note | — | Dylan | — | [Summary](../TEST_SUMMARY_2026-11-25.md) | On-car evidence added for INT-01, 03, 05, 09, 12 (rows above). INT-02 (barcode → decoder) is covered on car by S3-I02 (decodes on the track). INT-08 and INT-13 need special builds and were not run. Owners' sign-off still pending for every row. Car dismantled after the 13:22 run; no further measurement possible. |
| 2026-11-28 | INT-01–07, INT-09, INT-10, INT-12 (host assertions re-run) | `22a73b4` | Dylan (host run) | PASS (referenced host assertions only) | [Raw output](logs/2026-11-28_host_all_22a73b4.txt) | `make -B -C testing/host`, MSYS2 gcc 15.2.0 (firmware and test sources identical to `6899aa0`): 955 PASS, 0 FAIL; every host assertion named in the case table for these IDs prints PASS. Not INT-11: its case-table checks are S1-U03 (`TEST=1` image) and S1-I01 (broker), which this run does not cover; only the related host checks S1-U11 and S1-U12 print here (as in the 25 Nov row). INT-08 and INT-13 have no host part. First log kept in `integration/logs/`; on-car status unchanged (25 Nov rows). |
