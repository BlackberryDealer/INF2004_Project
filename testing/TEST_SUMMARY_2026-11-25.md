# Test summary · 25 November 2026 (demo day)

This page indexes the testing done on demo day. Each result row lives in its `TEST_LOG.md`; this page only summarises it and points to the evidence. Rows are never edited after the fact: later rows and the dated closing notes at the bottom of each log supersede earlier ones. The car was dismantled at 13:30, so anything not measured by then is recorded as such.

**Status key:** PASS · PARTIAL (part of the criteria shown) · FAIL · INCONCLUSIVE (test did not exercise the condition) · NOT RUN (with reason).

## Firmware used

| Commit | Image | What changed | Used for |
|---|---|---|---|
| `72482f3` | `mtk3pico_smp0_…_mqtt_test.uf2` / `…_mqtt.uf2` (single-core) | Test-only diagnostics on top of `main` `14ed65a` | Bench capture, boot, SYS-02, first lift trials |
| `14ed65a` | `PicoCar_hotspot_main_14ed65a.uf2` (previous day's image, console mirrored to MQTT) | Same logic as `main` | Lift A/B check, stall, sensor bench tests |
| `f4e94ad` | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`, SHA-256 `072aa8b7…` | IMU: retry a bad calibration once, then raise `APP_FAULT_IMU` | All tests from 12:08, including the full-track runs |
| `fc9e59a` | not flashed | Mission: hand over to line search when the rejoin window passes | **Reverted in `5751975`.** Never run on the car, and review found it clears the FR9 timer on entering the search |
| `5751975` / `ca84792` | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`, SHA-256 `072aa8b78f9cf6e8…` | Revert of `fc9e59a` + final verification logs | Firmware source identical to `f4e94ad`; rebuilt via `testing/run-demo.sh course` (`tracked_firmware_edits=no`), producing byte-for-byte identical SHA-256 `072aa8b78f9cf6e817f94f259859f5730f6574644a392e159091b42d580a09a9` |

## Unit tests (host)

| Scope | Commit | Result | Evidence |
|---|---|---|---|
| S1–S5 host suites, all unit tests | `f4e94ad` | **PASS** · 947 PASS, 0 FAIL | [host log](subsystem1/logs/2026-11-25_120829_host_f4e94ad_C1q6qT.txt) |
| S1–S5 host suites, all unit tests | `5751975` (submitted HEAD) | **PASS** · 947 PASS, 0 FAIL | [host log](subsystem1/logs/2026-11-25_host_5751975_linux.txt) |
| Layout and BARR-C static checks | `5751975` | **PASS** · BARR-C 101 files, 0 findings; layout 397 rows OK | [static checks](system/logs/2026-11-25_5751975_static_checks.txt) |
| S1-U03 on the car (failed 24 Nov) | `72482f3` | **PASS** 4/4 | [bench console](system/logs/2026-11-25_104810_617_bench_72482f3.txt) |
| Single-core on-target test image (SYS-08) | `72482f3` | **FAIL** · watchdog reset during S5-U06 on every boot; S3-I01 and S4-I09 not reached | same bench console |

## Subsystem integration tests (on the car)

| ID | What | Result | Key figures |
|---|---|---|---|
| S1-I01 | Telemetry rate at the broker | **PASS** | 300 STATE per 60 s (limit 270–330) |
| S1-I03 | Broker loss and retry | PARTIAL | Car survived a broker restart, telemetry back within 1 s; detection and retry timing not observed |
| S2-I06 | Blocked wheel → stall | **PASS** | STALL + fault, motors braked, ×2 |
| S2-I09 | Encoder counts per rev | **PASS** (measured) | 619 left, 626 right (configured 640) |
| S3-I02 | Barcode decode on track | PARTIAL | 7 decodes, 3 rejected captures; printed letters not recorded |
| S3-I05 | IR contrast | PARTIAL (PASS at normal height) | ≈2890 / 2670 codes (need ≥1241) |
| S3-I06 | IR noise | PARTIAL (PASS) | 0.34 % / 0.26 % of span (need <1 %) |
| S4-I05 | IMU bus unplugged | **PASS** | IMU fault → FAULT state; power-cycle recovers |
| S4-I08 | Pitch sign | **PASS** | nose-up +17°, nose-down −18° |
| S5-I01 | Obstacle stop | PARTIAL | Stopped 224 mm from the box, no contact; not at cruise speed |
| S5-I03 | Bypass and rejoin | PARTIAL | Rejoined on the full run; failed 5 times at the same spot (see SYS-05). FR9 timing not measured |
| S5-I06 | Sensor-to-front offset | MEASURED | 40 mm; code still uses 60 mm |
| S5-I07 | Ultrasonic range | PARTIAL (PASS at tested points) | 200/300/500 mm read 196/300/505 |
| S5-I09 | Obstacle width | PARTIAL | Car reported 88, 80, 88, 116 mm; box ruler widths not recorded, so ±10 mm not verified. Unavailable scans explained by geometry (correction row). 70 mm bottle runs invalid (outside spec) |

## Interface (integration) tests

| ID | Result | Notes |
|---|---|---|
| INT-01 … INT-12 (except 08) | PASS on host | Referenced host assertions at `72482f3` |
| INT-01, 03, 05, 09, 12 | On-car evidence added | See the rows from S5-I07, S5-I01, S2-I09, S4-I05 |
| INT-02 | Covered on car by S3-I02 decodes | |
| INT-08, INT-13 | NOT RUN | Need hang-injection / console-mirror builds |
| All | Owners' sign-off pending | |

## System tests

| ID | Requirement | Result | Key figures |
|---|---|---|---|
| SYS-01 | Boot and calibration | **PASS** (log) | BOOT → CAL → READY, IR spans 3177 / 3047 |
| SYS-02 | Dashboard control | **PASS** | START → RUN → STOP → FINISHED → SUMMARY → RESET → READY |
| SYS-03 / 05 / 02 | Full-track run | **PASS** (no fault) | 13:20–13:22: barcode M decoded ×2, obstacle 116 mm passed and rejoined (17 s from the stop; FR9 timing not measured), junction taken, 10.5 m in 119 s, 0 faults. Turn direction and rejoin branch not written down |
| SYS-05 | Obstacle one tile after a bend | **FAIL** 5 of 6 | REACQUIRE_FAIL: rejoin window passed with no crossing. Open defect (attempted fix reverted) |
| SYS-06 | Lift-stop | FAIL before fix, **PASS** after | 0/6 gentle lifts before `f4e94ad`; 3/3 stops after (≤100 ms bound not measured) |
| SYS-07 | Faults | PARTIAL | Stall PASS; IMU bus fault PASS; Wi-Fi-loss run NOT RUN |
| SYS-04 | Hump and ramp | NOT VERIFIED | HUMP events (3–22 mm) logged, no ruler reference |
| SYS-08 | Diagnostics | Host PASS, bench FAIL | See unit tests |

## Defects found today

| Defect | Evidence | Status |
|---|---|---|
| IMU sometimes starts dead after a reset (raw 2993 on every axis); no fault raised, lift-stop silently weakened | SYS-06 A/B rows | **Fixed** in `f4e94ad`, verified on car |
| Level lift: IR no-floor threshold 3950 above the ~3880 the sensors read when held up | S3-I05 row | Open |
| Single-core test image resets in S5-U06 | SYS-08 bench row | Open |
| Rejoin fails when the obstacle sits just after a bend | SYS-05 rows | Open. Attempted fix `fc9e59a` reverted: untested on car and it cleared the FR9 timer early |
| `OBS_FRONT_OFFSET_MM` 60 mm vs 40 mm measured | S5-I06 | Open |
| `APP_COUNTS_PER_REV` 640 vs 619–626 measured | S2-I09 | Open (team decision) |
| Loose power wire: 13 restarts in the last capture | MQTT captures | Hardware |

## Not run, and why

Floor motion tests S2-I01–I03 and S2-I08 need a MOVE-capable build. Scope or meter tests (S2-I04, S2-I05, S3-I10, S4-I07, S5-I05) and hang-injection tests (S5-I02, S5-I02b, S5-I02r, INT-08) need instruments or special builds. Laps (S3-I04), hump strips (S4-I14), the Wi-Fi-loss run and the numbered repeat trials in the [trial sheet](system/logs/2026-11-25_track_trials.csv) were not completed before the car was dismantled.

## Still to add

- Videos (none uploaded yet): obstacle stop runs 2–3 (S5-I01) were filmed. Add each unlisted link to [videos/README.md](../videos/README.md) and to its result row.
- Photos: the 40 mm offset with a ruler, and the barcode prints.
- Owners' sign-off on the integration rows.

## Evidence index

All raw captures are in [system/logs](system/logs/) and [subsystem1/logs](subsystem1/logs/). MQTT captures (`*_mqtt.csv`) hold the arrival time, topic, exact payload bytes and decoded fields of every message. `*_bench_*.txt` and `*_course_*.txt` are USB console captures.
