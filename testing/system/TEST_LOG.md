# PicoCar system tests

These IDs retain the definitions used by the November 24 Pico capture. Run the [demo-day procedure](../DEMO.md). Use the [trial sheet](track_trials.csv) for repeated measurements.

| ID | Requirement | Method and expected result |
|---|---|---|
| SYS-01 | FR1, FR16, FR17 · boot and calibration | Start capture before boot. BOOT → CAL_IMU → CAL_IR → READY, both IR spans ≥400 ADC codes, zero stop faults. Observe white boot, yellow calibration and solid green READY. |
| SYS-02 | FR10–12, NFR8 · dashboard | In READY issue START. During RUN request SUMMARY, then STOP and RESET. Verify RUN → FINISHED → READY, frozen final summary after STOP and live telemetry at 5 Hz ±10%. Use the broker timestamp trace to check rate. |
| SYS-03 | FR1–3, FR19 · line and barcode routing | Load all four assigned mappings over MQTT in READY. Log each acknowledgement. Drive over a full- and half-scale print on the left. Record the printed letter, decoded letter, next cross and actual movement. Expect the assigned action at the next cross and continued following. Exercise all four actions and both reading directions across trials. For S3-I02 use 20 passes per scale and ≥95% correct. For S3-I04 use three runs at 250 mm/s with no loss over 200 ms. |
| SYS-04 | FR4, FR5, FR13 · hump and ramp | Compare reported height with the ruler. For the 4 and 5 mm right-wheel strips, use five runs each, one event per strip within 1 mm, and three flat controls with zero events (S4-I14). Check ramp rejection and no false lift during driving. S4-I02 has separate larger-hump trials. |
| SYS-05 | FR6–9 · obstacle | At cruise observe stop, scan, chosen side, clearance and line rejoin. At 300 mm/s expect ≥100 mm stop clearance and braking within 150 ms of the first reading. For S5-I09 scan 100/150/200 mm boxes three times each. Record all widths, signed errors and spread; mean error for each box must be within ±10 mm. Record contact and rejoin time from end of pass leg (upper limit 5.5 s). |
| SYS-06 | FR13 · lift-stop | Run, lift, observe LIFT STOP, wheels stopped and FINISHED with STOP reason. RESET returns to READY. For S4-I12 perform ten lifts covering level, tilted and overturned on a soft surface. Use synchronized lift/PWM measurement to establish the ≤100 ms bound. |
| SYS-07 | FR15, FR17, NFR9 · faults and link loss | With USB capture or a track observer, switch off the hotspot during RUN. Car continues with WIFI_LOSS. On separate runs check blocked-wheel STALL and IMU bus failure. Capture FAULT, motors braked and RESET recovery after clearing the cause. |
| SYS-08 | NFR1, NFR5c, NFR11, NFR13, NFR17 · diagnostics | Run the host suites and single-core bench image. Measure line period 9–11 ms and WCET ≤300 µs, decode ≤50 ms, calibrated IR noise <1%, IMU read budget for the workload and minimum free stack ≥25%. Use S1-I06 for release latency, S5-I04 for CPU/RAM, and the project layout and BARR-C checks. Record each part independently. |

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | SYS-08 | `c96481d` | Lucas | PASS (host checks only) | [Host output](../subsystem1/logs/2026-11-24_host_all_suites.txt) | Host simulation, layout and BARR-C at that commit. |
| 2026-11-24 | SYS-01 | `40de1f5` | Hanz | PASS (boot log only) | [Pico console](logs/2026-11-24_target_test_session1.txt) | Reached READY, spans L 3222 and R 3136. SMP=1. LED observation and single-core repeat open. |
| 2026-11-24 | SYS-08 | `40de1f5` | Hanz | FAIL (bench criteria) | [Pico console](logs/2026-11-24_target_test_session1.txt) | S1-U03 queue test, S3-I01 line WCET, S4-I09 IMU read time and S5-U06 latency failed. Record the single-core rerun separately. |
| 2026-11-25 | SYS-08 | `72482f3` | Codex | PASS (host and static portions only) | [Host](../subsystem1/logs/2026-11-25_090514_host_72482f3_f1OjER.txt), [static checks](logs/2026-11-25_72482f3_static_checks.txt), [UF2 build records](logs/2026-11-25_72482f3_images.txt) | 944 printed PASS, zero FAIL or SKIP. Layout 396 rows. BARR-C 101 files, zero findings. On-car timing, CPU/RAM and physical results pending. |
