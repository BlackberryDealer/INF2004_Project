# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface `K1`–`K13` listed in `TEAM.md` section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results. Per the module coordinator's ruling, `.txt`/`.csv` logs prove pure data/protocol/timing checks; only physical car motion/clearance checks share the consolidated track/bench clips in `videos/`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Ultrasonic echo ISR -> `obstacle_task` (`SEM_ECHO` + packed `MSG_ECHO` 16-bit word, late/stuck/concurrent edges) | K1 (`R1 -> S5`), NFR6 | auto (host) | `testing/subsystem5/host/sim_obstacle_host.c` (`S5-U08`, `S5-U14`, `S5-U17`, `S5-U24`) | Valid echoes convert in 4 mm steps; late/stale/concurrent edges never corrupt distance |
| INT-02 | Barcode DO ISR -> `barcode_task` (`SEM_BAR` + 64-entry `MSG_EDGE` ring + DMB barriers + overflow discard) | K2 (`R1 -> S3`), FR2, NFR5 | auto (host) | `testing/subsystem3/host/test_line_barcode_sim_host.c` (`S3-U07`, `S3-U10`, `S3-U46`, `S3-U55`) | Full & half-scale frames decode on 29th edge; overflow discards cleanly to `IDLE` |
| INT-03 | Emergency & lift-stop flags (`FLG_SAFETY`: `SAFETY_ESTOP / LIFT / FAULT` from S4/S5 -> S2 `motion_task`) | K3 (`S4, S5 -> S2`), FR13, NFR6, NFR10a | auto (host) + car clip | Host: `sim_mission_host.c` (`S5-U78`), `test_imu_diag_host.c` (`S4-U20`), `test_motion_sim_host.c` (`S2-U03/U20`). Car: Clip 1 (obstacle stop) & Clip 2 (lift-stop) | `motion_task` brakes on next 10 ms cycle; `LIFT` in `RUN -> FINISHED`, `ESTOP` clears only via `mission_task` |
| INT-04 | Mission event queue (`MBF_MISSION_EVT`, 16-bit `MSG_EVT` words, non-blocking `TMO_POL`, overflow/clamp flags) | K4 (`all -> S5`), FR2–FR12 | auto (host) | `testing/subsystem5/host/sim_mission_host.c` (`S5-U04`, `S5-U05`, `S5-U75`, `S5-U77`) | 16 entries fit 128 B buffer; 17th sets `QUEUE_OVERFLOW` without blocking; value > 2047 clamps & flags |
| INT-05 | Tagged motion commands (`MBF_MOTION_CMD`, 16-bit `MSG_CMD` from S5 `mission_task` -> S2 `motion_task`, `MOVE_DONE` tag matching) | K5 (`S5 -> S2`), FR1, FR3, FR8 | auto (host) + car clip | Host: `sim_mission_host.c` (`S5-U07`, `S5-U27`) & `test_motion_sim_host.c` (`S2-U04/U09/U20`). Car: Clip 1 (turns + bypass) | Stale `MOVE_DONE` tags ignored; `FOLLOW`, `MOVE`, `TURN`, `ZERO` execute with bounded deadlines |
| INT-06 | Shared status board (`MTX_STATUS` `TA_INHERIT`, 11 packed `SB_*` words, 5 single-writer parts, atomic snapshots) | K6 (`each -> all`), NFR11 | auto (host) | `testing/subsystem5/host/test_status_board_host.c` (`S5-U74`) & `sim_mission_host.c` (`S5-U79`) | All 5 subsystem parts round-trip at min/typical/max in 44 B; sentinels preserved |
| INT-07 | Mission -> telemetry queue (`MBF_TELEM_EVT`: events, obstacle `PROFILE`, `SUMMARY`, on-demand `SNAPSHOT`) | K7 (`S5 -> S1`), FR10, FR12, NFR8 | auto (host) + `.txt` log | Host: `test_telem_bin_host.c` (`S1-U01/U02/U10/U42`), `sim_mission_host.c` (`S1-U41`), `test_dashboard.py` (`S1-U44/U50`) | Every binary payload matches `telem_decode.py` & `dashboard.js` byte-for-byte |
| INT-08 | Task alive check-ins & hardware watchdog (`FLG_ALIVE`, 500 ms boot -> 30 ms normal window, 65 ms obstacle age) | K8 (`T1–T4 -> S5`), NFR10b | auto (host + target) | Host: `test_hmi_health_host.c` (`S5-U85`). Target `TEST=1`: `test_mission.c` (`S5-U29`, `S5-U06`) | Watchdog switches to 30 ms only after all 4 check-ins; withholds feed if any task stalls |
| INT-09 | PIO quadrature encoders -> `motion_task` -> odometry & $2^{17}$ path wrap (`0..131071` counts across S2, S3, S4, S5) | K9 (`PIO -> S2 -> K6`), FR1, A4, A23 | auto (host) | `test_odometry_host.c` (`S2-U01/U02/U31`), `test_line_barcode_sim_host.c` (`S3-U50`), `sim_mission_host.c` (`S5-U79/U80`), `sim_obstacle_host.c` (`S5-U83`) | Speed, heading, barcode rewind, line loss, ramp clear, and odometer stay continuous across $2^{17}$ wrap |
| INT-10 | Mission mode flags (`FLG_MODE`: `CAL_IMU`, `CAL_IR`, `RUN_START`, `MOTOR_EN`, `BARCODE_REREAD`, `RESCAN`, `RESUME`) | K10 (`S5 -> all`), FR17 | auto (host) | `sim_mission_host.c` (`S5-U07`, `S5-U57`), `sim_obstacle_host.c` (`S5-U31`, `S5-U42`), `test_line_barcode_sim_host.c` (`S3-U44`) | Motors stay braked whenever `MOTOR_EN` is clear; calibration, reread, and rescan handshakes complete cleanly |
| INT-11 | Telemetry -> radio bridge (`MBF_TELEM` + `c28/#` MQTT routing, QoS, rate-limited remote commands, Wi-Fi loss decoupling) | K11 (`S1 -> radio`), FR10–FR12, NFR8, NFR9 | auto (host) + car clip | Host: `test_mqtt_bridge_host.c` (`S1-U11/U12/U30..U35/U40`), `test_mqtt_utk_host.c` (`S1-U13/U15/U36/U37/U39`). Car: Clip 1 (`S1-I12`) & Clip 3 (`S1-I05` Wi-Fi off) | Commands rate-limited at 100 ms; Wi-Fi loss sets informational `WIFI_LOSS` only while car keeps driving |
| INT-12 | Shared fault bitmask (`FLG_FAULT`: stop-faults vs informational `WIFI_LOSS`/`RANGE_CLAMP`) | K12 (`all -> S5, S1`), FR13, A25 | auto (host) + car clip | Host: `sim_mission_host.c` (`S5-U07`), `test_imu_diag_host.c` (`S4-U20`), `test_mqtt_bridge_host.c` (`S1-U33`). Car: Clip 3 (stall / SDA unplug -> `FAULT` -> `RESET`) | Stop-faults (`0x3AF`) brake into `FAULT` and require `RESET`; informational bits (`WIFI_LOSS`, `RANGE_CLAMP`) never abort `RUN` |
| INT-13 | Console mirror over MQTT (`K13`, `DEBUG_MQTT=1` images only: `app_log` -> `c28/l`, sizing & `FR9` dashboard parsing) | K13 (`all -> radio`), B1.4 | auto (host) | `test_lwip_sizing_host.c` (`S1-U38` normal & `DEBUG_MQTT=1`) & `tools/dashboard/test_ui.js` (`S1-U50`) | Fits 64 lwIP slots & 2048 B ring; `c28/l` console lines (`FR9`, invalid barcode counts) update dashboard |

*Method:* **auto** = `TEST_CHECK` / `HOST_CHECK` in code, result printed on the console;
**manual / car clip** = physical verification on the assembled PicoCar where `.txt` logs alone cannot show physical clearance/LED/wheel action.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
