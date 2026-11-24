# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Complete run started and stopped from the dashboard: the car follows the line over the whole course (also evidence for S3-I04, S1-I12) | FR1, FR11, FR12, NFR4, NFR8, NFR15 | manual (track, video Clip 1) | 1. Flash the normal build; car on the start line; power on and wait for READY (steady green). 2. Dashboard: **START**. 3. Let the car run the whole course; SYS-02 to SYS-05 happen during this run. 4. At the end, dashboard: **STOP**, then **RESET**. | START accepted → RUN (blue); no line loss over 200 ms at cruise; the dashboard state updates throughout (5 Hz); STOP → FINISHED (green blink) and the run summary appears; RESET → BOOT → calibration → READY |
| SYS-02 | Full-scale barcode decoded and its command carried out at the next junction (also evidence for S3-I02) | FR2, FR3, FR19, NFR5 | manual (track, video Clip 1) | During SYS-01 the car crosses the full-scale barcode, then reaches cross #1. Note the barcode's printed code before the run. | The decoded code shown on the console or dashboard matches the printed one; the car takes the commanded turn at cross #1 |
| SYS-03 | Half-scale barcode decoded, with the back-up and re-read when the first pass fails (also evidence for S3-I02) | FR2, FR3, FR19, NFR5 | manual (track, video Clip 1) | During SYS-01 the car crosses the half-scale barcode, then reaches cross #2. | The code is decoded, after a back-up and re-read if one is triggered; the car takes the commanded turn at cross #2 |
| SYS-04 | Hump height measured; hump and ramp give no false lift-stop (also evidence for S4-I01, S4-I02, S4-I13) | FR4, FR5, FR13 (lift-stop), NFR7 | manual (track, video Clip 1) | Ruler beside the hump, in the shot; write down the ruler height before the run. During SYS-01 the car crosses the hump and the ramp. | Reported peak height within ±20 % or ±5 mm of the ruler; no hump reported on flat track; RUN continues over the hump and the ramp (no `LIFT STOP` line) |
| SYS-05 | Obstacle: stop, width, bypass and rejoin (also evidence for S5-I01, S5-I03, S5-U37) | FR6, FR7, FR8, FR9, NFR6, NFR16 | manual (track, video Clip 1) | A box on a straight, approached at cruise; measure its width with a tape before the run. During SYS-01 the car meets the box. | Stops at least 100 mm before the box; the scan's width appears on the dashboard (record it next to the tape width); bypass on the planned side; line reacquired within 5 ± 0.5 s after the pass leg; the run then carries on (no path around → FAULT is a fail here) |
| SYS-06 | Lifting the car ends the run (also evidence for S4-I12) | FR13 (Change Requirement 2), NFR10 | manual (track, video Clip 2) | 1. READY, dashboard START. 2. During RUN, lift the car straight up. 3. RESET, START again; during RUN lift it tilted. | Each time the wheels stop at once (5-cycle confirm, 50 ms, then motors low within 15 ms); console `LIFT STOP`; state FINISHED (green blink), end reason STOP; RESET → READY |
| SYS-07 | Resilience: losing Wi-Fi does not stop the car; a stalled motor and a lost IMU bus stop it safely (also evidence for S1-I05, S2-I06, S4-I05) | FR17, NFR9, NFR10, B4.2.2 | manual (track, video Clip 3) | 1. During RUN, switch the phone hotspot off. 2. Hotspot on; new run; hold one wheel still for over 300 ms. 3. RESET; new run; pull the IMU SDA wire. 4. Reconnect SDA, RESET. | 1: run continues normally, only the `WIFI_LOSS` bit is set (loss detected within 7.5 s). 2: motors brake, `MOTOR_STALL` fault, FAULT (red blink). 3: `IMU` fault after 3 failed reads, motors low, FAULT (red blink). 4: RESET → BOOT → calibration → READY |

SYS-01 to SYS-05 are one continuous run of the course, filmed as one clip
(Clip 1); SYS-06 and SYS-07 are Clips 2 and 3. Each row names the subsystem
tests it also gives evidence for. A single run does not replace the repeat
counts those tests need (for example S3-I02: 20 passes per print scale):
log the repeats as text.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
