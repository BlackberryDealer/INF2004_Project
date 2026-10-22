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
| S1-U01 (baseline) | State JSON from a fixed `telem_bin_state_t` equals the expected string exactly | FR10, NFR8, B1.2 | auto (host) | `test_telem_json_host.c` (in `w10-baseline` only) | `[TEST] S1-U01 PASS` |
| S1-U02 (baseline) | Event, profile, summary, heartbeat JSON equal expected strings; too-small buffer returns 0 | FR10, B1.2 | auto (host) | `test_telem_json_host.c` (in `w10-baseline` only) | `[TEST] S1-U02 PASS` |
| S1-U01 (packed) | State payload from a fixed struct = the 18 hand-computed bytes `01 44 28 D9 D0 07 C4 E0 0F 83 A1 0F 5A 64 98 66 08 00`; RNG clamp reported, ENUM clamp silent | B1.4 | auto (host) | `test_telem_bin_host.c` | `[TEST] S1-U01 PASS` |
| S1-U02 (packed) | Event 3 B, profile 6 B, summary 7 B, heartbeat 6 B, availability 1 B (0x29 / 0x31), exact bytes | B1.4 | auto (host) | `test_telem_bin_host.c`, `test_mqtt_bridge_host.c` (S1-U32) | `[TEST] S1-U02 PASS` |
| S1-U03 | Fill K11 (TMO_POL) before the link is up: at most 8 fit, the next send fails at once; telem_task's next payload is counted in `telem_drops()`; `mqtt_bridge_take` drains them | FR11, NFR8 | auto (target) | `make … TEST=1`, flash, read the console | `[TEST] S1-U03 PASS` (×4) |
| S1-U10 | Golden vectors: every MQ_ payload at min and max (state also mid) written by the car-side C code, decoded by `tools/telem_decode.py --golden`, equals the input values | B1.4 step 10 | auto (host) | `make -C testing/subsystem1/host` (`telem_bin` target) | `[TEST] S1-U10 PASS` ×11, `golden: 11 PASS 0 FAIL` |
| S1-U11 | Command with version 2, wrong length, unknown type or MAP char > 42 is rejected and counted; drop counter saturates at 255 and never touches the publish-drop field | B1.4 | auto (host) | `test_mqtt_bridge_host.c` (real `mqtt_bridge.c`, mock kernel) | `[TEST] S1-U11 PASS` |
| S1-U12 | Two STOPs 50 ms apart: one STOP posted, one drop; 100 ms later accepted; per type; 16-bit time wrap handled | FR11, B1.4 | auto (host) + smoke (target) | host: `test_mqtt_bridge_host.c`; target: `test_telem.c` checks the calls never block (counts have no getter in the frozen contract) | `[TEST] S1-U12 PASS` |
| S1-U13 | Publish refused twice (`ERR_MEM` hook): payload counted in `PUB_DROPS`, next payload still sent; ≤ 2 publishes per drain call; none while disconnected; late failure in `publish_done_cb` counted | NFR9, B1.4 | auto (host) | `test_mqtt_utk_host.c` (real `mqtt_utk.c`, mock lwIP) | `[TEST] S1-U13 PASS` |
| S1-U14 | `gen_layout.py` rejects `MQ_HB.DROPS` (0..255) widened from 8 to 9 bits | B1.4 | auto (host) | `make -C testing/subsystem1/host` (`gen_layout_reject_mq`) | `[TEST] S1-U14 PASS` |
| S1-U15 | Payload A refused once then sent; payload B refused once is retried (its tries start at 0), not dropped | NFR9, B1.4 | auto (host) | `test_mqtt_utk_host.c` | `[TEST] S1-U15 PASS` |
| S1-U30 | **Added.** `mqtt_bridge_take`: empty K11 returns 0; returns the type (from MQ_HDR) and every byte; a payload larger than the caller's buffer is dropped and counted | D.9 | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U30 PASS` |
| S1-U31 | **Added.** `mqtt_bridge_route`: state QoS 0; event/profile/summary QoS 1; heartbeat QoS 0; availability QoS 1 retained; unknown → event QoS 0 | D.9, B1.2 | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U31 PASS` |
| S1-U32 | **Added.** Availability payload online / offline; too-small buffer returns 0 | D.9 | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U32 PASS` |
| S1-U33 | **Added.** Link down sets `APP_FAULT_WIFI_LOSS`, link up clears it; no mission event either way (NFR9c) | NFR9c | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U33 PASS` |
| S1-U34 | **Added.** Publish drops saturate at 255; command topic is `c28/c` | D.9 | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U34 PASS` |
| S1-U35 | **Added.** MAP command posts `APP_EVT_MAP` with char and command | FR11 (optional MAP) | auto (host) | `test_mqtt_bridge_host.c` | `[TEST] S1-U35 PASS` |
| S1-U36 | **Added.** Back-off: retries at 1, 2, 4, 8, 8 s; the 2 ms poll loop cannot cut the wait short; an immediate connect failure takes the same path; refused link reported down | NFR9b | auto (host) | `test_mqtt_utk_host.c` | `[TEST] S1-U36 PASS` |
| S1-U37 | **Added.** Connect uses the 4 s keepalive and a retained QoS 1 last will with the offline payload; on ACCEPTED: link up, back-off reset, online published, command topic subscribed at QoS 1, drain started; command bytes forwarded unfiltered, other topics ignored | NFR9a, B1.2 | auto (host) | `test_mqtt_utk_host.c` | `[TEST] S1-U37 PASS` |
| S1-I01 | State 5 Hz ± 10 % over 60 s | NFR8a | broker log | `tools/telem_decode.py` with timestamps, 60 s | 270–330 state messages |
| S1-I02 | Event ≤ 500 ms after it happens; heartbeat 1 Hz ± 10 % | NFR8b, c | broker log | console event time vs broker time; heartbeat count over 60 s | ≤ 500 ms; 54–66 heartbeats |
| S1-I03 | Broker killed: loss detected ≤ 7.5 s; retries at 1, 2, 4, 8, 8 s; `FAULT_WIFI_LOSS` set then cleared | NFR9a, b | manual (bench) | stop Mosquitto, watch console and broker | ≤ 7.5 s; retry times as listed |
| S1-I04 | Remote STOP during RUN ends in FINISHED like the button | FR11 | manual (with B5) | `mosquitto_pub -t c28/c` with the STOP byte 0x89 | FINISHED, summary published |
| S1-I05 | Wi-Fi off mid-run: run continues unchanged | NFR9c | manual (with B5) | turn the AP off during RUN | run completes; WIFI_LOSS bit only |
| S1-I06 | A16 radio-task time: worst `line_task` release latency ≤ 1000 µs in steady 5 Hz, broker killed, broker restarted, AP off/on, 30-min soak | NFR17, A16 | probe (`TEST=1 PROBE=1`) | Method A: telem_task prints `[A16] up … lat_max … drops …` every second | ≤ 1000 µs in all scenarios |
| S1-I07 | Silent blackhole (broker laptop leaves Wi-Fi, AP stays up), 5 trials: `FAULT_WIFI_LOSS` set ≤ 7.5 s after the last packet | NFR9a | manual (bench) | laptop Wi-Fi off, timestamp last broker packet | ≤ 7.5 s, 5 / 5 |
| S1-I10 | Sizes logged on the broker for both tags | B1.5 | broker log | `tools/mqtt_size_log.py --seconds 600` per tag | logged packet sizes = `docs/message_sizes.md` computed column |
| A10 | Broker bring-up: car connects to the laptop broker, messages visible with `mosquitto_sub -v -t '#'` | A10 | manual (bench) | copy `config/wifi_secrets.example.h` → `config/wifi_secrets.h` (never commit) | car online, messages arrive |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Design notes

**A16 method.** The spec's harness registers three dummy tasks in
`sys_tick` slots 0–2. On the merged main those slots already belong to
the real line, motion and IMU tasks, so a dummy task would take their
slot. S1-I06 therefore uses Method A only: in a `TEST=1 PROBE=1` build
`telem_task` prints `sys_tick_latency_max_us()` once per second, which
already covers all three real 10 ms slots.

**On-target S1-U12.** The command-drop counter has no getter in the
frozen contract (D.9), so the exact counts are checked on the laptop
against the real `mqtt_bridge.c`; the target test only proves the calls
never block.

**Extra test IDs S1-U30..U37** cover the bridge and radio-task behaviour
that the spec lists as requirements but gives no unit-test ID.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S1-U20..U24 | `<commit>` | Hanz | PASS | `logs/2026-10-21_host_app_bits.txt` | 15 checks + S1-U24 |
| 2026-10-22 | S1-U01, U02 (baseline), U12, U13, U15, U20..U24, U30..U37 | `6932292` | Hanz | PASS | `logs/2026-10-22_host_s1_baseline.txt` | 71 PASS lines, 0 FAIL (`w10-baseline` candidate) |
| 2026-10-22 | S1-U01, U02 (packed), U10..U15, U20..U24, U30..U37 | `5bb213b` | Hanz | PASS | `logs/2026-10-22_host_s1_packed.txt` | 85 PASS lines, 0 FAIL (incl. golden 11 / 11, S1-U14, S1-U24) |
| 2026-10-22 | build (normal, TEST=1, TEST=1 PROBE=1), BARR, layout | `5bb213b` | Hanz | PASS | `logs/2026-10-22_build_packed.txt` | 0 warnings; barr_check no findings; layout.csv 155 rows OK |
| — | S1-U03, S1-U12 (target smoke) | — | — | pending | — | needs the Pico: flash the TEST=1 image |
| — | A10, S1-I01..I07, S1-I10 | — | — | pending | — | needs the car, the lab AP and the laptop broker |
