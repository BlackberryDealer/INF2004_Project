# Message sizes: baseline vs packed

Owner: Buddy 1 (communications). Spec B1.5. Evidence for the Week 13
minimum-bit report (topic shortening and payload packing, NFR8).

Two tags are compared:

- **`w10-baseline`** = commit `6932292` on `b1-comms` (JSON payloads,
  topics `picocar/csc28-car/...`, text availability and commands).
- **`w13-packed`** = the packed build (binary payloads from
  `telem_bin.c` and the `MQ_*` rows of `tools/layout.csv`, topics `c28/*`).

Step 4.1 (short topics, still JSON) is commit `32ec397`; it gives the
separate "topic shortening" rows below, so the two savings can be told
apart.

## Debug payloads added 2026-11-06 (not part of the comparison)

An MQTT image can also send two bring-up diagnostics: SENSORS on `c28/d`
(12 B, `MQ_SENS`, 5 Hz, packed like every other payload) and LOG on
`c28/l` (header byte + one console line of up to 120 ASCII characters).
Since 2026-11-16 they are off unless the image is built with
`make NET=lwip MQTT=1 DEBUG_MQTT=1` (image name ends in `_dbg`;
`TELEM_DEBUG_MQTT` in `app_program/subsystem1/cfg_telem.h`). The normal
image sends no text over MQTT (Change Requirement 1). They have no
baseline counterpart, so build the Week 13 evidence without `DEBUG_MQTT`;
the tables below cover only the original payloads. With the diagnostics
on, K11 has 12 entries instead of 8 and K13 (console lines, 48 x 20 B =
960 B) exists; the lwIP output ring is 2048 B and 64 request slots
(S1-U38, checked for both builds).

## How the numbers were obtained

| Column | Source | Status |
|---|---|---|
| Payload B (baseline) | the real `telem_json.c` of `6932292`, run on the laptop; typical = the S1-U01 struct, largest = every field at its longest text | computed 2026-10-22 |
| Payload B (packed) | `telem_bin.c`, exact bytes checked by S1-U01/U02 (packed) and the golden vectors (S1-U10) | computed 2026-10-22 |
| K11 / K7 entry B | 4-byte kernel header + payload rounded up to 4 (spec B1.5) | computed |
| Kernel buffer B | entries × (4 + `maxmsz`), the static `_mbf_buf` arrays (`arm-none-eabi-nm` of both ELFs) | measured from the ELFs 2026-10-22 |
| MQTT packet B | `rem = 2 + topic + (2 if QoS > 0) + payload`, `packet = 1 + (1 if rem < 128 else 2) + rem` (the formula in `tools/mqtt_size_log.py`) | computed; packed topic/payload bytes verified on broker across 24 211 live car messages on `f4e94ad` (2026-11-25); `w10-baseline` broker log (`S1-I10`) not run before car dismantling |
| Wireshark frame B | one frame per topic on the broker laptop | not run (car dismantled 2026-11-25) |
| telem_task WCET, radio time per publish | GP14 probe (`PROBE=1`), scope | not run (car dismantled 2026-11-25) |

## Summary

| Item | Baseline | Topic shortening (4.1) | Packed (4.2–4.4) | Spec expectation |
|---|---:|---:|---:|---|
| State payload, typical / largest | 145 / 159 B | 145 / 159 B | 18 B | — |
| State MQTT packet, typical | 173 B | 155 B | 27 B | ≈ 137 → 27 B |
| State traffic at 5 Hz | 865 B/s | 775 B/s | 135 B/s | — |
| Heartbeat MQTT packet, typical | 54 B | 39 B | 15 B | — |
| K11 buffer (8 entries) | 8 × (4 + 160) = 1312 B | 1312 B | 8 × (4 + 20) = 192 B | 1312 → 192 B |
| K7 buffer (8 entries) | 8 × (4 + 32) = 288 B | 288 B | 8 × (4 + 8) = 96 B | 288 → 96 B |
| Radio task pending copy (`pend_buf`) | 160 B | 160 B | 20 B | — |
| Radio task `mqtt_bridge_take` local copy | 161 B (stack) | 161 B | 20 B | — |

The baseline state packet is larger than the spec's estimate (173 vs
≈ 137 B) because the JSON keys and the two arrays are longer than the
estimate assumed; the packed result matches the spec exactly (27 B).
In the final submitted course ELF
(`mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.elf`, SHA-256 `072aa8b7…`),
`arm-none-eabi-nm` confirms `g_telem_k11_mbf_buf = 192 B`,
`g_telem_k7_mbf_buf = 96 B`, and `pend_buf = 20 B` (unchanged from the
table above).

## Per message

### State (5 Hz)

Topic `picocar/csc28-car/state` → `c28/s`, QoS 0. Queue: K11.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline (typical) | `w10-baseline` (JSON, long topic) | 23 | 145 | 152 | 173 |
| baseline (largest) | `w10-baseline` | 23 | 159 | 164 | 187 |
| topic shortening (typical) | 4.1 (JSON, short topic) | 5 | 145 | 152 | 155 |
| topic shortening (largest) | 4.1 | 5 | 159 | 164 | 169 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 18 | 24 | 27 |

### Event

Topic `picocar/csc28-car/event` → `c28/e`, QoS 1. Queue: K7 → K11.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline (typical) | `w10-baseline` (JSON, long topic) | 23 | 14 | 20 | 43 |
| baseline (largest) | `w10-baseline` | 23 | 18 | 24 | 47 |
| topic shortening (typical) | 4.1 (JSON, short topic) | 5 | 14 | 20 | 25 |
| topic shortening (largest) | 4.1 | 5 | 18 | 24 | 29 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 3 | 8 | 14 |

K7 entry (message + 4-byte header, rounded to 4): baseline 36 B → packed 8 B.

### Obstacle profile

Topic `picocar/csc28-car/event` → `c28/e`, QoS 1. Queue: K7 → K11.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline (typical) | `w10-baseline` (JSON, long topic) | 23 | 68 | 72 | 97 |
| baseline (largest) | `w10-baseline` | 23 | 73 | 80 | 102 |
| topic shortening (typical) | 4.1 (JSON, short topic) | 5 | 68 | 72 | 79 |
| topic shortening (largest) | 4.1 | 5 | 73 | 80 | 84 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 6 | 12 | 17 |

K7 entry (message + 4-byte header, rounded to 4): baseline 36 B → packed 12 B.

### Run summary

Topic `picocar/csc28-car/event` → `c28/e`, QoS 1. Queue: K7 → K11.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline (typical) | `w10-baseline` (JSON, long topic) | 23 | 59 | 64 | 88 |
| baseline (largest) | `w10-baseline` | 23 | 66 | 72 | 95 |
| topic shortening (typical) | 4.1 (JSON, short topic) | 5 | 59 | 64 | 70 |
| topic shortening (largest) | 4.1 | 5 | 66 | 72 | 77 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 7 | 12 | 18 |

K7 entry (message + 4-byte header, rounded to 4): baseline 36 B → packed 12 B.

### Snapshot (reply to CMD_SNAP, "summary now")

Topic `c28/e`, QoS 1. Queue: K7 → K11. Added 2026-11-03 after the packed
format, so there is no baseline or topic-shortening row.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` | 23 | not in baseline | — | — |
| payload packing | binary, short topic (`MQ_SNAP`) | 5 | 7 | 12 | 18 |

K7 entry: 2-byte event word + 6-byte `K7_SNAP` = 8 B message → 12 B.

### Heartbeat (1 Hz)

Topic `picocar/csc28-car/hb` → `c28/h`, QoS 0. Queue: K11.

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline (typical) | `w10-baseline` (JSON, long topic) | 20 | 30 | 36 | 54 |
| baseline (largest) | `w10-baseline` | 20 | 35 | 40 | 59 |
| topic shortening (typical) | 4.1 (JSON, short topic) | 5 | 30 | 36 | 39 |
| topic shortening (largest) | 4.1 | 5 | 35 | 40 | 44 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 6 | 12 | 15 |

### Availability online

Topic `picocar/csc28-car/status` → `c28/a`, QoS 1. Queue: none (radio task).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` (JSON, long topic) | 24 | 6 | — | 36 |
| topic shortening | 4.1 (JSON, short topic) | 5 | 6 | — | 17 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 1 | — | 12 |

### Availability offline (last will)

Topic `picocar/csc28-car/status` → `c28/a`, QoS 1. Queue: none (broker).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` (JSON, long topic) | 24 | 7 | — | 37 |
| topic shortening | 4.1 (JSON, short topic) | 5 | 7 | — | 18 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 1 | — | 12 |

### Command START / RESET

Topic `picocar/csc28-car/cmd` → `c28/c`, QoS 1. Queue: none (laptop → car).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` (JSON, long topic) | 21 | 15 | — | 42 |
| topic shortening | 4.1 (JSON, short topic) | 5 | 15 | — | 26 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 1 | — | 12 |

### Command STOP

Topic `picocar/csc28-car/cmd` → `c28/c`, QoS 1. Queue: none (laptop → car).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` (JSON, long topic) | 21 | 14 | — | 41 |
| topic shortening | 4.1 (JSON, short topic) | 5 | 14 | — | 25 |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 1 | — | 12 |

### Command MAP

Topic `picocar/csc28-car/cmd` → `c28/c`, QoS 1. Queue: none (laptop → car).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` | 21 | not in baseline | — | — |
| payload packing | 4.2–4.4 `w13-packed` (binary, short topic) | 5 | 2 | — | 13 |

### Command SNAP ("summary now")

Topic `c28/c`, QoS 1. Queue: none (laptop → car). Byte `0xA1`
(`tools/telem_cmd.py summary`).

| Row | Tag / step | Topic B | Payload B | K11 entry B | MQTT packet B |
|---|---|---:|---:|---:|---:|
| baseline | `w10-baseline` | 21 | not in baseline | — | — |
| payload packing | binary, short topic | 5 | 1 | — | 12 |

## Hardware measurements and 25 Nov broker observations (S1-I10)

To log raw MQTT PUBLISH packet sizes directly on a live car for both tags:

```
python3 tools/mqtt_size_log.py --out testing/subsystem1/logs/<date>_sizes_<tag>.csv --seconds 60
```

### Live packed broker captures on the submitted course image (2026-11-25)

The dedicated `tools/mqtt_size_log.py` baseline-vs-packed comparison
(`S1-I10`), Wireshark captures, and GP14 scope timing were not run before
the car was dismantled on 2026-11-25. However, the four 25 Nov course-image
broker captures on `f4e94ad` (SHA-256 `072aa8b7…`, byte-identical to final
`6899aa0`: `testing/system/logs/2026-11-25_120858_764062_mqtt.csv`,
`2026-11-25_121144_105944_mqtt.csv`, `2026-11-25_121342_949638_mqtt.csv`,
and `2026-11-25_121702_512268_mqtt.csv` — 24 211 messages total) recorded
every topic and hex payload byte delivered at the broker, with zero length
deviations and zero debug/text messages (`c28/d` or `c28/l`, confirming
Change Requirement 1):

| Topic | Message type | Observed count (`f4e94ad`) | Observed payload B | Configured QoS | Resulting MQTT packet B |
|---|---|---:|---:|---:|---:|
| `c28/s` | `STATE` | 18 465 | 18 | 0 | 27 |
| `c28/e` | `EVENT` | 1 912 | 3 | 1 | 14 |
| `c28/e` | `PROFILE` | 12 | 6 | 1 | 17 |
| `c28/e` | `SUMMARY` | 27 | 7 | 1 | 18 |
| `c28/e` | `SNAPSHOT` | 0* | 7* | 1 | 18 |
| `c28/h` | `HEARTBEAT` | 3 692 | 6 | 0 | 15 |
| `c28/a` | `ONLINE` / `OFFLINE` | 45 (24 / 21) | 1 | 1 | 12 |
| `c28/c` | `CMD_START` / `STOP` / `RESET` | 46 (17 / 14 / 15) | 1 | 1 | 12 |
| `c28/c` | `CMD_MAP` | 12 | 2 | 1 | 13 |
| `c28/c` | `CMD_SNAP` | 0* | 1* | 1 | 12 |

\*`CMD_SNAP` (1 B → 12 B packet) and its `SNAPSHOT` reply (7 B → 18 B
packet) were observed twice each on the earlier 25 Nov single-core course
image (`72482f3`, identical telemetry encoding) in
`testing/system/logs/2026-11-25_105904_758534_mqtt.csv` and
`2026-11-25_110756_189423_mqtt.csv`.

Every observed topic and payload length matches the computed tables above.
QoS is set in `app_program/subsystem1/mqtt_bridge.c` (`mqtt_bridge_route()`)
and `tools/telem_cmd.py`; the broker CSV logger records topic, retained
flag, and exact payload hex rather than the wire QoS field.

### Unrun hardware measurements (car dismantled 2026-11-25)

| Topic | Baseline packet B (log) | Packed packet B (25 Nov CSV + QoS) | Wireshark frame B baseline | Wireshark frame B packed |
|---|---|---|---|---|
| state (`c28/s`) | not run (car dismantled) | 27 (18 B payload × 18 465) | not run | not run |
| event (`c28/e`) | not run (car dismantled) | 14 / 17 / 18 (3 / 6 / 7 B × 1 951) | not run | not run |
| hb (`c28/h`) | not run (car dismantled) | 15 (6 B payload × 3 692) | not run | not run |
| availability (`c28/a`) | not run (car dismantled) | 12 (1 B payload × 45) | not run | not run |
| cmd (`c28/c`) | not run (car dismantled) | 12 / 13 (1 / 2 B × 58) | not run | not run |

| Timing (GP14, `PROBE=1`) | Baseline | Packed |
|---|---|---|
| telem_task WCET per state payload | not run (car dismantled 2026-11-25) | not run (car dismantled 2026-11-25) |
| Radio task time per publish | not run (car dismantled 2026-11-25) | not run (car dismantled 2026-11-25) |
