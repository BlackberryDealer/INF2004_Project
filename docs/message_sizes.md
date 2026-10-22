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

## How the numbers were obtained

| Column | Source | Status |
|---|---|---|
| Payload B (baseline) | the real `telem_json.c` of `6932292`, run on the laptop; typical = the S1-U01 struct, largest = every field at its longest text | computed 2026-10-22 |
| Payload B (packed) | `telem_bin.c`, exact bytes checked by S1-U01/U02 (packed) and the golden vectors (S1-U10) | computed 2026-10-22 |
| K11 / K7 entry B | 4-byte kernel header + payload rounded up to 4 (spec B1.5) | computed |
| Kernel buffer B | entries × (4 + `maxmsz`), the static `_mbf_buf` arrays (`arm-none-eabi-nm` of both ELFs) | measured from the ELFs 2026-10-22 |
| MQTT packet B | `rem = 2 + topic + (2 if QoS > 0) + payload`, `packet = 1 + (1 if rem < 128 else 2) + rem` (the formula in `tools/mqtt_size_log.py`) | computed; **broker log pending (S1-I10)** |
| Wireshark frame B | one frame per topic on the broker laptop | **pending (hardware)** |
| telem_task WCET, radio time per publish | GP14 probe (`PROBE=1`), scope | **pending (hardware)** |

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

## Pending hardware measurements (S1-I10)

Run on the broker laptop once per tag (flash the tag, let the car run a
course for 60 s):

```
python3 tools/mqtt_size_log.py --out testing/subsystem1/logs/<date>_sizes_<tag>.csv --seconds 60
```

then fill in:

| Topic | Baseline packet B (log) | Packed packet B (log) | Wireshark frame B baseline | Wireshark frame B packed |
|---|---:|---:|---:|---:|
| state | pending | pending | pending | pending |
| event | pending | pending | pending | pending |
| hb | pending | pending | pending | pending |
| availability | pending | pending | pending | pending |
| cmd (laptop → car) | pending | pending | pending | pending |

| Timing (GP14, `PROBE=1`) | Baseline | Packed |
|---|---:|---:|
| telem_task WCET per state payload | pending | pending |
| Radio task time per publish | pending | pending |

The logged packet sizes must equal the computed column above; a
difference means the broker saw extra properties or a different QoS.
