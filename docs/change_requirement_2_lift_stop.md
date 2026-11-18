# Change Requirement 2: lift-stop replaces IMU collision detection

**Source:** lecturer's messages of 11 Nov 2026 (12:59 and 17:49).
**Owner of the change:** Buddy 4 (IMU) with Buddy 2 (K3 safety flag) and
Buddy 5 (mission statechart).

## 1. What was asked

> Collision detection using the IMU will be removed. Instead, the IMU should
> be used to detect when the robot has been lifted or overturned. When such
> a condition is detected, the motors should immediately stop. During the
> demonstration, lifting the robot will be used to signal the end of the run.

> This is intended as a simplification rather than an additional IMU
> feature. [...] There is no need to implement sophisticated lift or
> overturn classification.

So: remove the impact path, add one small stop rule, keep everything else.

## 2. What changed

| Area | Before (impact) | After (lift-stop) |
|---|---|---|
| Sensor setup (`drv_lsm303.c`) | Interrupt generator 1 routed to INT1 (CTRL3 0x20), X/Y-high event (IG_CFG1 0x0A), threshold `IMU_IMPACT_THS` (IG_THS1), 10 ms duration (IG_DUR1), latched (CTRL5 LIR1) | INT1 not routed (CTRL3 0x00), IG_CFG1 0x00, no latch (CTRL5 0x70). `drv_lsm303_read_int1_src()` removed |
| Interrupt | R1 callback `imu_int1_isr` on GP19 (rising) | None. GP19 is `PIN_SPARE_19`; R1 serves GP1 and GP7 only |
| Detection | Hardware: any X/Y acceleration above ~1.5 g | Software, in `imu_task` every 10 ms: `terrain_lift_step()` on the calibrated averages it already reads, plus (since 18 Nov) `terrain_ir_lift_step()` on the two raw IR readings for a level lift (section 3.1) |
| Safety flag K3 | `SAFETY_IMPACT` (0x02), set from the ISR | `SAFETY_LIFT` (0x02, same bit), set by `imu_task` with `safety_set()` |
| Motion class | `APP_MCLASS_IMPACT` (5), held 200 ms | `APP_MCLASS_LIFTED` (5), shown while `SAFETY_LIFT` is set |
| Mission | IMPACT in CAL_IR, run start or RUN → FAULT | LIFT in RUN → **FINISHED, end reason STOP** (the end-of-run signal); LIFT in the CAL_IR sweep or the run start → FAULT |
| Bring-up diagnosis | `IMU_DIAG_IMPACT` INT1 report (snapshots, glitch counter, `imu_diag_service()`) | Removed. One console line `LIFT STOP: robot lifted or overturned (az N mg)` (IMU path) or `LIFT STOP: no floor under the IR sensors (L N R N)` (IR path), and `RUN END: robot lifted (lift-stop)` |

`motion_task` is unchanged: it already brakes at its next cycle while any
K3 bit is set.

## 3. The rule (no classification)

The check is armed only while `APP_MODE_MOTOR_EN` is set (the CAL_IR sweep
and RUN), so carrying the car around with the motors off never latches a
stop. On every cycle that brings new samples:

```
lifted_now = (az < IMU_LIFT_TILT_AZ_MG)                         // 500 mg
          || (|a|^2 < (1000 - IMU_LIFT_DG_MG)^2)                // 300 mg
          || (|a|^2 > (1000 + IMU_LIFT_DG_MG)^2)
LIFT_CNT   = lifted_now ? min(LIFT_CNT + 1, 5) : 0
if LIFT_CNT == IMU_LIFT_CONFIRM_CYCLES (5): safety_set(SAFETY_LIFT)
```

- **Overturned / tipped:** `az < 500 mg` means tilted past 60 deg or upside
  down. Course ramps and humps stay far below 45 deg (assumption A24).
- **Lifted:** a hand lift pushes the car up, so the size of the acceleration
  vector leaves the 0.7 g..1.3 g band. The size does not change with the
  slope, so a ramp cannot trip it. It is compared squared, so there is no
  square root; each axis is clamped to ±8192 mg first, so the sum fits
  `int32_t`.
- **Confirm:** 5 cycles in a row (50 ms), so a short bump on a hump does not
  stop the run. One normal cycle restarts the count.

All three numbers are in `cfg_imu.h` and marked TUNE; S4-I12 and S4-I13 set
them on the car.

### 3.1 Level lift: the IR floor check (S4-U17, added 18 Nov 2026)

On the car on 18 Nov (13:19) a quick lift stopped the run, but a smooth,
level lift did not: the acceleration only leaves the 0.7 g..1.3 g band while
the car speeds up, and a slow straight lift keeps |a| near 1 g with az near
+1000 mg. Off the floor, both IR line sensors read close to full scale
(3980..4067 lifted), higher than any tape on the track (darkest tape that
day 3570, the dark ramp up to 3708 / 3905). So a second rule runs in the
same `imu_task` cycle, also only while `APP_MODE_MOTOR_EN` is set:

```
both_high  = (raw_l > IMU_LIFT_IR_RAW_MIN) && (raw_r > IMU_LIFT_IR_RAW_MIN)   // 3950
IR_LIFT_CNT = both_high ? min(IR_LIFT_CNT + 1, 30) : 0
if IR_LIFT_CNT == IMU_LIFT_IR_CYCLES (30, = 300 ms): safety_set(SAFETY_LIFT)
```

- **Both sensors:** one sensor over a dark patch or off the tape edge never
  counts.
- **300 ms in a row:** longer than a crest or a cross at cruise speed, so a
  short dark stretch on the track cannot trip it.
- **Threshold 3950:** first set to 3850 (13:19), raised to 3950 after the
  13:28 ramp run read up to 3905 on one sensor, which left too little margin.
- The raw values come from line_task's `SH_IR_RAW` word (one writer, read
  once per cycle), so imu_task does not touch the ADC.
- Either rule sets the same K3 bit, so mission and motion behave exactly as
  in section 2.

Both numbers are in `cfg_imu.h` and marked TUNE.

**Response time:** 50 ms confirm + up to 10 ms until the next `imu_task`
cycle + up to 11 ms until `motion_task` brakes, so the motors are low about
70 ms after the lift starts, well inside what a person lifting the car can
notice. A smooth level lift that only the IR check sees stops after the
300 ms confirm, about 320 ms after both sensors lose the floor.

## 4. Minimum-bit data representation (Tribunal 1 change) is kept

Every field still uses the fewest bits its range needs, and the generator
(`tools/gen_layout.py --check`) passes.

| Field | Group | Before | After |
|---|---|---|---|
| `IMPACT_HOLD` 0..31 tick | PRV_IMU_A | 5 bits | removed |
| `IMPACT_SEQ` 0..15 (low bits of the ISR counter) | PRV_IMU_A | 4 bits | removed |
| `LIFT_CNT` 0..5 cycles, SAT | PRV_IMU_A | — | **3 bits** |
| `IR_LIFT_CNT` 0..30 cycles, SAT (level lift, S4-U17) | PRV_IMU_A | — | **5 bits** |
| `IMPACT_SEQ` 0..255 (ISR → task byte) | SH_IMU | 8 bits | group removed |
| `SAFETY_*` bits | K3 event flag | 3 bits (ESTOP, IMPACT, FAULT) | 3 bits (ESTOP, LIFT, FAULT): same bit |
| `MCLASS` 0..6 | SB_IMU0, MQ_STATE, EVT_MCLASS | 3 bits | 3 bits (code 5 renamed LIFTED) |
| `END_REASON` 0..3 | SB_MIS0, K7/MQ_SUMM | 2 bits | 2 bits: a lift ends the run with the existing STOP code, so no wider field |

- PRV_IMU_A now uses 31 of its 32 bits (26 after the change, plus the
  5-bit `IR_LIFT_CNT` of 18 Nov; it was 32 of 32 before the change).
- Application bits for the feature: 17 (5 + 4 + 8) → 3.
- **Shared data:** `SH_IMU` was the only IMU data written by an ISR. With it
  gone, every packed IMU word has a single writer (`imu_task`), so no
  read-modify-write race is possible. K3 stays a kernel event flag, which
  makes each set/clear/read atomic.
- **Messages:** no message changed size. MQ_STATE 17 B, MQ_EVT 2 B, MQ_SUMM
  6 B, K4 16-bit event, K7 sizes as in `docs/data_representation.md`.

## 5. RAM (measured)

`arm-none-eabi-size` on the car image (`make NET=lwip MQTT=1`, SDK 2.2.0,
same toolchain), `main` at `14c3dc1` vs this change:

| Section | Before B | After B | Delta |
|---|---:|---:|---:|
| .data | 340 | 256 | −84 |
| .bss | 99508 | 99464 | −44 |
| **RAM total** | **99848** | **99720** | **−128** |
| .text (flash) | 388332 | 387280 | −1052 |

Symbols that went away (`arm-none-eabi-nm -S`): `g_imu_impact_seq` 1 B,
`g_imu_isr_us` 4 B, `g_imu_int1_glitches` 4 B and the INT1 diagnosis buffers
(`g_dbg_*` 108 B, `s_glitch_*` 8 B): 125 B, plus 3 B of alignment padding.
`g_imu_prv_a` stays 4 B with `LIFT_CNT` inside it.

## 6. Tests

| ID | Level | What | Status |
|---|---|---|---|
| S4-U16 | host | `terrain_lift_step`: flat, 45 deg slope, ±0.2 g, 40 ms bumps, hand lift, upside down, 70 deg tip, out-of-range input, LIFTED class | PASS (12 checks) |
| S5-U78 | host sim | LIFT in RUN → FINISHED (STOP), MOTOR_EN off, LIFT kept until RESET | PASS |
| S5-U32, S5-U33, S5-U16, S5-U07 | host sim | same scenarios with LIFT in place of IMPACT | PASS |
| S2-U14 | host | K3 with the LIFT bit | PASS |
| S4-U17 | host | `terrain_ir_lift_step`: darkest tape 2 s, one sensor high, dark ramp 1 s (3708/3905), 290 ms crest, logged lift values, saturation at 30 | PASS |
| S4-I12 | car | lifts in RUN: motors stop, FINISHED | 18 Nov 13:26 (`3e0bba7`): quick lift stopped on the IMU (az 241 mg), smooth level lift stopped on the IR check (3996/3999). The full 10-lift series is not logged yet |
| S4-I13 | track | runs over humps and ramps: no false stop | 18 Nov 13:28: ramp run, no false stop (IR max 3708/3905); threshold then raised to 3950 for margin. 3 full runs still to log |
| S4-I03 | — | impact taps | **withdrawn** (ID kept, never reused) |

All host suites: 829 checks pass (`make -C testing/host`). Both the normal
and the `TEST=1` images build with zero warnings; `tools/barr_check.sh` has
no findings.
