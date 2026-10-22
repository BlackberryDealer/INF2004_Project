# Data representation

GENERATED from `tools/layout.csv` by `tools/gen_layout.py`. Do not edit;
change the CSV and run `make layout` in `build_make/`.

Stored value = clamp(value, min, max) + offset, in stored LSB units. *Bits
needed* = ceil(log2(max - min + 1)). Class S = shared, P = private,
E = justified exception (natural C type kept; *bits stored* is that type's
width). Kinds: SAT saturates silently at a sentinel; RNG clamps and sets
`APP_FAULT_RANGE_CLAMP`; PAD and RAW are exempt from the minimal-width rule.

## B1

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| VER | MQ_HDR | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_MQ_VERSION |
| TYPE | MQ_HDR | 0..31 × 1 code | 1 | 5 | 5 | ENUM | S | APP_MQ_TYPE_* |
| SYS | MQ_STATE | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | from SB_MIS0 |
| RUN | MQ_STATE | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | from SB_MIS0 |
| MOT | MQ_STATE | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | from SB_MOT0 |
| SPEED_L | MQ_STATE | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | from SB_MOT0 |
| SPEED_R | MQ_STATE | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | from SB_MOT0 |
| POS_L | MQ_STATE | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | from SB_MOT1 |
| POS_R | MQ_STATE | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | from SB_MOT2 |
| LINE | MQ_STATE | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | from SB_LINE0 status |
| STEER | MQ_STATE | -1000..1000 × 1 permille | 1 | 11 | 11 | RNG | S | from SB_LINE0 |
| ODO_MM | MQ_STATE | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | path x um/count / 1000 |
| PITCH | MQ_STATE | -450..450 × 1 0.1 deg | 1 | 10 | 10 | RNG | S | from SB_IMU0 |
| HUMP_PEAK | MQ_STATE | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | from SB_IMU0 |
| TURN | MQ_STATE | -360..360 × 1 deg/s | 1 | 10 | 10 | RNG | S | from SB_IMU1 |
| MCLASS | MQ_STATE | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | from SB_IMU0 |
| FAULTS | MQ_STATE | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | S | fault_get() |
| PAD | MQ_STATE | 0..0 × 1 - | 1 | 0 | 7 | PAD | S | Zero to the byte boundary |
| WORD | MQ_EVT | 0..65535 × 1 word | 1 | 16 | 16 | RAW | S | The 16-bit app event word |
| ANGLE_STEP | MQ_PROF | 0..24 × 1 5 deg | 1 | 5 | 5 | RNG | S | as K7_PROF |
| WIDTH | MQ_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | as K7_PROF |
| NEAREST | MQ_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | as K7_PROF |
| CLEAR_L | MQ_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | as K7_PROF |
| CLEAR_R | MQ_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | as K7_PROF |
| SIDE | MQ_PROF | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | as K7_PROF |
| PAD | MQ_PROF | 0..0 × 1 - | 1 | 0 | 2 | PAD | S | Zero |
| END_REASON | MQ_SUMM | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | as K7_SUMM |
| HUMP_PEAK | MQ_SUMM | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | as K7_SUMM |
| ODO_MM | MQ_SUMM | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | as K7_SUMM |
| FAULTS | MQ_SUMM | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | S | as K7_SUMM |
| RUN_S | MQ_SUMM | 0..1023 × 1 s | 1 | 10 | 10 | RNG | S | as K7_SUMM |
| PAD | MQ_SUMM | 0..0 × 1 - | 1 | 0 | 4 | PAD | S | Zero |
| UPTIME_S | MQ_HB | 0..65535 × 1 s | 1 | 16 | 16 | SAT | S | Seconds since boot |
| DROPS | MQ_HB | 0..255 × 1 count | 1 | 8 | 8 | SAT | S | telem_drops() |
| LAT_MAX_US | MQ_HB | 0..4095 × 1 us | 1 | 12 | 12 | SAT | S | sys_tick_latency_max_us() |
| PAD | MQ_HB | 0..0 × 1 - | 1 | 0 | 4 | PAD | S | Zero |
| CHAR | MQ_MAP | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Byte 1 of a MAP command |
| CMD | MQ_MAP | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | Byte 1 of a MAP command |

## B2

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| SPEED_L | SB_MOT0 | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | Left wheel speed (approx 7 mm/s per LSB) |
| SPEED_R | SB_MOT0 | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | Right wheel speed |
| STATE | SB_MOT0 | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_MOT_* |
| POS_L | SB_MOT1 | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | Left encoder position (A23) |
| POS_R | SB_MOT2 | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | Right encoder position (A23) |
| PATH | SB_MOT3 | 0..131071 × 1 count | 1 | 17 | 17 | RNG | S | Path length sum of \|travel\| (A23) |
| PARAM | MSG_CMD | 0..2047 × 1 mm\|deg\|mm/s | 1 | 11 | 11 | RNG | S | K5 command parameter |
| TYPE | MSG_CMD | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | APP_CMD_* |
| TAG | MSG_CMD | 0..3 × 1 count | 1 | 2 | 2 | RAW | S | Command tag; echoed in MOVE_DONE; wraps |
| TAG | EVT_MOVE_DONE | 0..3 × 1 count | 1 | 2 | 2 | RAW | S | Tag of the finished command |
| LEFT | EVT_STALL | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Left wheel stalled |
| RIGHT | EVT_STALL | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Right wheel stalled |

## B3

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| STEERING | SB_LINE0 | -1000..1000 × 1 permille | 1 | 11 | 11 | RNG | S | Steering error L-R; + = line left |
| STATUS | SB_LINE0 | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_LINE_* |
| JUNCTION | SB_LINE0 | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | APP_JCT_* last junction class |
| CALIBRATED | SB_LINE0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | IR calibration done |
| GAP | MSG_EDGE | 0..32767 × 16 us | 16 | 15 | 15 | SAT | S | Gap since previous accepted edge; 32767 = stopped |
| LEVEL | MSG_EDGE | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | DO level after the edge (1 = black) |
| CLASS | EVT_JUNCTION | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | APP_JCT_* |
| CROSS_MM | EVT_LINE_FOUND | 0..255 × 1 mm | 1 | 8 | 8 | SAT | S | Black crossing length |
| CHAR | EVT_BARCODE | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Code 39 character index |
| CMD | EVT_BARCODE | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_BAR_CMD_* |
| CHAR | EVT_UNMAPPED | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Code 39 character index |

## B4

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| PITCH | SB_IMU0 | -450..450 × 1 0.1 deg | 1 | 10 | 10 | RNG | S | Pitch (A24) |
| HUMP_NOW | SB_IMU0 | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | Current hump height (A25) |
| HUMP_PEAK | SB_IMU0 | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | Highest hump this run (A25) |
| MCLASS | SB_IMU0 | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | APP_MCLASS_* |
| TURN_SRC | SB_IMU0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | 1 = turn rate from encoders (A7) |
| TURN_RATE | SB_IMU1 | -360..360 × 1 deg/s | 1 | 10 | 10 | RNG | S | Turn rate (A26) |
| PEAK_MM | EVT_HUMP | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | Peak of this hump (A25) |
| CLASS | EVT_MCLASS | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | APP_MCLASS_* |
| STATE | PRV_IMU_A | 0..4 × 1 code | 1 | 3 | 3 | ENUM | P | Terrain state |
| HYST | PRV_IMU_A | 0..15 × 1 sample | 1 | 4 | 4 | SAT | P | Hysteresis count |
| I2C_ERR | PRV_IMU_A | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | I2C errors in a row |
| GOOD_CNT | PRV_IMU_A | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | Good reads in FAULT |
| CLASS_GAP | PRV_IMU_A | 0..31 × 1 tick | 1 | 5 | 5 | SAT | P | Ticks since MOTION_CLASS |
| LAST_CLASS | PRV_IMU_A | 0..6 × 1 code | 1 | 3 | 3 | ENUM | P | Last posted class |
| MAG_DIV | PRV_IMU_A | 0..4 × 1 tick | 1 | 3 | 3 | RNG | P | Cycles to next magnetometer read |
| IMPACT_HOLD | PRV_IMU_A | 0..31 × 1 tick | 1 | 5 | 5 | SAT | P | Impact class hold |
| IMPACT_SEQ | PRV_IMU_A | 0..15 × 1 count | 1 | 4 | 4 | RAW | P | Last seen INT1 sequence (low bits) |
| CAL_CNT | PRV_IMU_B | 0..200 × 1 sample | 1 | 8 | 8 | RNG | P | Calibration samples |
| CLIMB_CNT | PRV_IMU_B | 0..2047 × 1 count | 1 | 11 | 11 | SAT | P | Climb distance |
| HIST_IDX | PRV_IMU_B | 0..4 × 1 slot | 1 | 3 | 3 | RNG | P | Speed history index |
| PEAK_NOW | PRV_IMU_B | 0..127 × 1 mm | 1 | 7 | 7 | RNG | P | Peak of this hump (A25) |
| PREV_HDG | PRV_IMU_C | 0..3599 × 1 0.1 deg | 1 | 12 | 12 | RNG | P | Previous magnetometer heading (wrapped 0..3599) |
| PREV_HDG_VALID | PRV_IMU_C | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | PREV_HDG is from the previous 50 ms read |
| PITCH_MAX | PRV_IMU_C | -450..450 × 1 0.1 deg | 1 | 10 | 10 | RNG | P | Max pitch this hump (A24) |
| RUN_PEAK | PRV_IMU_C | 0..127 × 1 mm | 1 | 7 | 7 | RNG | P | Highest hump this run |
| PREV_DIFF | PRV_IMU_D | -262142..262142 × 1 count | 1 | 19 | 19 | RNG | P | pos_l-pos_r 50 ms ago |
| PREV_POS_L | PRV_IMU_E | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | P | Previous left position |
| PREV_POS_R | PRV_IMU_F | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | P | Previous right position |
| H0 | PRV_HIST_A | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | P | Speed history 0 |
| H1 | PRV_HIST_A | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | P | Speed history 1 |
| H2 | PRV_HIST_A | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | P | Speed history 2 |
| H3 | PRV_HIST_A | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | P | Speed history 3 |
| H4 | PRV_HIST_B | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | P | Speed history 4 |
| AX | PRV_OFS_A | -2047..2047 × 2 mg | 2 | 12 | 12 | RNG | P | Accelerometer X offset |
| AY | PRV_OFS_A | -2047..2047 × 2 mg | 2 | 12 | 12 | RNG | P | Accelerometer Y offset |
| AZ | PRV_OFS_B | -2047..2047 × 2 mg | 2 | 12 | 12 | RNG | P | Accelerometer Z offset |
| FIFO_OVR | PRV_OFS_B | 0..255 × 1 count | 1 | 8 | 8 | SAT | P | FIFO overruns |
| ACC | E_IMU | 0..0 × 1 um x mg | 1 | 0 | 32 | RAW | E | E2 height accumulator: arithmetic headroom |
| CAL_SUM | E_IMU | 0..0 × 1 code | 1 | 0 | 32 | RAW | E | E2 calibration sums x3 (live 2 s only) |
| RAW_SAMPLE | E_IMU_SAMPLE | 0..0 × 1 code | 1 | 0 | 16 | RAW | E | E4 accelerometer register image (own E group; SPEC-QUESTION in TEST_LOG) |
| IMPACT_SEQ | SH_IMU | 0..255 × 1 count | 1 | 8 | 8 | RAW | S | R1 callback -> imu_task; wraps |

## B5

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| DIST | SB_OBS0 | 0..511 × 4 mm | 4 | 9 | 9 | SAT | S | Forward distance; 511 = nothing in range |
| SCAN_STATE | SB_OBS0 | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_SCAN_* |
| ANGLE_STEP | SB_OBS0 | 0..24 × 1 5 deg | 1 | 5 | 5 | RNG | S | Obstacle angle = 30 + 5 x step |
| PLAN_SIDE | SB_OBS0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | APP_SIDE_* |
| PROFILE_VALID | SB_OBS0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Profile fields are valid |
| WIDTH | SB_OBS1 | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Obstacle width; 255 = clear (A27) |
| NEAREST | SB_OBS1 | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Closest point |
| CLEAR_L | SB_OBS1 | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Left clearance |
| CLEAR_R | SB_OBS1 | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Right clearance |
| LEG_LAT | SB_OBS2 | 0..255 × 4 mm | 4 | 8 | 8 | RNG | S | Planned lateral leg (NFR16) |
| LEG_FWD | SB_OBS2 | 0..255 × 4 mm | 4 | 8 | 8 | RNG | S | Planned forward leg (NFR16) |
| SYS | SB_MIS0 | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_SYS_* |
| RUN | SB_MIS0 | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | APP_RUN_* |
| AVOID | SB_MIS0 | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | APP_AVOID_* |
| END_REASON | SB_MIS0 | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_END_* |
| ARMED | SB_MIS0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | A barcode command is stored |
| ARMED_CMD | SB_MIS0 | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_BAR_CMD_* |
| VALUE | MSG_EVT | 0..2047 × 1 per type | 1 | 11 | 11 | RNG | S | K4/K7 event value (width per EVT_ group) |
| TYPE | MSG_EVT | 0..23 × 1 code | 1 | 5 | 5 | ENUM | S | APP_EVT_* |
| WIDTH | MSG_ECHO | 0..4095 × 4 us | 4 | 12 | 12 | SAT | S | Echo pulse width; 4095 = no echo |
| SEQ | MSG_ECHO | 0..3 × 1 count | 1 | 2 | 2 | RAW | S | Ping sequence (wraps by design) |
| VALID | MSG_ECHO | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Width is a complete measurement |
| PAD | MSG_ECHO | 0..0 × 1 - | 1 | 0 | 1 | PAD | S | Zero |
| SIDE | EVT_PLAN | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | APP_SIDE_* |
| SOURCE | EVT_CAL_DONE | 0..1 × 1 code | 1 | 1 | 1 | ENUM | S | APP_CAL_SRC_* |
| FAULT_IDX | EVT_SFAULT | 0..9 × 1 code | 1 | 4 | 4 | ENUM | S | APP_FAULT_IDX_* |
| SYS | EVT_STATE | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_SYS_* |
| RUN | EVT_STATE | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | APP_RUN_* |
| CHAR | EVT_MAP | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Character to remap |
| CMD | EVT_MAP | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | New command |
| ANGLE_STEP | K7_PROF | 0..24 × 1 5 deg | 1 | 5 | 5 | RNG | S | Profile angle step |
| WIDTH | K7_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Width |
| NEAREST | K7_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Closest point |
| CLEAR_L | K7_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Left clearance |
| CLEAR_R | K7_PROF | 0..255 × 4 mm | 4 | 8 | 8 | SAT | S | Right clearance |
| SIDE | K7_PROF | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Chosen side |
| END_REASON | K7_SUMM | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_END_* |
| HUMP_PEAK | K7_SUMM | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | Highest hump of the run |
| ODO_MM | K7_SUMM | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | Distance travelled (A23) |
| FAULTS | K7_SUMM | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | S | APP_FAULT_* mask |
| RUN_S | K7_SUMM | 0..1023 × 1 s | 1 | 10 | 10 | RNG | S | Run time (A28) |

## Groups

| Group | Used bits | Storage bits |
|---|---|---|
| SB_LINE0 | 17 | 32 |
| SB_MOT0 | 19 | 32 |
| SB_MOT1 | 18 | 32 |
| SB_MOT2 | 18 | 32 |
| SB_MOT3 | 17 | 32 |
| SB_IMU0 | 28 | 32 |
| SB_IMU1 | 10 | 32 |
| SB_OBS0 | 19 | 32 |
| SB_OBS1 | 32 | 32 |
| SB_OBS2 | 16 | 32 |
| SB_MIS0 | 14 | 32 |
| MSG_CMD | 16 | 16 |
| MSG_EVT | 16 | 16 |
| MSG_ECHO | 16 | 16 |
| MSG_EDGE | 16 | 16 |
| EVT_JUNCTION | 3 | 11 |
| EVT_LINE_FOUND | 8 | 11 |
| EVT_BARCODE | 8 | 11 |
| EVT_UNMAPPED | 6 | 11 |
| EVT_PLAN | 1 | 11 |
| EVT_MOVE_DONE | 2 | 11 |
| EVT_STALL | 2 | 11 |
| EVT_HUMP | 7 | 11 |
| EVT_MCLASS | 3 | 11 |
| EVT_CAL_DONE | 1 | 11 |
| EVT_SFAULT | 4 | 11 |
| EVT_STATE | 6 | 11 |
| EVT_MAP | 8 | 11 |
| K7_PROF | 38 | 40 |
| K7_SUMM | 44 | 48 |
| MQ_HDR | 8 | 8 |
| MQ_STATE | 136 | 136 |
| MQ_EVT | 16 | 16 |
| MQ_PROF | 40 | 40 |
| MQ_SUMM | 48 | 48 |
| MQ_HB | 40 | 40 |
| MQ_MAP | 8 | 8 |
| PRV_IMU_A | 31 | 32 |
| PRV_IMU_B | 29 | 32 |
| PRV_IMU_C | 30 | 32 |
| PRV_IMU_D | 19 | 32 |
| PRV_IMU_E | 18 | 32 |
| PRV_IMU_F | 18 | 32 |
| PRV_HIST_A | 32 | 32 |
| PRV_HIST_B | 8 | 32 |
| PRV_OFS_A | 24 | 32 |
| PRV_OFS_B | 20 | 32 |
| SH_IMU | 8 | 8 |

## Range assumptions

- **A23** One run is at most 32 m (APP_RUN_MAX_MM): encoder positions fit +/-131071 counts, path length 131071 counts, odometry 32767 mm.
- **A24** Pitch stays within +/-45.0 deg (+/-450 x 0.1 deg).
- **A25** A hump is at most 127 mm high.
- **A26** Turn rate stays within +/-360 deg/s.
- **A27** Obstacle width and clearances are reported up to 1020 mm (255 x 4 mm); the top code means clear / out of range.
- **A28** One run lasts at most 1023 s, about 17 min (APP_RUN_MAX_S).
