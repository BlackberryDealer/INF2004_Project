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
| ODO_MM | MQ_STATE | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | path x um/count / 1000; restarts from 0 when the path wraps |
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
| SYS | MQ_SNAP | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | as K7_SNAP |
| RUN | MQ_SNAP | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | as K7_SNAP |
| HUMP_PEAK | MQ_SNAP | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | as K7_SNAP |
| ODO_MM | MQ_SNAP | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | as K7_SNAP |
| FAULTS | MQ_SNAP | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | S | as K7_SNAP |
| RUN_S | MQ_SNAP | 0..1023 × 1 s | 1 | 10 | 10 | RNG | S | as K7_SNAP |
| UPTIME_S | MQ_HB | 0..65535 × 1 s | 1 | 16 | 16 | SAT | S | Seconds since boot |
| DROPS | MQ_HB | 0..255 × 1 count | 1 | 8 | 8 | SAT | S | telem_drops() |
| LAT_MAX_US | MQ_HB | 0..4095 × 1 us | 1 | 12 | 12 | SAT | S | sys_tick_latency_max_us() |
| PAD | MQ_HB | 0..0 × 1 - | 1 | 0 | 4 | PAD | S | Zero |
| IR_L | MQ_SENS | 0..4095 × 1 adc | 1 | 12 | 12 | RNG | S | Raw left IR ADC (SH_IR_RAW); debug telemetry |
| IR_R | MQ_SENS | 0..4095 × 1 adc | 1 | 12 | 12 | RNG | S | Raw right IR ADC (SH_IR_RAW) |
| US_MM | MQ_SENS | 0..511 × 4 mm | 4 | 9 | 9 | SAT | S | Ultrasonic distance; 511 x 4 = 2044 mm = nothing in range |
| BAR_DO | MQ_SENS | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Barcode DO now (1 = black) |
| BAR_EDGES | MQ_SENS | 0..255 × 1 count | 1 | 8 | 8 | RAW | S | Accepted barcode DO edges; low 8 bits; wraps |
| AX | MQ_SENS | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accelerometer X after the CAL_IMU offsets (SH_IMU_ACC) |
| AY | MQ_SENS | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accelerometer Y after the offsets |
| AZ | MQ_SENS | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accelerometer Z after the offsets (flat = +1000) |
| WDT_GAP | MQ_SENS | 0..255 × 250 us | 250 | 8 | 8 | SAT | S | Longest watchdog feed gap in the current 2 s window |
| SCAN | MQ_SENS | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_SCAN_* obstacle scanner state |
| PAD | MQ_SENS | 0..0 × 1 - | 1 | 0 | 5 | PAD | S | Zero |
| CHAR | MQ_MAP | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Byte 1 of a MAP command |
| CMD | MQ_MAP | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | Byte 1 of a MAP command |
| DROPS | PRV_TEL | 0..255 × 1 count | 1 | 8 | 8 | SAT | P | K11 drops; telem_task only |
| HB_DIV | PRV_TEL | 0..4 × 1 count | 1 | 3 | 3 | RNG | P | State cycles since heartbeat |
| UPTIME_HI | PRV_TEL | 0..65535 × 1 s | 1 | 16 | 16 | SAT | P | Uptime seconds |
| DROPS | PRV_TEL_K7 | 0..255 × 1 count | 1 | 8 | 8 | SAT | P | K7 drops; written only in mission_task context |
| CMD_DROPS | PRV_BRIDGE | 0..255 × 1 count | 1 | 8 | 8 | SAT | P | Rejected or rate-limited commands |
| PUB_DROPS | PRV_BRIDGE | 0..255 × 1 count | 1 | 8 | 8 | SAT | P | Payloads abandoned after 2 publish tries |
| CMD_SEEN | PRV_BRIDGE | 0..31 × 1 mask | 1 | 5 | 5 | RAW | P | One bit per command type (START STOP RESET MAP SNAP) set at its first acceptance; a mask so RAW; radio task only |
| BACKOFF | PRV_MQTT | 0..3 × 1 exponent | 1 | 2 | 2 | RNG | P | Back-off 1 2 4 8 s; radio task only |
| PEND_TRIES | PRV_MQTT | 0..2 × 1 count | 1 | 2 | 2 | RNG | P | Publish tries of the pending payload |
| PEND_LEN | PRV_MQTT | 0..20 × 1 byte | 1 | 5 | 5 | RNG | P | Pending payload length (0 = none) |
| PEND_TYPE | PRV_MQTT | 0..31 × 1 code | 1 | 5 | 5 | ENUM | P | APP_MQ_TYPE_* of the pending payload |
| LAST | PRV_CMD_T | 0..65535 × 10 ms | 10 | 16 | 16 | RAW | P | Last accepted command time x5 (START STOP RESET MAP SNAP); 10 ms units; wraps; radio task only |

## B2

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| SPEED_L | SB_MOT0 | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | Left wheel speed (approx 7 mm/s per LSB) |
| SPEED_R | SB_MOT0 | -127..127 × 1 count/50ms | 1 | 8 | 8 | RNG | S | Right wheel speed |
| STATE | SB_MOT0 | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_MOT_* |
| POS_L | SB_MOT1 | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | Left encoder position (A23) |
| POS_R | SB_MOT2 | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | S | Right encoder position (A23) |
| PATH | SB_MOT3 | 0..131071 × 1 count | 1 | 17 | 17 | RAW | S | Path length sum of \|travel\|; wraps at 2^17 counts (about 42.5 m); read only as differences (odometry_path_since) |
| PARAM | MSG_CMD | 0..2047 × 1 mm\|deg\|mm/s | 1 | 11 | 11 | RNG | S | K5 command parameter |
| TYPE | MSG_CMD | 0..6 × 1 code | 1 | 3 | 3 | ENUM | S | APP_CMD_* |
| TAG | MSG_CMD | 0..3 × 1 count | 1 | 2 | 2 | RAW | S | Command tag; echoed in MOVE_DONE; wraps |
| TAG | EVT_MOVE_DONE | 0..3 × 1 count | 1 | 2 | 2 | RAW | S | Tag of the finished command |
| LEFT | EVT_STALL | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Left wheel stalled |
| RIGHT | EVT_STALL | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Right wheel stalled |
| STATE | PRV_MOT_A | 0..7 × 1 code | 1 | 3 | 3 | ENUM | P | APP_MOT_* state of the Motion Executor |
| ENC_OK | PRV_MOT_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Encoder PIO program loaded |
| ACT_TYPE | PRV_MOT_A | 0..6 × 1 code | 1 | 3 | 3 | ENUM | P | APP_CMD_* of the running command (0 = none) |
| ACT_TAG | PRV_MOT_A | 0..3 × 1 count | 1 | 2 | 2 | RAW | P | Tag of the running command; echoed in MOVE_DONE |
| PEND_VALID | PRV_MOT_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | A command waits in the one-slot latch |
| PEND_TYPE | PRV_MOT_A | 0..6 × 1 code | 1 | 3 | 3 | ENUM | P | APP_CMD_* of the latched command |
| PEND_TAG | PRV_MOT_A | 0..3 × 1 count | 1 | 2 | 2 | RAW | P | Tag of the latched command |
| PEND_PARAM | PRV_MOT_A | 0..2047 × 1 mm\|deg\|mm/s | 1 | 11 | 11 | RNG | P | Parameter of the latched command (K5 PARAM width) |
| CREEP_STILL | PRV_MOT_A | 0..15 × 1 tick | 1 | 4 | 4 | SAT | P | Still cycles inside the creep zone (MOTION_CREEP_STUCK_TICKS) |
| BASE_CPW | PRV_MOT_B | -63..63 × 1 count/50ms | 1 | 7 | 7 | RNG | P | Base set-point; FOLLOW max 300 mm/s = 46 |
| CREEP_CPW | PRV_MOT_B | -15..15 × 1 count/50ms | 1 | 5 | 5 | RNG | P | Creep set-point with the sign of the base; 80 mm/s = 12 |
| STALL_L | PRV_MOT_B | 0..30 × 1 tick | 1 | 5 | 5 | SAT | P | Left wheel: cycles driven hard without a count (MOTION_STALL_TICKS) |
| STALL_R | PRV_MOT_B | 0..30 × 1 tick | 1 | 5 | 5 | SAT | P | Right wheel: cycles driven hard without a count |
| LOG_PEND | PRV_MOT_B | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Impossible encoder read recorded; printed after step 9 |
| LOG_WHEEL | PRV_MOT_B | 0..1 × 1 code | 1 | 1 | 1 | ENUM | P | Wheel of the recorded read (0 = L 1 = R) |
| TARGET2 | PRV_MOT_C | 0..16383 × 1 half count | 1 | 14 | 14 | RNG | P | 2 x counts of the running MOVE or TURN (2047 mm = 12636) |
| PROG2 | PRV_MOT_C | 0..16383 × 1 half count | 1 | 14 | 14 | SAT | P | Progress of the running MOVE or TURN; saturates above every target |
| STRK_L | PRV_MOT_C | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | Left wheel: impossible encoder reads in a row |
| STRK_R | PRV_MOT_C | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | Right wheel: impossible encoder reads in a row |
| PATH2 | PRV_MOT_D | 0..262143 × 1 half count | 1 | 18 | 18 | RAW | P | 2 x PATH; wraps with the 17-bit PATH |
| DIFF_START | PRV_MOT_E | 0..65535 × 1 count | 1 | 16 | 16 | RAW | P | Low 16 bits of pos_l - pos_r when the MOVE started; the drift is taken modulo 2^16 |
| LOG_DT | PRV_MOT_F | 0..1000000 × 1 us | 1 | 20 | 20 | RNG | P | Time since the trusted read of the recorded read (capped at 1 s) |
| INTEG | PRV_MOT_PI | -76800..76800 × 1 permille/256 | 1 | 18 | 18 | RNG | P | PI integrator per wheel x2 (MOTION_I_CLAMP) |
| DUTY | PRV_MOT_PI | -1000..1000 × 1 permille | 1 | 11 | 11 | RNG | P | Duty applied last cycle per wheel x2 |
| DELTA | PRV_WIN_L | -63..63 × 1 count | 1 | 7 | 7 | RNG | P | Speed window: left counts per tick x5 |
| DELTA | PRV_WIN_R | -63..63 × 1 count | 1 | 7 | 7 | RNG | P | Speed window: right counts per tick x5 |
| DT_MS | PRV_WIN_T | 0..63 × 1 ms | 1 | 6 | 6 | SAT | P | Speed window: tick length x5 |
| IDX | PRV_WIN_I | 0..4 × 1 slot | 1 | 3 | 3 | RNG | P | Speed window: next slot |
| COUNT | E_MOT_RAW | 0..0 × 1 count | 1 | 0 | 32 | RAW | E | E3 encoder images x4 (at ZERO and last trusted; L and R): the PIO counter is 32 bits and wraps and a jump is judged on its full width (bits=31 for gen_layout.py) |
| TIME | E_MOT_US | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 times x3 (last read; last trusted read L and R): the 1 us timer is 32 bits and wraps and only differences are used (bits=31 for gen_layout.py) |
| DELTA | E_MOT_LOG | 0..0 × 1 count | 1 | 0 | 32 | RAW | E | Counts of the recorded impossible encoder read: any 32-bit jump; console only (bits=31 for gen_layout.py) |

## B3

| Field | Group | Range | LSB | Bits needed | Bits stored | Kind | Class | Description |
|---|---|---|---|---|---|---|---|---|
| STEERING | SB_LINE0 | -1000..1000 × 1 permille | 1 | 11 | 11 | RNG | S | Steering error L-R; + = line left |
| STATUS | SB_LINE0 | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_LINE_* |
| JUNCTION | SB_LINE0 | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_JCT_* last junction class |
| CALIBRATED | SB_LINE0 | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | IR calibration done |
| GAP | MSG_EDGE | 0..32767 × 16 us | 16 | 15 | 15 | SAT | S | Gap since previous accepted edge; 32767 = stopped |
| LEVEL | MSG_EDGE | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | DO level after the edge (1 = black) |
| CLASS | EVT_JUNCTION | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_JCT_* |
| CROSS_MM | EVT_LINE_FOUND | 0..255 × 1 mm | 1 | 8 | 8 | SAT | S | Black crossing length |
| CHAR | EVT_BARCODE | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Code 39 character index |
| CMD | EVT_BARCODE | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | APP_BAR_CMD_* |
| CHAR | EVT_UNMAPPED | 0..42 × 1 code | 1 | 6 | 6 | ENUM | S | Code 39 character index |
| N_ELEM | EVT_INVALID | 0..63 × 1 count | 1 | 6 | 6 | SAT | S | Elements in the failed capture (a barcode has 29); PRV_BAR_A.N_ELEM |
| MM | EVT_BARCODE_PROBE | 0..2047 × 1 mm | 1 | 11 | 11 | RNG | S | Travel since the capture started (rewind length) |
| WHITE | PRV_CAL_L | 0..4095 × 1 code | 1 | 12 | 12 | RNG | P | Left white level |
| BLACK | PRV_CAL_L | 0..4095 × 1 code | 1 | 12 | 12 | RNG | P | Left black level |
| WHITE | PRV_CAL_R | 0..4095 × 1 code | 1 | 12 | 12 | RNG | P | Right white level |
| BLACK | PRV_CAL_R | 0..4095 × 1 code | 1 | 12 | 12 | RNG | P | Right black level |
| LOST_CNT | PRV_LINE_A | 0..31 × 1 tick | 1 | 5 | 5 | SAT | P | White cycles |
| LOST_SENT | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | LINE_LOST posted |
| JCT_STATE | PRV_LINE_A | 0..2 × 1 code | 1 | 2 | 2 | ENUM | P | IDLE CANDIDATE CONFIRM |
| SAT_L | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Left saturated seen |
| SAT_R | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Right saturated seen |
| SEQ0 | PRV_LINE_A | 0..255 × 1 count | 1 | 8 | 8 | RAW | P | Branch sequence at start |
| LAST_SEQ | PRV_LINE_D | 0..255 × 1 count | 1 | 8 | 8 | RAW | P | Branch sequence seen last cycle |
| BRANCH_PATH | PRV_LINE_D | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path when the branch sequence last changed (wrapping path) |
| RUN_ACTIVE | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Black run in progress |
| CAL_ACTIVE | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Calibration mode seen |
| CALIBRATED | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Calibration valid |
| JCT_MARK | PRV_LINE_B | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at junction start (wrapping path) |
| RUN_MARK | PRV_LINE_C | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at black run start (wrapping path) |
| REJOIN_WHITE | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Rejoin latch: both sensors saw white since the last black run |
| CROSS_STRT | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Steering bias cleared until LINE_JUNCTION_EXIT_MM past CROSS_MARK |
| BRANCH_SEEN | PRV_LINE_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | A branch sequence change is still recent |
| LAST_JCT | PRV_LINE_A | 0..3 × 1 code | 1 | 2 | 2 | ENUM | P | APP_JCT_* last junction class posted |
| RAW_LOG_CNT | PRV_LINE_A | 0..49 × 1 tick | 1 | 6 | 6 | RNG | P | Bring-up raw print divider (LINE_RAW_LOG_CYCLES 50) |
| WHITE_MARK | PRV_LINE_E | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path where the current both-white stretch began (S3-U19; wrapping path) |
| CROSS_MARK | PRV_LINE_F | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at the last cycle with both sensors on a stripe (wrapping path) |
| STATE | PRV_BAR_A | 0..3 × 1 code | 1 | 2 | 2 | ENUM | P | Decoder state |
| N_ELEM | PRV_BAR_A | 0..63 × 1 count | 1 | 6 | 6 | SAT | P | Elements captured |
| LAST_LEVEL | PRV_BAR_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Level of the last entry |
| CLAMP_AGE | PRV_BAR_A | 0..29 × 1 count | 1 | 5 | 5 | SAT | P | Elements stored after the last clamped gap; 29 = none in the decode window (S3-U48) |
| MARK | PRV_BAR_B | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at last edge or cooldown start (wrapping path) |
| CARRY | PRV_BAR_B | 0..32767 × 16 us | 16 | 15 | 15 | SAT | P | Flicker time carried into the next element (S3-U37); saturates like MSG_EDGE GAP |
| FIX_CNT | PRV_BAR_C | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | Level fixes in this capture (BAR_LEVEL_FIX_MAX) |
| SETTLE | PRV_BAR_C | 0..249 × 16 us | 16 | 8 | 8 | RNG | P | Time since the last accepted DO transition while under BAR_SETTLE_US (S3-U37) |
| REREAD_FIRST | PRV_BAR_C | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Verified reread lead-in: the first transition starts the capture |
| PROBE_OK | PRV_BAR_C | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Probe accepted the first bars of this capture |
| QUIET_KNOWN | PRV_BAR_C | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Sensor known to be on white with no edge behind it |
| START_PATH | PRV_BAR_C | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at capture start (wrapping path) |
| HEAD | PRV_BAR_LOG | 0..47 × 1 slot | 1 | 6 | 6 | RNG | P | Raw edge log next slot (BAR_EDGE_LOG_MAX 48; bring-up log S3-U36) |
| CNT | PRV_BAR_LOG | 0..48 × 1 count | 1 | 6 | 6 | SAT | P | Raw edge log entries stored |
| LEFT | PRV_BAR_LOG | 0..48 × 1 count | 1 | 6 | 6 | RNG | P | Raw edge log entries still to print |
| CYCLE | PRV_BAR_LOG | 0..255 × 1 count | 1 | 8 | 8 | RAW | P | barcode_task cycle number; wraps (log keeps 8 bits; print every 4 cycles) |
| WIDTH | PRV_BAR_W | 0..255 × 100 um | 100 | 8 | 8 | SAT | P | Element width x32 bytes |
| CHAR | PRV_BAR_MAP | 0..63 × 1 code | 1 | 6 | 6 | ENUM | S | Map character (63 = empty) x8; mission_task writes one byte per update (MAP) and barcode_task reads whole bytes |
| CMD | PRV_BAR_MAP | 0..3 × 1 code | 1 | 2 | 2 | ENUM | S | Map command x8 (same byte as CHAR) |
| BRANCH_SEQ | SH_BAR | 0..255 × 1 count | 1 | 8 | 8 | RAW | S | barcode_task -> line_task; wraps |
| OVERFLOW | SH_BAR_OVF | 0..255 × 1 count | 1 | 8 | 8 | SAT | S | Ring overflows (ISR writer) |
| COUNT | SH_BAR_EDGES | 0..65535 × 1 count | 1 | 16 | 16 | RAW | S | Accepted DO edges (ISR writer); wraps; readers compare for change or keep the low 8 bits |
| RECIP | E_LINE | 0..0 × 1 Q16 | 1 | 0 | 32 | RAW | E | E8 precomputed reciprocals x2 |
| HEAD_TAIL | E_BAR | 0..0 × 1 index | 1 | 0 | 8 | RAW | E | E6 ring head (ISR) and tail (task): one writer per byte |
| LAST_US | E_BAR_US | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 ISR-private last accepted edge time (own E group and bits=31 for gen_layout.py; SPEC-QUESTION SQ-B3-1) |
| L | SH_IR_RAW | 0..4095 × 1 adc | 1 | 12 | 12 | RNG | S | Raw left IR ADC; line_task writes the whole word each cycle; readers load it once |
| R | SH_IR_RAW | 0..4095 × 1 adc | 1 | 12 | 12 | RNG | S | Raw right IR ADC |

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
| LIFT_CNT | PRV_IMU_A | 0..5 × 1 tick | 1 | 3 | 3 | SAT | P | Lift-stop: cycles in a row lifted or overturned (CR2) |
| NEED_CFG | PRV_IMU_A | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | LSM303D registers must be (re)written |
| IR_LIFT_CNT | PRV_IMU_A | 0..30 × 1 tick | 1 | 5 | 5 | SAT | P | Level lift: cycles in a row both IR read no floor (S4-U17) |
| CAL_CNT | PRV_IMU_B | 0..200 × 1 sample | 1 | 8 | 8 | RNG | P | Calibration samples |
| CLIMB_CNT | PRV_IMU_B | 0..2047 × 1 count | 1 | 11 | 11 | SAT | P | Climb distance |
| HIST_IDX | PRV_IMU_B | 0..4 × 1 slot | 1 | 3 | 3 | RNG | P | Speed history index |
| PEAK_NOW | PRV_IMU_B | 0..127 × 1 mm | 1 | 7 | 7 | RNG | P | Peak of this hump (A25) |
| PREV_HDG | PRV_IMU_C | 0..3599 × 1 0.1 deg | 1 | 12 | 12 | RNG | P | Previous magnetometer heading (wrapped 0..3599) |
| PREV_HDG_VALID | PRV_IMU_C | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | PREV_HDG is from the previous 50 ms read |
| PITCH_MAX | PRV_IMU_C | -450..450 × 1 0.1 deg | 1 | 10 | 10 | RNG | P | Max pitch this hump (A24) |
| RUN_PEAK | PRV_IMU_C | 0..127 × 1 mm | 1 | 7 | 7 | RNG | P | Highest hump this run |
| PREV_DIFF | PRV_IMU_D | -262142..262142 × 1 count | 1 | 19 | 19 | RNG | P | pos_l-pos_r 50 ms ago |
| STATE | PRV_IMU_R | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | One-wheel hump: 1 = on it (S4-U18) |
| HYST | PRV_IMU_R | 0..7 × 1 sample | 1 | 3 | 3 | SAT | P | One-wheel hump: cycles in a row past the start / level threshold |
| FILT | PRV_IMU_R | -511..511 × 1 mg | 1 | 10 | 10 | RNG | P | One-wheel hump: smoothed sideways tilt (ay) |
| PEAK | PRV_IMU_R | 0..511 × 1 mg | 1 | 9 | 9 | RNG | P | One-wheel hump: largest smoothed \|ay\| on this hump |
| TICKS | PRV_IMU_R | 0..255 × 1 tick | 1 | 8 | 8 | SAT | P | One-wheel hump: cycles on this hump (ramp / held-tilt reject) |
| PREV_POS_L | PRV_IMU_E | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | P | Previous left position |
| PREV_POS_R | PRV_IMU_F | -131071..131071 × 1 count | 1 | 18 | 18 | RNG | P | Previous right position |
| PITCH | PRV_IMU_G | -1800..1800 × 1 0.1 deg | 1 | 12 | 12 | RNG | P | Pitch this cycle before the board clamp (atan2 range); read for the board and the magnetometer check |
| HUMP_NOW | PRV_IMU_G | 0..2148 × 1 mm | 1 | 12 | 12 | RNG | P | Hump height this cycle before the board clamp (terrain_acc_to_mm of a positive int32 is at most 2148) |
| CAL_ACTIVE | PRV_IMU_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | IMU calibration mode seen |
| CAL_DONE | PRV_IMU_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Offsets computed for this calibration |
| TURN_ENC | PRV_IMU_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Reported turn rate is from the encoders (A7) |
| AX | PRV_IMU_H | -8191..8191 × 1 mg | 1 | 14 | 14 | RNG | P | Calibrated average X: int16 sample x 61/500 minus an offset of at most 4096 mg (\|AX\| <= 8093) |
| AY | PRV_IMU_H | -8191..8191 × 1 mg | 1 | 14 | 14 | RNG | P | Calibrated average Y (same range as AX) |
| AZ | PRV_IMU_J | -8191..8191 × 1 mg | 1 | 14 | 14 | RNG | P | Calibrated average Z (same range as AX); 1000 at boot |
| A_FWD | PRV_IMU_J | -4095..4095 × 1 mg | 1 | 13 | 13 | RNG | P | Forward acceleration from the 50 ms speed history: 254 cpw x gain / 1024 (\|A_FWD\| <= 3615) |
| TURN_DPS | PRV_IMU_K | -2097151..2097151 × 1 deg/s | 1 | 22 | 22 | RNG | P | Turn rate before the board clamp: encoder heading difference / 5 (\|rate\| <= 1677708) or magnetometer x 2 (<= 3600) |
| ENC_HDG | PRV_IMU_L | -4194303..4194303 × 1 0.01 deg | 1 | 23 | 23 | RNG | P | Encoder heading at the last magnetometer cycle: wheel difference (A23) x 16 (\|heading\| <= 4194272) |
| MX | PRV_IMU_M | -32768..32767 × 1 code | 1 | 16 | 16 | RAW | P | Magnetometer X register image of this cycle |
| MY | PRV_IMU_M | -32768..32767 × 1 code | 1 | 16 | 16 | RAW | P | Magnetometer Y register image of this cycle |
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
| AX | SH_IMU_ACC | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accel X after offsets; imu_task writes the whole word each cycle |
| AY | SH_IMU_ACC | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accel Y after offsets |
| AZ | SH_IMU_ACC | -511..511 × 4 mg | 4 | 10 | 10 | SAT | S | Accel Z after offsets |

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
| TYPE | MSG_EVT | 0..25 × 1 code | 1 | 5 | 5 | ENUM | S | APP_EVT_* |
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
| SYS | K7_SNAP | 0..7 × 1 code | 1 | 3 | 3 | ENUM | S | APP_SYS_* when the snapshot was taken |
| RUN | K7_SNAP | 0..4 × 1 code | 1 | 3 | 3 | ENUM | S | APP_RUN_* when the snapshot was taken |
| HUMP_PEAK | K7_SNAP | 0..127 × 1 mm | 1 | 7 | 7 | RNG | S | Highest hump this run |
| ODO_MM | K7_SNAP | 0..32767 × 1 mm | 1 | 15 | 15 | RNG | S | Distance this run so far (A23) |
| FAULTS | K7_SNAP | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | S | APP_FAULT_* mask |
| RUN_S | K7_SNAP | 0..1023 × 1 s | 1 | 10 | 10 | RNG | S | Run time so far; frozen once the run ends (A28) |
| MEASURED | PRV_OBS_M | 0..33554431 × 1 mask | 1 | 25 | 25 | RAW | P | Scan steps measured (bit n = step n) |
| STATE | PRV_OBS_M | 0..7 × 1 code | 1 | 3 | 3 | ENUM | P | APP_SCAN_* scanner state |
| SETTLE | PRV_OBS_M | 0..4 × 1 ping | 1 | 3 | 3 | RNG | P | Servo settle pings left (OBS_WATCH_SETTLE_PINGS) |
| ON_RAMP | PRV_OBS_M | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Pitched or just off a ramp (S5-U42) |
| INFERRED | PRV_OBS_I | 0..33554431 × 1 mask | 1 | 25 | 25 | RAW | P | Scan steps filled from neighbours (bit n = step n) |
| STEP | PRV_OBS_I | 0..24 × 1 5 deg | 1 | 5 | 5 | RNG | P | Current scan step (30 + 5 x step deg) |
| SAMPLE_CNT | PRV_OBS_I | 0..2 × 1 ping | 1 | 2 | 2 | RNG | P | Pings taken at this step (OBS_SCAN_SAMPLES - 1) |
| PREV | PRV_OBS_A | 0..511 × 4 mm | 4 | 9 | 9 | SAT | P | Previous reading; 511 = nothing (APP_DIST_NONE_MM) |
| ECHO | PRV_OBS_A | 0..511 × 4 mm | 4 | 9 | 9 | SAT | P | Last real echo in APPROACH (S5-U38); 511 = nothing |
| WARMUP | PRV_OBS_A | 0..34 × 1 ping | 1 | 6 | 6 | RNG | P | Power-up grace pings left (OBS_WARMUP_PINGS; S5-U30) |
| SILENT | PRV_OBS_A | 0..3 × 1 ping | 1 | 2 | 2 | SAT | P | Silent pings in a row (OBS_NO_ECHO_PINGS) |
| LOST | PRV_OBS_A | 0..3 × 1 ping | 1 | 2 | 2 | SAT | P | Lost echoes in APPROACH (stop at OBS_LOST_PINGS = 2) |
| FAR | PRV_OBS_A | 0..3 × 1 ping | 1 | 2 | 2 | SAT | P | Far readings in APPROACH (clear at OBS_CLEAR_READINGS = 2) |
| PASS_MARK | PRV_OBS_B | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at PASSING entry (wrapping path) |
| RAMP_HOLD | PRV_OBS_B | 0..8 × 1 ping | 1 | 4 | 4 | RNG | P | Ramp hold pings left (OBS_RAMP_HOLD_PINGS) |
| MODE_SEEN | PRV_OBS_B | 0..7 × 1 mask | 1 | 3 | 3 | RAW | P | RESCAN / STOPPED / RESUME mode bits seen last cycle |
| DIST_LOG | PRV_OBS_B | 0..7 × 1 ping | 1 | 3 | 3 | RNG | P | Bring-up distance print divider (OBS_DIST_LOG_PINGS 8) |
| RAMP_LOGGED | PRV_OBS_B | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | One RAMP line per approach |
| CREEP_LOGGED | PRV_OBS_B | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | One CREEP line per stop |
| RAMP_MARK | PRV_OBS_C | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at the last pitched cycle (wrapping path) |
| EDGE_LO_D | PRV_OBS_C | 0..20 × 4 mm | 4 | 5 | 5 | RNG | P | Low-side corner echo below EDGE_LIMIT (0..OBS_CORE_BAND_MM; S5-U83) |
| EDGE_HI_D | PRV_OBS_C | 0..20 × 4 mm | 4 | 5 | 5 | RNG | P | High-side corner echo below EDGE_LIMIT (0..OBS_CORE_BAND_MM; S5-U83) |
| EDGE_PHASE | PRV_OBS_E | 0..2 × 1 code | 1 | 2 | 2 | ENUM | P | 1-degree edge pass (S5-U83): 0 = off; 1 = low-angle edge; 2 = high-angle edge |
| EDGE_SUB | PRV_OBS_E | 0..5 × 1 deg | 1 | 3 | 3 | RNG | P | Degrees past the start point now measured (1..OBS_EDGE_SUB_MAX) |
| EDGE_LO | PRV_OBS_E | 0..150 × 1 deg | 1 | 8 | 8 | RNG | P | Outermost in-band servo angle on the low-angle side |
| EDGE_HI | PRV_OBS_E | 0..150 × 1 deg | 1 | 8 | 8 | RNG | P | Outermost in-band servo angle on the high-angle side |
| EDGE_LIMIT | PRV_OBS_E | 0..255 × 4 mm | 4 | 8 | 8 | SAT | P | Core band limit (shortest echo + OBS_CORE_BAND_MM); 255 = clear |
| EDGE_OK_LO | PRV_OBS_E | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Low-side edge confirmed: the next degree out jumped to background |
| EDGE_OK_HI | PRV_OBS_E | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | High-side edge confirmed: the next degree out jumped to background |
| EDGE_GRACE | PRV_OBS_E | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | One more degree probed after a non-jump reading past the edge |
| MM | PRV_OBS_SCAN | 0..255 × 4 mm | 4 | 8 | 8 | SAT | P | Scan table point x25 (APP_SCAN_STEPS); 255 = clear (APP_PROFILE_CLEAR_MM) |
| MM | PRV_OBS_SMP | 0..255 × 4 mm | 4 | 8 | 8 | SAT | P | Pings at one step x3 (OBS_SCAN_SAMPLES); median kept |
| COUNT | SH_ECHO_CNT | 0..255 × 1 count | 1 | 8 | 8 | RAW | S | Accepted echo rises and falls x2 (ISR writer; one byte each); obstacle_task compares for change |
| SEQ | SH_PING | 0..255 × 1 count | 1 | 8 | 8 | RAW | S | Ping number; obstacle_task writes; the ISR reads; wraps |
| OPEN | SH_ECHO_WIN | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | Echo window open; obstacle_task writes; the ISR reads (own byte: one writer) |
| RISE_SEQ | PRV_ECHO_ISR | 0..255 × 1 count | 1 | 8 | 8 | RAW | P | Ping number captured at the rise (ISR only) |
| ARMED | PRV_ECHO_ARM | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | A rise is armed (ISR only) |
| TIME | E_OBS_US | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 timer images x2 (echo rise time in the ISR; scan start for the scan-time log): a stuck-high echo or a scan can exceed 65 ms (bits=31 for gen_layout.py) |
| BTN_RUN | PRV_MIS_G | 0..3 × 1 tick | 1 | 2 | 2 | SAT | P | Run button pressed polls (MISSION_BTN_DEBOUNCE_TICKS) |
| BTN_RESET | PRV_MIS_G | 0..200 × 1 tick | 1 | 8 | 8 | SAT | P | Reset button held polls (MISSION_BTN_RESET_HOLD_TICKS) |
| RESET_SENT | PRV_MIS_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | RESET_HOLD reported for this hold |
| WARNING | PRV_MIS_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | 1 s start warning on |
| LED | PRV_MIS_G | 0..6 × 1 code | 1 | 3 | 3 | ENUM | P | HMI_LED_* colour last written; 6 = unknown (forces the first write) |
| BUZZ | PRV_MIS_G | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Buzzer is on |
| PREV_SYS | PRV_MIS_G | 0..8 × 1 code | 1 | 4 | 4 | ENUM | P | APP_SYS_* shown last pass; 8 = none yet |
| BEEP | PRV_MIS_G | 0..2 × 1 code | 1 | 2 | 2 | ENUM | P | HMI_BEEP_* pattern in progress |
| TIME | E_MIS_US | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 timer images x2 (HMI state entry; beep start): blink and alarm periods exceed 65 ms (bits=31 for gen_layout.py) |
| NORMAL_WIN | PRV_HLT | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Watchdog runs the normal 30 ms window (else the 500 ms startup one): selects the LOAD value to feed |
| WDT_ACTIVE | PRV_HLT | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Watchdog enabled |
| STARTED | PRV_HLT | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | All four tasks checked in; normal window armed |
| DONE | SH_OBS_CHK | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | S | obstacle_task has checked in (obstacle_task writes; mission_task reads; own byte) |
| TIME | E_HLT_US | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 timer images x3 (obstacle check-in; watchdog enable; last feed): ages are compared with limits above 65 ms (bits=31 for gen_layout.py) |
| MAX_US | E_HLT_GAP | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E7 longest gap between two watchdog feeds (bring-up latency figure; bits=31 for gen_layout.py) |
| DIFF_AT_STOP | PRV_MIS_A | -262142..262142 × 1 count | 1 | 19 | 19 | RNG | P | pos_l - pos_r when the bypass started (A23) |
| REJOIN_E | PRV_MIS_A | 0..1442 × 1 mm | 1 | 11 | 11 | RNG | P | Expected rejoin distance: LEG_LAT x 1.414 |
| END_REASON | PRV_MIS_A | 0..3 × 1 code | 1 | 2 | 2 | ENUM | P | APP_END_* |
| CAL_DIFF0 | PRV_MIS_B | -262142..262142 × 1 count | 1 | 19 | 19 | RNG | P | pos_l - pos_r when CAL_IR started (A23) |
| LAST_FAULTS | PRV_MIS_B | 0..1023 × 1 mask | 1 | 10 | 10 | RAW | P | APP_FAULT_* mask seen last pass |
| SYS_STATE | PRV_MIS_B | 0..7 × 1 code | 1 | 3 | 3 | ENUM | P | APP_SYS_* |
| RETRY_PARAM | PRV_MIS_C | -131071..131071 × 1 mm\|deg | 1 | 18 | 18 | RNG | P | Parameter to retry; the largest caller is the CAL_IR return turn (\|heading\| / 100) |
| BAR_BACK_LEFT | PRV_MIS_C | 0..600 × 1 mm | 1 | 10 | 10 | RNG | P | Rewind still to send (<= MISSION_BARCODE_REWIND_MM) |
| SYS_REQUEST | PRV_MIS_C | 0..8 × 1 code | 1 | 4 | 4 | ENUM | P | APP_SYS_* requested by a RUN handler; 8 = none (MISSION_REQ_NONE) |
| BAR_MARK | PRV_MIS_D | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Rewind target; progress or read start (wrapping path) |
| MOVE_LIMIT_MS | PRV_MIS_D | 0..21469 × 1 ms | 1 | 15 | 15 | RNG | P | Deadline of the tracked move: 2047 mm at 150 mm/s x 1.5 + 1 s at most |
| BAR_PROBE_MARK | PRV_MIS_E | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Pending rewind target (wrapping path) |
| LEG_LAT | PRV_MIS_E | 0..1020 × 1 mm | 1 | 10 | 10 | RNG | P | Planned lateral leg (from SB_OBS2) |
| RUN_STATE | PRV_MIS_E | 0..4 × 1 code | 1 | 3 | 3 | ENUM | P | APP_RUN_* |
| ARMED_CMD | PRV_MIS_E | 0..3 × 1 code | 1 | 2 | 2 | ENUM | P | APP_BAR_CMD_* |
| BAR_REARM_PATH | PRV_MIS_F | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path when the last rewind or read ended |
| LEG_FWD | PRV_MIS_F | 0..1020 × 1 mm | 1 | 10 | 10 | RNG | P | Planned forward leg (from SB_OBS2) |
| AVOID_STEP | PRV_MIS_F | 0..6 × 1 code | 1 | 3 | 3 | ENUM | P | APP_AVOID_* |
| NO_PATH_CNT | PRV_MIS_F | 0..3 × 1 count | 1 | 2 | 2 | SAT | P | NO_PATH answers for this obstacle (counts to 2; not read) |
| BAR_FOLLOW | PRV_MIS_H | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Start of the current following stretch |
| ARMED_CHAR | PRV_MIS_H | 0..63 × 1 code | 1 | 6 | 6 | RNG | P | Decoded Code 39 index (RUN_BAR_CHAR_MASK) |
| SUBSTEP | PRV_MIS_H | 0..5 × 1 step | 1 | 3 | 3 | RNG | P | Step in a sweep / turn / rejoin (RUN_SUB_* or MISSION_CAL_SUB_*) |
| BAR_PHASE | PRV_MIS_H | 0..4 × 1 code | 1 | 3 | 3 | ENUM | P | RUN_BAR_* rewind / reread phase |
| RETRY_KIND | PRV_MIS_H | 0..4 × 1 code | 1 | 3 | 3 | ENUM | P | MISSION_MV_* to retry |
| BAR_BACK_PATH | PRV_MIS_J | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path when the reverse command started |
| BAR_TRIES | PRV_MIS_J | 0..2 × 1 count | 1 | 2 | 2 | RNG | P | Slow passes retried for this strip (MISSION_BARCODE_RETRIES) |
| SEARCH_FIRST | PRV_MIS_J | 0..3 × 1 code | 1 | 2 | 2 | ENUM | P | MISSION_MV_LEFT / RIGHT: first search sweep |
| CAL_CORR_CNT | PRV_MIS_J | 0..2 × 1 count | 1 | 2 | 2 | RNG | P | Heading return turns after the sweep (MISSION_CAL_CORR_MAX) |
| WAIT_TAG | PRV_MIS_J | 0..3 × 1 count | 1 | 2 | 2 | RAW | P | Tag of the awaited move (MSG_CMD TAG) |
| ARMED | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | A barcode command is stored |
| PROBE_PENDING | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | PROBE_MARK holds a target |
| REARM_WAIT | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | No rewind until REARM mm past REARM_PATH |
| SIDE | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | Planned bypass side (APP_SIDE_*) |
| WARNING | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | is warning |
| RESUME_PENDING | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | resume pending |
| INHIBIT_PEND | PRV_MIS_J | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | inhibit pending |
| BAR_EDGE_PATH | PRV_MIS_K | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at the last edge or black sample |
| FR9_RUNNING | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | fr9 running |
| WAIT_STILL | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | wait standstill |
| CREEPING | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | creeping |
| CREPT | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | crept |
| HAS_RUN | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | has run |
| POST_AVOID | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | post avoid slow |
| QUIET | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | is quiet |
| MOVE_PENDING | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | move pending |
| RETRY_PENDING | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | retry pending |
| MOTION_FAILED | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | motion failed |
| IMU_VALID | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | imu valid |
| IR_VALID | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | ir valid |
| WDT_ENABLED | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | wdt enabled |
| RUN_STARTING | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | run starting |
| ZERO_DONE | PRV_MIS_K | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | zero done |
| PATH_AT_STOP | PRV_MIS_L | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path when the bypass started |
| RECOVERING | PRV_MIS_L | 0..1 × 1 flag | 1 | 1 | 1 | FLAG | P | recovering |
| RUN_ODO_MARK | PRV_MIS_M | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path when RUN_ODO was last summed |
| RUN_ODO | PRV_MIS_N | 0..131071 × 1 count | 1 | 17 | 17 | RNG | P | Path travelled this run; saturates at MISSION_ODO_MAX_COUNTS |
| REJOIN_START | PRV_MIS_P | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at the rejoin move |
| INHIBIT_MARK | PRV_MIS_Q | 0..131071 × 1 count | 1 | 17 | 17 | RAW | P | Path at which the inhibits clear |
| BAR_EDGES | PRV_MIS_R | 0..65535 × 1 count | 1 | 16 | 16 | RAW | P | Last observed SH_BAR_EDGES count |
| TIME | E_MIS_RUN | 0..0 × 1 us | 1 | 0 | 32 | RAW | E | E5 mission timer images x12 (barcode phase; CAL_IR; sweep; warning; search; FR9; resume; run start / end; move; RUN_START; recovery): intervals exceed 65 ms (bits=31 for gen_layout.py) |

## Groups

| Group | Used bits | Storage bits |
|---|---|---|
| SB_LINE0 | 16 | 32 |
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
| EVT_JUNCTION | 2 | 11 |
| EVT_LINE_FOUND | 8 | 11 |
| EVT_BARCODE | 8 | 11 |
| EVT_UNMAPPED | 6 | 11 |
| EVT_INVALID | 6 | 11 |
| EVT_BARCODE_PROBE | 11 | 11 |
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
| K7_SNAP | 48 | 48 |
| MQ_HDR | 8 | 8 |
| MQ_STATE | 136 | 136 |
| MQ_EVT | 16 | 16 |
| MQ_PROF | 40 | 40 |
| MQ_SUMM | 48 | 48 |
| MQ_SNAP | 48 | 48 |
| MQ_HB | 40 | 40 |
| MQ_SENS | 88 | 88 |
| MQ_MAP | 8 | 8 |
| PRV_MOT_A | 30 | 32 |
| PRV_MOT_B | 24 | 32 |
| PRV_MOT_C | 32 | 32 |
| PRV_MOT_D | 18 | 32 |
| PRV_MOT_E | 16 | 16 |
| PRV_MOT_F | 20 | 32 |
| PRV_MOT_PI | 29 | 32 |
| PRV_WIN_L | 7 | 8 |
| PRV_WIN_R | 7 | 8 |
| PRV_WIN_T | 6 | 8 |
| PRV_WIN_I | 3 | 8 |
| PRV_IMU_A | 31 | 32 |
| PRV_IMU_B | 29 | 32 |
| PRV_IMU_C | 30 | 32 |
| PRV_IMU_D | 19 | 32 |
| PRV_IMU_R | 31 | 32 |
| PRV_IMU_E | 18 | 32 |
| PRV_IMU_F | 18 | 32 |
| PRV_IMU_G | 27 | 32 |
| PRV_IMU_H | 28 | 32 |
| PRV_IMU_J | 27 | 32 |
| PRV_IMU_K | 22 | 32 |
| PRV_IMU_L | 23 | 32 |
| PRV_IMU_M | 32 | 32 |
| PRV_HIST_A | 32 | 32 |
| PRV_HIST_B | 8 | 32 |
| PRV_OFS_A | 24 | 32 |
| PRV_OFS_B | 20 | 32 |
| PRV_TEL | 27 | 32 |
| PRV_TEL_K7 | 8 | 8 |
| PRV_BRIDGE | 21 | 32 |
| PRV_MQTT | 14 | 16 |
| PRV_CMD_T | 16 | 16 |
| PRV_CAL_L | 24 | 32 |
| PRV_CAL_R | 24 | 32 |
| PRV_LINE_A | 32 | 32 |
| PRV_LINE_D | 25 | 32 |
| PRV_LINE_B | 17 | 32 |
| PRV_LINE_C | 17 | 32 |
| PRV_LINE_E | 17 | 32 |
| PRV_LINE_F | 17 | 32 |
| PRV_BAR_A | 14 | 32 |
| PRV_BAR_B | 32 | 32 |
| PRV_BAR_C | 30 | 32 |
| PRV_BAR_LOG | 26 | 32 |
| PRV_BAR_W | 8 | 8 |
| PRV_BAR_MAP | 8 | 8 |
| SH_BAR | 8 | 8 |
| SH_BAR_OVF | 8 | 8 |
| SH_BAR_EDGES | 16 | 16 |
| SH_IR_RAW | 24 | 32 |
| SH_IMU_ACC | 30 | 32 |
| PRV_OBS_M | 32 | 32 |
| PRV_OBS_I | 32 | 32 |
| PRV_OBS_A | 30 | 32 |
| PRV_OBS_B | 29 | 32 |
| PRV_OBS_C | 27 | 32 |
| PRV_OBS_E | 32 | 32 |
| PRV_OBS_SCAN | 8 | 8 |
| PRV_OBS_SMP | 8 | 8 |
| SH_ECHO_CNT | 8 | 8 |
| SH_PING | 8 | 8 |
| SH_ECHO_WIN | 1 | 8 |
| PRV_ECHO_ISR | 8 | 8 |
| PRV_ECHO_ARM | 1 | 8 |
| PRV_MIS_G | 22 | 32 |
| PRV_HLT | 3 | 8 |
| SH_OBS_CHK | 1 | 8 |
| PRV_MIS_A | 32 | 32 |
| PRV_MIS_B | 32 | 32 |
| PRV_MIS_C | 32 | 32 |
| PRV_MIS_D | 32 | 32 |
| PRV_MIS_E | 32 | 32 |
| PRV_MIS_F | 32 | 32 |
| PRV_MIS_H | 32 | 32 |
| PRV_MIS_J | 32 | 32 |
| PRV_MIS_K | 32 | 32 |
| PRV_MIS_L | 18 | 32 |
| PRV_MIS_M | 17 | 32 |
| PRV_MIS_N | 17 | 32 |
| PRV_MIS_P | 17 | 32 |
| PRV_MIS_Q | 17 | 32 |
| PRV_MIS_R | 16 | 32 |

## Range assumptions

- **A23** One run is at most 32 m (APP_RUN_MAX_MM): encoder positions fit +/-131071 counts, path length 131071 counts, odometry 32767 mm.
- **A24** Pitch stays within +/-45.0 deg (+/-450 x 0.1 deg).
- **A25** A hump is at most 127 mm high.
- **A26** Turn rate stays within +/-360 deg/s.
- **A27** Obstacle width and clearances are reported up to 1020 mm (255 x 4 mm); the top code means clear / out of range.
- **A28** One run lasts at most 1023 s, about 17 min (APP_RUN_MAX_S).
