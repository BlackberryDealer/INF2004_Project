"""
telem_fields.py - field table of every packed group.

GENERATED from tools/layout.csv by tools/gen_layout.py. Do not edit.

GROUPS[group] lists (field, bits, offset, lsb, unit) in CSV order, which
is also the order the firmware writes them (LSB first). Decoded value =
(raw - offset) * lsb, in unit.
"""

GROUPS = {
    'SB_LINE0': [
        ('STEERING', 11, 1024, 1, 'permille'),
        ('STATUS', 2, 0, 1, 'code'),
        ('JUNCTION', 2, 0, 1, 'code'),
        ('CALIBRATED', 1, 0, 1, 'flag'),
    ],
    'SB_MOT0': [
        ('SPEED_L', 8, 128, 1, 'count/50ms'),
        ('SPEED_R', 8, 128, 1, 'count/50ms'),
        ('STATE', 3, 0, 1, 'code'),
    ],
    'SB_MOT1': [
        ('POS_L', 18, 131072, 1, 'count'),
    ],
    'SB_MOT2': [
        ('POS_R', 18, 131072, 1, 'count'),
    ],
    'SB_MOT3': [
        ('PATH', 17, 0, 1, 'count'),
    ],
    'SB_IMU0': [
        ('PITCH', 10, 512, 1, '0.1 deg'),
        ('HUMP_NOW', 7, 0, 1, 'mm'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('MCLASS', 3, 0, 1, 'code'),
        ('TURN_SRC', 1, 0, 1, 'flag'),
    ],
    'SB_IMU1': [
        ('TURN_RATE', 10, 512, 1, 'deg/s'),
    ],
    'SB_OBS0': [
        ('DIST', 9, 0, 4, 'mm'),
        ('SCAN_STATE', 3, 0, 1, 'code'),
        ('ANGLE_STEP', 5, 0, 1, '5 deg'),
        ('PLAN_SIDE', 1, 0, 1, 'flag'),
        ('PROFILE_VALID', 1, 0, 1, 'flag'),
    ],
    'SB_OBS1': [
        ('WIDTH', 8, 0, 4, 'mm'),
        ('NEAREST', 8, 0, 4, 'mm'),
        ('CLEAR_L', 8, 0, 4, 'mm'),
        ('CLEAR_R', 8, 0, 4, 'mm'),
    ],
    'SB_OBS2': [
        ('LEG_LAT', 8, 0, 4, 'mm'),
        ('LEG_FWD', 8, 0, 4, 'mm'),
    ],
    'SB_MIS0': [
        ('SYS', 3, 0, 1, 'code'),
        ('RUN', 3, 0, 1, 'code'),
        ('AVOID', 3, 0, 1, 'code'),
        ('END_REASON', 2, 0, 1, 'code'),
        ('ARMED', 1, 0, 1, 'flag'),
        ('ARMED_CMD', 2, 0, 1, 'code'),
    ],
    'MSG_CMD': [
        ('PARAM', 11, 0, 1, 'mm|deg|mm/s'),
        ('TYPE', 3, 0, 1, 'code'),
        ('TAG', 2, 0, 1, 'count'),
    ],
    'MSG_EVT': [
        ('VALUE', 11, 0, 1, 'per type'),
        ('TYPE', 5, 0, 1, 'code'),
    ],
    'MSG_ECHO': [
        ('WIDTH', 12, 0, 4, 'us'),
        ('SEQ', 2, 0, 1, 'count'),
        ('VALID', 1, 0, 1, 'flag'),
        ('PAD', 1, 0, 1, '-'),
    ],
    'MSG_EDGE': [
        ('GAP', 15, 0, 16, 'us'),
        ('LEVEL', 1, 0, 1, 'flag'),
    ],
    'EVT_JUNCTION': [
        ('CLASS', 2, 0, 1, 'code'),
    ],
    'EVT_LINE_FOUND': [
        ('CROSS_MM', 8, 0, 1, 'mm'),
    ],
    'EVT_BARCODE': [
        ('CHAR', 6, 0, 1, 'code'),
        ('CMD', 2, 0, 1, 'code'),
    ],
    'EVT_UNMAPPED': [
        ('CHAR', 6, 0, 1, 'code'),
    ],
    'EVT_INVALID': [
        ('N_ELEM', 6, 0, 1, 'count'),
    ],
    'EVT_BARCODE_PROBE': [
        ('MM', 11, 0, 1, 'mm'),
    ],
    'EVT_PLAN': [
        ('SIDE', 1, 0, 1, 'flag'),
    ],
    'EVT_MOVE_DONE': [
        ('TAG', 2, 0, 1, 'count'),
    ],
    'EVT_STALL': [
        ('LEFT', 1, 0, 1, 'flag'),
        ('RIGHT', 1, 0, 1, 'flag'),
    ],
    'EVT_HUMP': [
        ('PEAK_MM', 7, 0, 1, 'mm'),
    ],
    'EVT_MCLASS': [
        ('CLASS', 3, 0, 1, 'code'),
    ],
    'EVT_CAL_DONE': [
        ('SOURCE', 1, 0, 1, 'code'),
    ],
    'EVT_SFAULT': [
        ('FAULT_IDX', 4, 0, 1, 'code'),
    ],
    'EVT_STATE': [
        ('SYS', 3, 0, 1, 'code'),
        ('RUN', 3, 0, 1, 'code'),
    ],
    'EVT_MAP': [
        ('CHAR', 6, 0, 1, 'code'),
        ('CMD', 2, 0, 1, 'code'),
    ],
    'K7_PROF': [
        ('ANGLE_STEP', 5, 0, 1, '5 deg'),
        ('WIDTH', 8, 0, 4, 'mm'),
        ('NEAREST', 8, 0, 4, 'mm'),
        ('CLEAR_L', 8, 0, 4, 'mm'),
        ('CLEAR_R', 8, 0, 4, 'mm'),
        ('SIDE', 1, 0, 1, 'flag'),
    ],
    'K7_SUMM': [
        ('END_REASON', 2, 0, 1, 'code'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('ODO_MM', 15, 0, 1, 'mm'),
        ('FAULTS', 10, 0, 1, 'mask'),
        ('RUN_S', 10, 0, 1, 's'),
    ],
    'K7_SNAP': [
        ('SYS', 3, 0, 1, 'code'),
        ('RUN', 3, 0, 1, 'code'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('ODO_MM', 15, 0, 1, 'mm'),
        ('FAULTS', 10, 0, 1, 'mask'),
        ('RUN_S', 10, 0, 1, 's'),
    ],
    'MQ_HDR': [
        ('VER', 3, 0, 1, 'code'),
        ('TYPE', 5, 0, 1, 'code'),
    ],
    'MQ_STATE': [
        ('SYS', 3, 0, 1, 'code'),
        ('RUN', 3, 0, 1, 'code'),
        ('MOT', 3, 0, 1, 'code'),
        ('SPEED_L', 8, 128, 1, 'count/50ms'),
        ('SPEED_R', 8, 128, 1, 'count/50ms'),
        ('POS_L', 18, 131072, 1, 'count'),
        ('POS_R', 18, 131072, 1, 'count'),
        ('LINE', 2, 0, 1, 'code'),
        ('STEER', 11, 1024, 1, 'permille'),
        ('ODO_MM', 15, 0, 1, 'mm'),
        ('PITCH', 10, 512, 1, '0.1 deg'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('TURN', 10, 512, 1, 'deg/s'),
        ('MCLASS', 3, 0, 1, 'code'),
        ('FAULTS', 10, 0, 1, 'mask'),
        ('PAD', 7, 0, 1, '-'),
    ],
    'MQ_EVT': [
        ('WORD', 16, 0, 1, 'word'),
    ],
    'MQ_PROF': [
        ('ANGLE_STEP', 5, 0, 1, '5 deg'),
        ('WIDTH', 8, 0, 4, 'mm'),
        ('NEAREST', 8, 0, 4, 'mm'),
        ('CLEAR_L', 8, 0, 4, 'mm'),
        ('CLEAR_R', 8, 0, 4, 'mm'),
        ('SIDE', 1, 0, 1, 'flag'),
        ('PAD', 2, 0, 1, '-'),
    ],
    'MQ_SUMM': [
        ('END_REASON', 2, 0, 1, 'code'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('ODO_MM', 15, 0, 1, 'mm'),
        ('FAULTS', 10, 0, 1, 'mask'),
        ('RUN_S', 10, 0, 1, 's'),
        ('PAD', 4, 0, 1, '-'),
    ],
    'MQ_SNAP': [
        ('SYS', 3, 0, 1, 'code'),
        ('RUN', 3, 0, 1, 'code'),
        ('HUMP_PEAK', 7, 0, 1, 'mm'),
        ('ODO_MM', 15, 0, 1, 'mm'),
        ('FAULTS', 10, 0, 1, 'mask'),
        ('RUN_S', 10, 0, 1, 's'),
    ],
    'MQ_HB': [
        ('UPTIME_S', 16, 0, 1, 's'),
        ('DROPS', 8, 0, 1, 'count'),
        ('LAT_MAX_US', 12, 0, 1, 'us'),
        ('PAD', 4, 0, 1, '-'),
    ],
    'MQ_SENS': [
        ('IR_L', 12, 0, 1, 'adc'),
        ('IR_R', 12, 0, 1, 'adc'),
        ('US_MM', 9, 0, 4, 'mm'),
        ('BAR_DO', 1, 0, 1, 'flag'),
        ('BAR_EDGES', 8, 0, 1, 'count'),
        ('AX', 10, 512, 4, 'mg'),
        ('AY', 10, 512, 4, 'mg'),
        ('AZ', 10, 512, 4, 'mg'),
        ('WDT_GAP', 8, 0, 250, 'us'),
        ('SCAN', 3, 0, 1, 'code'),
        ('PAD', 5, 0, 1, '-'),
    ],
    'MQ_MAP': [
        ('CHAR', 6, 0, 1, 'code'),
        ('CMD', 2, 0, 1, 'code'),
    ],
    'PRV_MOT_A': [
        ('STATE', 3, 0, 1, 'code'),
        ('ENC_OK', 1, 0, 1, 'flag'),
        ('ACT_TYPE', 3, 0, 1, 'code'),
        ('ACT_TAG', 2, 0, 1, 'count'),
        ('PEND_VALID', 1, 0, 1, 'flag'),
        ('PEND_TYPE', 3, 0, 1, 'code'),
        ('PEND_TAG', 2, 0, 1, 'count'),
        ('PEND_PARAM', 11, 0, 1, 'mm|deg|mm/s'),
        ('CREEP_STILL', 4, 0, 1, 'tick'),
    ],
    'PRV_MOT_B': [
        ('BASE_CPW', 7, 64, 1, 'count/50ms'),
        ('CREEP_CPW', 5, 16, 1, 'count/50ms'),
        ('STALL_L', 5, 0, 1, 'tick'),
        ('STALL_R', 5, 0, 1, 'tick'),
        ('LOG_PEND', 1, 0, 1, 'flag'),
        ('LOG_WHEEL', 1, 0, 1, 'code'),
    ],
    'PRV_MOT_C': [
        ('TARGET2', 14, 0, 1, 'half count'),
        ('PROG2', 14, 0, 1, 'half count'),
        ('STRK_L', 2, 0, 1, 'count'),
        ('STRK_R', 2, 0, 1, 'count'),
    ],
    'PRV_MOT_D': [
        ('PATH2', 18, 0, 1, 'half count'),
    ],
    'PRV_MOT_E': [
        ('DIFF_START', 16, 0, 1, 'count'),
    ],
    'PRV_MOT_F': [
        ('LOG_DT', 20, 0, 1, 'us'),
    ],
    'PRV_MOT_PI': [
        ('INTEG', 18, 131072, 1, 'permille/256'),
        ('DUTY', 11, 1024, 1, 'permille'),
    ],
    'PRV_WIN_L': [
        ('DELTA', 7, 64, 1, 'count'),
    ],
    'PRV_WIN_R': [
        ('DELTA', 7, 64, 1, 'count'),
    ],
    'PRV_WIN_T': [
        ('DT_MS', 6, 0, 1, 'ms'),
    ],
    'PRV_WIN_I': [
        ('IDX', 3, 0, 1, 'slot'),
    ],
    'PRV_IMU_A': [
        ('STATE', 3, 0, 1, 'code'),
        ('HYST', 4, 0, 1, 'sample'),
        ('I2C_ERR', 2, 0, 1, 'count'),
        ('GOOD_CNT', 2, 0, 1, 'count'),
        ('CLASS_GAP', 5, 0, 1, 'tick'),
        ('LAST_CLASS', 3, 0, 1, 'code'),
        ('MAG_DIV', 3, 0, 1, 'tick'),
        ('LIFT_CNT', 3, 0, 1, 'tick'),
        ('NEED_CFG', 1, 0, 1, 'flag'),
        ('IR_LIFT_CNT', 5, 0, 1, 'tick'),
    ],
    'PRV_IMU_B': [
        ('CAL_CNT', 8, 0, 1, 'sample'),
        ('CLIMB_CNT', 11, 0, 1, 'count'),
        ('HIST_IDX', 3, 0, 1, 'slot'),
        ('PEAK_NOW', 7, 0, 1, 'mm'),
        ('CAL_RETRIED', 1, 0, 1, 'flag'),
    ],
    'PRV_IMU_C': [
        ('PREV_HDG', 12, 0, 1, '0.1 deg'),
        ('PREV_HDG_VALID', 1, 0, 1, 'flag'),
        ('PITCH_MAX', 10, 512, 1, '0.1 deg'),
        ('RUN_PEAK', 7, 0, 1, 'mm'),
    ],
    'PRV_IMU_D': [
        ('PREV_DIFF', 19, 262144, 1, 'count'),
    ],
    'PRV_IMU_R': [
        ('STATE', 1, 0, 1, 'flag'),
        ('HYST', 3, 0, 1, 'sample'),
        ('FILT', 10, 512, 1, 'mg'),
        ('PEAK', 9, 0, 1, 'mg'),
        ('TICKS', 8, 0, 1, 'tick'),
    ],
    'PRV_IMU_E': [
        ('PREV_POS_L', 18, 131072, 1, 'count'),
    ],
    'PRV_IMU_F': [
        ('PREV_POS_R', 18, 131072, 1, 'count'),
    ],
    'PRV_IMU_G': [
        ('PITCH', 12, 2048, 1, '0.1 deg'),
        ('HUMP_NOW', 12, 0, 1, 'mm'),
        ('CAL_ACTIVE', 1, 0, 1, 'flag'),
        ('CAL_DONE', 1, 0, 1, 'flag'),
        ('TURN_ENC', 1, 0, 1, 'flag'),
    ],
    'PRV_IMU_H': [
        ('AX', 14, 8192, 1, 'mg'),
        ('AY', 14, 8192, 1, 'mg'),
    ],
    'PRV_IMU_J': [
        ('AZ', 14, 8192, 1, 'mg'),
        ('A_FWD', 13, 4096, 1, 'mg'),
    ],
    'PRV_IMU_K': [
        ('TURN_DPS', 22, 2097152, 1, 'deg/s'),
    ],
    'PRV_IMU_L': [
        ('ENC_HDG', 23, 4194304, 1, '0.01 deg'),
    ],
    'PRV_IMU_M': [
        ('MX', 16, 32768, 1, 'code'),
        ('MY', 16, 32768, 1, 'code'),
    ],
    'PRV_HIST_A': [
        ('H0', 8, 128, 1, 'count/50ms'),
        ('H1', 8, 128, 1, 'count/50ms'),
        ('H2', 8, 128, 1, 'count/50ms'),
        ('H3', 8, 128, 1, 'count/50ms'),
    ],
    'PRV_HIST_B': [
        ('H4', 8, 128, 1, 'count/50ms'),
    ],
    'PRV_OFS_A': [
        ('AX', 12, 2048, 2, 'mg'),
        ('AY', 12, 2048, 2, 'mg'),
    ],
    'PRV_OFS_B': [
        ('AZ', 12, 2048, 2, 'mg'),
        ('FIFO_OVR', 8, 0, 1, 'count'),
    ],
    'PRV_TEL': [
        ('DROPS', 8, 0, 1, 'count'),
        ('HB_DIV', 3, 0, 1, 'count'),
        ('UPTIME_HI', 16, 0, 1, 's'),
    ],
    'PRV_TEL_K7': [
        ('DROPS', 8, 0, 1, 'count'),
    ],
    'PRV_BRIDGE': [
        ('CMD_DROPS', 8, 0, 1, 'count'),
        ('PUB_DROPS', 8, 0, 1, 'count'),
        ('CMD_SEEN', 5, 0, 1, 'mask'),
    ],
    'PRV_MQTT': [
        ('BACKOFF', 2, 0, 1, 'exponent'),
        ('PEND_TRIES', 2, 0, 1, 'count'),
        ('PEND_LEN', 5, 0, 1, 'byte'),
        ('PEND_TYPE', 5, 0, 1, 'code'),
    ],
    'PRV_CMD_T': [
        ('LAST', 16, 0, 10, 'ms'),
    ],
    'PRV_CAL_L': [
        ('WHITE', 12, 0, 1, 'code'),
        ('BLACK', 12, 0, 1, 'code'),
    ],
    'PRV_CAL_R': [
        ('WHITE', 12, 0, 1, 'code'),
        ('BLACK', 12, 0, 1, 'code'),
    ],
    'PRV_LINE_A': [
        ('LOST_CNT', 5, 0, 1, 'tick'),
        ('LOST_SENT', 1, 0, 1, 'flag'),
        ('JCT_STATE', 2, 0, 1, 'code'),
        ('SAT_L', 1, 0, 1, 'flag'),
        ('SAT_R', 1, 0, 1, 'flag'),
        ('SEQ0', 8, 0, 1, 'count'),
        ('RUN_ACTIVE', 1, 0, 1, 'flag'),
        ('CAL_ACTIVE', 1, 0, 1, 'flag'),
        ('CALIBRATED', 1, 0, 1, 'flag'),
        ('REJOIN_WHITE', 1, 0, 1, 'flag'),
        ('CROSS_STRT', 1, 0, 1, 'flag'),
        ('BRANCH_SEEN', 1, 0, 1, 'flag'),
        ('LAST_JCT', 2, 0, 1, 'code'),
        ('RAW_LOG_CNT', 6, 0, 1, 'tick'),
    ],
    'PRV_LINE_D': [
        ('LAST_SEQ', 8, 0, 1, 'count'),
        ('BRANCH_PATH', 17, 0, 1, 'count'),
    ],
    'PRV_LINE_B': [
        ('JCT_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_LINE_C': [
        ('RUN_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_LINE_E': [
        ('WHITE_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_LINE_F': [
        ('CROSS_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_BAR_A': [
        ('STATE', 2, 0, 1, 'code'),
        ('N_ELEM', 6, 0, 1, 'count'),
        ('LAST_LEVEL', 1, 0, 1, 'flag'),
        ('CLAMP_AGE', 5, 0, 1, 'count'),
    ],
    'PRV_BAR_B': [
        ('MARK', 17, 0, 1, 'count'),
        ('CARRY', 15, 0, 16, 'us'),
    ],
    'PRV_BAR_C': [
        ('FIX_CNT', 2, 0, 1, 'count'),
        ('SETTLE', 8, 0, 16, 'us'),
        ('REREAD_FIRST', 1, 0, 1, 'flag'),
        ('PROBE_OK', 1, 0, 1, 'flag'),
        ('QUIET_KNOWN', 1, 0, 1, 'flag'),
        ('START_PATH', 17, 0, 1, 'count'),
    ],
    'PRV_BAR_LOG': [
        ('HEAD', 6, 0, 1, 'slot'),
        ('CNT', 6, 0, 1, 'count'),
        ('LEFT', 6, 0, 1, 'count'),
        ('CYCLE', 8, 0, 1, 'count'),
    ],
    'PRV_BAR_W': [
        ('WIDTH', 8, 0, 100, 'um'),
    ],
    'PRV_BAR_MAP': [
        ('CHAR', 6, 0, 1, 'code'),
        ('CMD', 2, 0, 1, 'code'),
    ],
    'SH_BAR': [
        ('BRANCH_SEQ', 8, 0, 1, 'count'),
    ],
    'SH_BAR_OVF': [
        ('OVERFLOW', 8, 0, 1, 'count'),
    ],
    'SH_BAR_EDGES': [
        ('COUNT', 16, 0, 1, 'count'),
    ],
    'SH_IR_RAW': [
        ('L', 12, 0, 1, 'adc'),
        ('R', 12, 0, 1, 'adc'),
    ],
    'SH_IMU_ACC': [
        ('AX', 10, 512, 4, 'mg'),
        ('AY', 10, 512, 4, 'mg'),
        ('AZ', 10, 512, 4, 'mg'),
    ],
    'PRV_OBS_M': [
        ('MEASURED', 25, 0, 1, 'mask'),
        ('STATE', 3, 0, 1, 'code'),
        ('SETTLE', 3, 0, 1, 'ping'),
        ('ON_RAMP', 1, 0, 1, 'flag'),
    ],
    'PRV_OBS_I': [
        ('INFERRED', 25, 0, 1, 'mask'),
        ('STEP', 5, 0, 1, '5 deg'),
        ('SAMPLE_CNT', 2, 0, 1, 'ping'),
    ],
    'PRV_OBS_A': [
        ('PREV', 9, 0, 4, 'mm'),
        ('ECHO', 9, 0, 4, 'mm'),
        ('WARMUP', 6, 0, 1, 'ping'),
        ('SILENT', 2, 0, 1, 'ping'),
        ('LOST', 2, 0, 1, 'ping'),
        ('FAR', 2, 0, 1, 'ping'),
    ],
    'PRV_OBS_B': [
        ('PASS_MARK', 17, 0, 1, 'count'),
        ('RAMP_HOLD', 4, 0, 1, 'ping'),
        ('MODE_SEEN', 3, 0, 1, 'mask'),
        ('DIST_LOG', 3, 0, 1, 'ping'),
        ('RAMP_LOGGED', 1, 0, 1, 'flag'),
        ('CREEP_LOGGED', 1, 0, 1, 'flag'),
    ],
    'PRV_OBS_C': [
        ('RAMP_MARK', 17, 0, 1, 'count'),
        ('EDGE_LO_D', 5, 0, 4, 'mm'),
        ('EDGE_HI_D', 5, 0, 4, 'mm'),
    ],
    'PRV_OBS_E': [
        ('EDGE_PHASE', 2, 0, 1, 'code'),
        ('EDGE_SUB', 3, 0, 1, 'deg'),
        ('EDGE_LO', 8, 0, 1, 'deg'),
        ('EDGE_HI', 8, 0, 1, 'deg'),
        ('EDGE_LIMIT', 8, 0, 4, 'mm'),
        ('EDGE_OK_LO', 1, 0, 1, 'flag'),
        ('EDGE_OK_HI', 1, 0, 1, 'flag'),
        ('EDGE_GRACE', 1, 0, 1, 'flag'),
    ],
    'PRV_OBS_SCAN': [
        ('MM', 8, 0, 4, 'mm'),
    ],
    'PRV_OBS_SMP': [
        ('MM', 8, 0, 4, 'mm'),
    ],
    'SH_ECHO_CNT': [
        ('COUNT', 8, 0, 1, 'count'),
    ],
    'SH_PING': [
        ('SEQ', 8, 0, 1, 'count'),
    ],
    'SH_ECHO_WIN': [
        ('OPEN', 1, 0, 1, 'flag'),
    ],
    'PRV_ECHO_ISR': [
        ('RISE_SEQ', 8, 0, 1, 'count'),
    ],
    'PRV_ECHO_ARM': [
        ('ARMED', 1, 0, 1, 'flag'),
    ],
    'PRV_MIS_G': [
        ('BTN_RUN', 2, 0, 1, 'tick'),
        ('BTN_RESET', 8, 0, 1, 'tick'),
        ('RESET_SENT', 1, 0, 1, 'flag'),
        ('WARNING', 1, 0, 1, 'flag'),
        ('LED', 3, 0, 1, 'code'),
        ('BUZZ', 1, 0, 1, 'flag'),
        ('PREV_SYS', 4, 0, 1, 'code'),
        ('BEEP', 2, 0, 1, 'code'),
    ],
    'PRV_HLT': [
        ('NORMAL_WIN', 1, 0, 1, 'flag'),
        ('WDT_ACTIVE', 1, 0, 1, 'flag'),
        ('STARTED', 1, 0, 1, 'flag'),
    ],
    'SH_OBS_CHK': [
        ('DONE', 1, 0, 1, 'flag'),
    ],
    'PRV_MIS_A': [
        ('DIFF_AT_STOP', 19, 262144, 1, 'count'),
        ('REJOIN_E', 11, 0, 1, 'mm'),
        ('END_REASON', 2, 0, 1, 'code'),
    ],
    'PRV_MIS_B': [
        ('CAL_DIFF0', 19, 262144, 1, 'count'),
        ('LAST_FAULTS', 10, 0, 1, 'mask'),
        ('SYS_STATE', 3, 0, 1, 'code'),
    ],
    'PRV_MIS_C': [
        ('RETRY_PARAM', 18, 131072, 1, 'mm|deg'),
        ('BAR_BACK_LEFT', 10, 0, 1, 'mm'),
        ('SYS_REQUEST', 4, 0, 1, 'code'),
    ],
    'PRV_MIS_D': [
        ('BAR_MARK', 17, 0, 1, 'count'),
        ('MOVE_LIMIT_MS', 15, 0, 1, 'ms'),
    ],
    'PRV_MIS_E': [
        ('BAR_PROBE_MARK', 17, 0, 1, 'count'),
        ('LEG_LAT', 10, 0, 1, 'mm'),
        ('RUN_STATE', 3, 0, 1, 'code'),
        ('ARMED_CMD', 2, 0, 1, 'code'),
    ],
    'PRV_MIS_F': [
        ('BAR_REARM_PATH', 17, 0, 1, 'count'),
        ('LEG_FWD', 10, 0, 1, 'mm'),
        ('AVOID_STEP', 3, 0, 1, 'code'),
        ('NO_PATH_CNT', 2, 0, 1, 'count'),
    ],
    'PRV_MIS_H': [
        ('BAR_FOLLOW', 17, 0, 1, 'count'),
        ('ARMED_CHAR', 6, 0, 1, 'code'),
        ('SUBSTEP', 3, 0, 1, 'step'),
        ('BAR_PHASE', 3, 0, 1, 'code'),
        ('RETRY_KIND', 3, 0, 1, 'code'),
    ],
    'PRV_MIS_J': [
        ('BAR_BACK_PATH', 17, 0, 1, 'count'),
        ('BAR_TRIES', 2, 0, 1, 'count'),
        ('SEARCH_FIRST', 2, 0, 1, 'code'),
        ('CAL_CORR_CNT', 2, 0, 1, 'count'),
        ('WAIT_TAG', 2, 0, 1, 'count'),
        ('ARMED', 1, 0, 1, 'flag'),
        ('PROBE_PENDING', 1, 0, 1, 'flag'),
        ('REARM_WAIT', 1, 0, 1, 'flag'),
        ('SIDE', 1, 0, 1, 'flag'),
        ('WARNING', 1, 0, 1, 'flag'),
        ('RESUME_PENDING', 1, 0, 1, 'flag'),
        ('INHIBIT_PEND', 1, 0, 1, 'flag'),
    ],
    'PRV_MIS_K': [
        ('BAR_EDGE_PATH', 17, 0, 1, 'count'),
        ('FR9_RUNNING', 1, 0, 1, 'flag'),
        ('WAIT_STILL', 1, 0, 1, 'flag'),
        ('CREEPING', 1, 0, 1, 'flag'),
        ('CREPT', 1, 0, 1, 'flag'),
        ('HAS_RUN', 1, 0, 1, 'flag'),
        ('POST_AVOID', 1, 0, 1, 'flag'),
        ('QUIET', 1, 0, 1, 'flag'),
        ('MOVE_PENDING', 1, 0, 1, 'flag'),
        ('RETRY_PENDING', 1, 0, 1, 'flag'),
        ('MOTION_FAILED', 1, 0, 1, 'flag'),
        ('IMU_VALID', 1, 0, 1, 'flag'),
        ('IR_VALID', 1, 0, 1, 'flag'),
        ('WDT_ENABLED', 1, 0, 1, 'flag'),
        ('RUN_STARTING', 1, 0, 1, 'flag'),
        ('ZERO_DONE', 1, 0, 1, 'flag'),
    ],
    'PRV_MIS_L': [
        ('PATH_AT_STOP', 17, 0, 1, 'count'),
        ('RECOVERING', 1, 0, 1, 'flag'),
    ],
    'PRV_MIS_M': [
        ('RUN_ODO_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_MIS_N': [
        ('RUN_ODO', 17, 0, 1, 'count'),
    ],
    'PRV_MIS_P': [
        ('REJOIN_START', 17, 0, 1, 'count'),
    ],
    'PRV_MIS_Q': [
        ('INHIBIT_MARK', 17, 0, 1, 'count'),
    ],
    'PRV_MIS_R': [
        ('BAR_EDGES', 16, 0, 1, 'count'),
    ],
}
