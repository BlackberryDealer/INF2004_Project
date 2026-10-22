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
        ('JUNCTION', 3, 0, 1, 'code'),
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
        ('CLASS', 3, 0, 1, 'code'),
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
    'MQ_HB': [
        ('UPTIME_S', 16, 0, 1, 's'),
        ('DROPS', 8, 0, 1, 'count'),
        ('LAT_MAX_US', 12, 0, 1, 'us'),
        ('PAD', 4, 0, 1, '-'),
    ],
    'MQ_MAP': [
        ('CHAR', 6, 0, 1, 'code'),
        ('CMD', 2, 0, 1, 'code'),
    ],
    'PRV_IMU_A': [
        ('STATE', 3, 0, 1, 'code'),
        ('HYST', 4, 0, 1, 'sample'),
        ('I2C_ERR', 2, 0, 1, 'count'),
        ('GOOD_CNT', 2, 0, 1, 'count'),
        ('CLASS_GAP', 5, 0, 1, 'tick'),
        ('LAST_CLASS', 3, 0, 1, 'code'),
        ('MAG_DIV', 3, 0, 1, 'tick'),
        ('IMPACT_HOLD', 5, 0, 1, 'tick'),
        ('IMPACT_SEQ', 4, 0, 1, 'count'),
    ],
    'PRV_IMU_B': [
        ('CAL_CNT', 8, 0, 1, 'sample'),
        ('CLIMB_CNT', 11, 0, 1, 'count'),
        ('HIST_IDX', 3, 0, 1, 'slot'),
        ('PEAK_NOW', 7, 0, 1, 'mm'),
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
    'PRV_IMU_E': [
        ('PREV_POS_L', 18, 131072, 1, 'count'),
    ],
    'PRV_IMU_F': [
        ('PREV_POS_R', 18, 131072, 1, 'count'),
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
    'SH_IMU': [
        ('IMPACT_SEQ', 8, 0, 1, 'count'),
    ],
}
