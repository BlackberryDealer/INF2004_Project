/**
 * @file    cfg_imu.h
 * @brief   Every IMU and terrain tunable and measured constant (B4.1.3).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    cfg
 *
 * SPEC-QUESTION SQ-B4-2: B4.1.3 describes the LSM303DLHC. The GY-511 the
 * team holds carries an LSM303D (single address 0x1D, WHO_AM_I 0x49), so
 * two values below no longer mean what B4.1.3 says:
 *   - IMU_MG_PER_LSB: the DLHC gave 12-bit samples at 2 mg/LSB after /16.
 *     The LSM303D gives 16-bit samples at 0.122 mg/LSB for +-4 g, which
 *     no integer constant can express, so imu.c converts with the
 *     fraction IMU_MG_NUM/IMU_MG_DEN (61/500) and this constant is kept
 *     only so the B4.1.3 list stays complete.
 *   - IMU_IMPACT_THS: the per-LSB weight of IG_THS1 is not published for
 *     this part in the register chapter, so the 1.5 g intent in the
 *     comment is nominal only. S4-I03 measures the real threshold and
 *     the value is tuned there, exactly like the TUNE marker says.
 * Pending team decision before merge.
 */

#ifndef CFG_IMU_H
#define CFG_IMU_H

#include "../app_types.h"

#define IMU_X_FORWARD_SIGN       (1)  /* MEASURE: S4-I08 */
#define IMU_MAG_HEADING_SIGN     (1)  /* MEASURE: + = right turn */
#define IMU_MAG_OFS_X            (0)  /* MEASURE: S4-I10 hard-iron X  */
#define IMU_MAG_OFS_Y            (0)  /* MEASURE: S4-I10 hard-iron Y  */
#define IMU_MAG_MAX_PITCH_DDEG   (50) /* above 5.0 deg use encoder rate */
#define IMU_ACC_SAMPLES_MAX      (4u)
#define IMU_IMPACT_THS           (47u)  /* TUNE: nominal 1.5 g (SQ-B4-2) */
#define IMU_CAL_SAMPLES          (200u) /* 2 s */
#define IMU_MG_PER_LSB           (2)    /* DLHC only, see SQ-B4-2 */
#define IMU_MAG_EVERY_N          (5u)   /* magnetometer every 50 ms */
#define IMU_I2C_ERR_MAX          (3u)
#define IMU_CLASS_MIN_GAP_TICKS  (20u) /* 200 ms between events  */
#define IMU_IMPACT_HOLD_TICKS    (20u) /* class IMPACT shown for 200 ms */
#define IMU_TURNING_DPS          (30)  /* TUNE */
#define IMU_ACCEL_MG             (50)  /* TUNE */
#define IMU_TURN_DISAGREE_PCT    (10)  /* A7 */
#define IMU_TURN_CHECK_MIN_DPS   (20)
#define IMU_LOG_HUMPS            (0)  /* 1 = log both estimates */
#define TERRAIN_CLIMB_START_DDEG (30) /* TUNE: 3.0 deg */
#define TERRAIN_LEVEL_DDEG       (15) /* TUNE */
#define TERRAIN_HYST_SAMPLES     (5u)
#define TERRAIN_MAX_CLIMB_MM     (400u) /* TUNE: ramp rule        */
#define TERRAIN_HUMP_MIN_MM      (5u)
#define TERRAIN_WHEELBASE_MM     (100) /* MEASURE: axle to front contact */
#define TERRAIN_REPORT_MODE      (0u)  /* 0 integ, 1 geom, 2 mean */
/* Forward acceleration gain: a change of 1 count per 50 ms window over
   50 ms is 14.2 mg. a = dcpw x um-per-count x 1000 / window^2 mm/s^2,
   / 9.807 for mg, x 1024 for Q10. Placeholder values give 14576.    */
#define TERRAIN_AFWD_MG_PER_CPW_Q10                                            \
    ((APP_UM_PER_COUNT * 1024000u) /                                           \
     ((APP_SPEED_WINDOW_MS * APP_SPEED_WINDOW_MS * 9807u) / 1000u))

#endif /* CFG_IMU_H */

/*** end of file ***/
