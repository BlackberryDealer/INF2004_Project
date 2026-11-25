/**
 * @file    cfg_imu.h
 * @brief   Every IMU and terrain tunable and measured constant (B4.1.3).
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    cfg
 *
 * SPEC-QUESTION SQ-B4-2: B4.1.3 describes the LSM303DLHC. The GY-511 the
 * team holds carries an LSM303D (single address 0x1D, WHO_AM_I 0x49), so
 * IMU_MG_PER_LSB no longer means what B4.1.3 says: the DLHC gave 12-bit
 * samples at 2 mg/LSB after /16, the LSM303D gives 16-bit samples at
 * 0.122 mg/LSB for +-4 g, which no integer constant can express, so imu.c
 * converts with the fraction IMU_MG_NUM/IMU_MG_DEN (61/500) and this
 * constant is kept only so the B4.1.3 list stays complete.
 *
 * Change Requirement 2 (lecturer, 11 Nov 2026): IMU collision detection
 * (INT1 impact interrupt, IMU_IMPACT_THS, IMPACT class) is removed. The
 * only replacement is the lift-stop: while the motors are enabled, a car
 * that is lifted or overturned sets SAFETY_LIFT and the motors stop. It
 * is two plain threshold checks on the calibrated averages imu_task
 * already computes every cycle (no classification):
 *   - overturned / steeply tilted: az below IMU_LIFT_TILT_AZ_MG
 *     (cos 60 deg = 0.5 g; course ramps and humps stay far below 45 deg,
 *     A24);
 *   - lifted: the size of the acceleration vector differs from 1 g by
 *     more than IMU_LIFT_DG_MG, i.e. the push a hand lift gives. The size
 *     does not change with the slope, so a ramp cannot trip it; it is
 *     compared squared, so no square root is needed.
 * Either one must hold for IMU_LIFT_CONFIRM_CYCLES consecutive 10 ms
 * cycles, so a single bump on a hump cannot stop the run. Both thresholds
 * are TUNE values for S4-I12 (lift) and S4-I13 (hump run, no trip).
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
#define IMU_CAL_SAMPLES          (200u) /* 2 s */
/* Uncorrected gravity magnitude while braked. The mounted car measures
   about 870 mg, with negative Z. Allow margin in either orientation. */
#define IMU_CAL_GRAVITY_MIN_MG   (650)
#define IMU_CAL_GRAVITY_MAX_MG   (1350)
#define IMU_MG_PER_LSB           (2)    /* DLHC only, see SQ-B4-2 */
#define IMU_MAG_EVERY_N          (5u)   /* magnetometer every 50 ms */
#define IMU_I2C_ERR_MAX          (3u)
#define IMU_INIT_TRIES           (5u)   /* power-on: the LSM303D may NACK */
#define IMU_INIT_RETRY_MS        (20u)  /* between start-up tries        */
#define IMU_CLASS_MIN_GAP_TICKS  (20u) /* 200 ms between events  */
/* Lift-stop (Change Requirement 2, see the header). LIFT_CNT in
   PRV_IMU_A is 3 bits, so IMU_LIFT_CONFIRM_CYCLES must stay <= 7.    */
#define IMU_REST_AZ_MG           (1000) /* at rest az reads +1000 mg */
#define IMU_LIFT_TILT_AZ_MG      (500)  /* TUNE: az below = tilted > 60 deg */
#define IMU_LIFT_DG_MG           (300)  /* TUNE: ||a| - 1 g| above = lifted */
#define IMU_LIFT_CONFIRM_CYCLES  (5u)   /* 50 ms in a row before the stop */
/* Level lift (car run 2026-11-18 13:19, S4-U17): a smooth level lift
   keeps |a| at 1 g, so the IMU check never fires. With no floor under
   them both IR sensors read 3890..4067 raw; the darkest tape in every
   run that day read <= 3570, the dark ramp up to 3905 (13:28). Both
   above IMU_LIFT_IR_RAW_MIN (3950, lifted reads 3980..4067) for
   IMU_LIFT_IR_CYCLES cycles in a row (300 ms, longer than a crest or a
   cross at cruise speed) also sets SAFETY_LIFT. IR_LIFT_CNT in PRV_IMU_A
   is 5 bits, so IMU_LIFT_IR_CYCLES must stay <= 31.                  */
#define IMU_LIFT_IR_RAW_MIN      (3950u) /* TUNE: raw ADC, no floor above */
#define IMU_LIFT_IR_CYCLES       (30u)   /* 300 ms in a row               */
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
/* One-wheel hump (track 2026-11-18, S4-U18): the demo hump is a thin
   strip only the right wheel climbs, so the car tilts sideways about the
   line from the left wheel to the front contact, not nose up. For a
   wheel raised h the sideways gravity component is ay = g x h / track
   (the pitch part, g x h / wheelbase, is mixed with braking), so
   h = track x |ay| / 1000 mg. ay is smoothed (1/4 per cycle); a hump
   starts after TERRAIN_ROLL_HYST cycles above TERRAIN_ROLL_START_MG
   while driving straight, ends after TERRAIN_ROLL_HYST cycles below
   TERRAIN_ROLL_LEVEL_MG, and more than TERRAIN_ROLL_MAX_TICKS on it is a
   ramp or a held tilt, not a hump. GAIN_PCT corrects the sensor scale
   against a ruler. */
/* 45 -> 30 (professor 24 Nov 2026: the hump is now 4..5 mm, S4-U19).
   On the car 1 mm of rise reads ~12 mg (gain 67 %), so 4 mm is ~48 mg
   and the smoothed value never held 45 mg on a thin strip. 30 mg is
   ~2.5 mm: below the smallest demo hump, above the +-30 mg vibration
   case of S4-U18 (its smoothed value stays under 10 mg).            */
#define TERRAIN_ROLL_START_MG    (30)   /* TUNE: ~2.5 mm on the car    */
#define TERRAIN_ROLL_LEVEL_MG    (20)   /* TUNE                        */
#define TERRAIN_ROLL_HYST        (3u)   /* cycles, <= 7 (3-bit HYST)   */
#define TERRAIN_ROLL_STRAIGHT_COUNTS (3) /* |dl - dr| per 10 ms, ~10 mg */
#define TERRAIN_ROLL_MAX_TICKS   (200u) /* TUNE: 2 s, ramp rule, <= 255 */
#define TERRAIN_ROLL_GAIN_PCT    (67u)  /* car 18 Nov: 15 mm vs ruler 10 */
#define TERRAIN_REPORT_MODE      (0u)  /* 0 integ, 1 geom, 2 mean */
/* Forward acceleration gain: a change of 1 count per 50 ms window over
   50 ms is 14.2 mg. a = dcpw x um-per-count x 1000 / window^2 mm/s^2,
   / 9.807 for mg, x 1024 for Q10. Placeholder values give 14576.    */
#define TERRAIN_AFWD_MG_PER_CPW_Q10                                            \
    ((APP_UM_PER_COUNT * 1024000u) /                                           \
     ((APP_SPEED_WINDOW_MS * APP_SPEED_WINDOW_MS * 9807u) / 1000u))

#endif /* CFG_IMU_H */

/*** end of file ***/
