/**
 * @file    imu.h
 * @brief   IMU task entry points (D.15).
 * @owner   Buddy 4 (IMU and terrain)
 * @kind    task
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER imu_init(void);
extern ER imu_start(void);
/* Calibrated accelerometer averages as one SH_IMU_ACC word (AX, AY, AZ
   in 4 mg steps, offset 512). One aligned load, so atomic.            */
extern uint32_t imu_acc_word(void);

#endif /* IMU_H */

/*** end of file ***/
