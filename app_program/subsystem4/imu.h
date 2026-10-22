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

#endif /* IMU_H */

/*** end of file ***/
