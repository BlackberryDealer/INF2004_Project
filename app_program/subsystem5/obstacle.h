/**
 * @file    obstacle.h
 * @brief   Obstacle task entry points (D.15).
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   obstacle_init(void);
extern ER   obstacle_start(void);

#endif /* OBSTACLE_H */

/*** end of file ***/
