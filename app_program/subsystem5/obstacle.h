/**
 * @file    obstacle.h
 * @brief   Obstacle subsystem entry points (obstacle_task, K1 SEM_ECHO).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include "../app.h"

extern ER obstacle_init(void);    /* creates K1 SEM_ECHO */
extern ER obstacle_start(void);   /* attaches the echo ISR, starts the task */

#endif /* OBSTACLE_H */

/*** end of file ***/
