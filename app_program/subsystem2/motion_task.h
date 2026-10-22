/**
 * @file    motion_task.h
 * @brief   motion_task entry point, started by motion_start() in motion.c.
 * @owner   Buddy 2 (motion)
 * @kind    task
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern void motion_task(INT, void *);

#endif /* MOTION_TASK_H */

/*** end of file ***/
