/**
 * @file    safety.h
 * @brief   FLG_SAFETY (K3). Motion brakes while any bit is set.
 * @owner   Buddy 2 (motion)
 * @kind    task
 */

#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

#define SAFETY_ESTOP    (0x01u)   /* set by obstacle_task            */
#define SAFETY_IMPACT   (0x02u)   /* set from R1 (IMU INT1 callback) */
#define SAFETY_FAULT    (0x04u)   /* set by mission_task in FAULT    */
#define SAFETY_ALL      (0x07u)

extern ER       safety_init(void);
extern void     safety_set(uint32_t bits);           /* any task          */
extern void     safety_set_from_isr(uint32_t bits);  /* R1 callbacks only */
extern void     safety_clear(uint32_t bits);         /* mission_task only */
extern bool     safety_is_active(void);              /* any bit set?      */
extern uint32_t safety_get(void);

#endif /* SAFETY_H */

/*** end of file ***/
