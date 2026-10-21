/**
 * @file    sys_health.h
 * @brief   K8 FLG_ALIVE check-ins and the 100 ms hardware watchdog.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

#include <stdint.h>
#include "../app.h"

#define SYS_HEALTH_BIT_LINE      (0x01u)
#define SYS_HEALTH_BIT_MOTION    (0x02u)
#define SYS_HEALTH_BIT_IMU       (0x04u)
#define SYS_HEALTH_BIT_OBSTACLE  (0x08u)
#define SYS_HEALTH_BIT_ALL       (0x0Fu)

extern ER   sys_health_init(void);              /* creates K8 */
extern void sys_health_check_in(uint32_t task_bit);   /* T1-T4, each cycle */
extern void sys_health_service(void);  /* mission_task, every pass: feeds
                                          the watchdog if all bits set   */
extern void sys_health_enable_watchdog(void);   /* mission, after BOOT */

#endif /* SYS_HEALTH_H */

/*** end of file ***/
