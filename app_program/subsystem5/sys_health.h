/**
 * @file    sys_health.h
 * @brief   FLG_ALIVE (K8) task check-ins and the hardware watchdog.
 * @owner   Buddy 5 (shared services)
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
#define SYS_HEALTH_WDT_MS        (30u)   /* hang -> reset <= 65 + 30 ms  */
#define SYS_HEALTH_OBS_MAX_AGE_MS (65u)   /* 60 ms period + 5 ms jitter   */
#define SYS_HEALTH_WDT_START_MS  (500u)  /* first check-ins must arrive  */

extern ER   sys_health_init(void);              /* creates K8 */
/* T1-T4, once per cycle, right after sys_tick_cycle_start() (before any
   wait inside the cycle). For OBSTACLE it also stores the check-in time
   in a 32-bit word written only by obstacle_task (one aligned store).  */
extern void sys_health_check_in(uint32_t task_bit);
/* mission_task, every pass (<= 10 ms). Feeds the watchdog only if LINE,
   MOTION and IMU have all checked in since the last feed AND OBSTACLE's
   own check-in time is at most SYS_HEALTH_OBS_MAX_AGE_MS (65 ms) old.  */
extern void sys_health_service(void);
/* mission, in BOOT: starts the STARTUP window (SYS_HEALTH_WDT_START_MS),
   switches to the normal 30 ms window by itself once every task has
   checked in for the first time.                                       */
extern void sys_health_enable_watchdog(void);
extern bool sys_health_is_started(void);  /* all four first check-ins */

#endif /* SYS_HEALTH_H */

/*** end of file ***/
