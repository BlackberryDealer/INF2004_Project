/**
 * @file    sys_tick.h
 * @brief   R2 10 ms and R3 60 ms releases, latency and overrun counters.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef SYS_TICK_H
#define SYS_TICK_H

#include <stdint.h>
#include "../app.h"

#define SYS_TICK_SLOT_LINE      (0u)   /* 10 ms */
#define SYS_TICK_SLOT_MOTION    (1u)   /* 10 ms */
#define SYS_TICK_SLOT_IMU       (2u)   /* 10 ms */
#define SYS_TICK_SLOT_OBSTACLE  (3u)   /* 60 ms */
#define SYS_TICK_SLOT_COUNT     (4u)

extern ER       sys_tick_init(void);   /* creates R2, R3 (stopped) */
extern ER       sys_tick_start(void);  /* starts R2, R3 */

/* Called once by each periodic task before its loop. */
extern ER       sys_tick_register(uint32_t slot, ID task_id);
extern void     sys_tick_register_feeder(ID task_id); /* mission_task */

/* Called by the task right after tk_slp_tsk() returns. Records release
   latency (E7) and counts queued wake-ups as overruns.              */
extern void     sys_tick_cycle_start(uint32_t slot);

extern uint32_t sys_tick_now_us(void);           /* 1 MHz, wraps 71 min */
extern uint32_t sys_tick_latency_max_us(void);   /* all slots, <= 4095 */
extern uint32_t sys_tick_overruns(uint32_t slot);/* <= 255 */

#endif /* SYS_TICK_H */

/*** end of file ***/
