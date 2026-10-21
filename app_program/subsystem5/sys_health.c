/**
 * @file    sys_health.c
 * @brief   Phase 0 STUB of the alive flags and watchdog: the watchdog is never
 *          enabled.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 * @reqs    NFR10b
 */

#include "sys_health.h"        /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * sys_health_init
 * @brief   STUB: will create K8 FLG_ALIVE.
 * @return  E_OK
 * @context usermain
 */
ER sys_health_init(void)
{
    return E_OK;
}

/**
 * sys_health_check_in
 * @brief   STUB: a periodic task reports it is alive.
 * @param   task_bit  SYS_HEALTH_BIT_*
 * @return  none
 * @context line, motion, imu, obstacle tasks
 */
void sys_health_check_in(uint32_t task_bit)
{
    (void) task_bit;   /* stub: unused until the owner's version */
}

/**
 * sys_health_service
 * @brief   STUB: feed the watchdog if every task checked in.
 * @return  none
 * @context mission_task
 */
void sys_health_service(void)
{
}

/**
 * sys_health_enable_watchdog
 * @brief   STUB: start the startup watchdog window.
 * @return  none
 * @context mission_task
 */
void sys_health_enable_watchdog(void)
{
}

/**
 * sys_health_is_started
 * @brief   STUB: have all four tasks checked in once?
 * @return  false (stub)
 * @context mission_task
 */
bool sys_health_is_started(void)
{
    return false;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
