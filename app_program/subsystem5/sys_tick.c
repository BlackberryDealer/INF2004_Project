/**
 * @file    sys_tick.c
 * @brief   Phase 0 STUB of the time base: no cyclic handlers yet, time reads 0.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 * @reqs    NFR1, NFR17
 */

#include "sys_tick.h"          /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * sys_tick_init
 * @brief   STUB: will create R2 (10 ms) and R3 (60 ms), stopped.
 * @return  E_OK
 * @context usermain
 */
ER sys_tick_init(void)
{
    return E_OK;
}

/**
 * sys_tick_start
 * @brief   STUB: will start R2 and R3.
 * @return  E_OK
 * @context usermain
 */
ER sys_tick_start(void)
{
    return E_OK;
}

/**
 * sys_tick_register
 * @brief   STUB: register a periodic task for its slot.
 * @param   slot     SYS_TICK_SLOT_*
 * @param   task_id  task ID
 * @return  E_OK
 * @context periodic task, before its loop
 */
ER sys_tick_register(uint32_t slot, ID task_id)
{
    (void) slot;   /* stub: unused until the owner's version */
    (void) task_id;

    return E_OK;
}

/**
 * sys_tick_cycle_start
 * @brief   STUB: record release latency and overruns.
 * @param   slot  SYS_TICK_SLOT_*
 * @return  none
 * @context periodic task
 */
void sys_tick_cycle_start(uint32_t slot)
{
    (void) slot;   /* stub: unused until the owner's version */
}

/**
 * sys_tick_now_us
 * @brief   STUB: 1 MHz time.
 * @return  0 (stub)
 * @context any
 */
uint32_t sys_tick_now_us(void)
{
    return 0u;
}

/**
 * sys_tick_latency_max_us
 * @brief   STUB: worst release latency.
 * @return  0 (stub)
 * @context any task
 */
uint32_t sys_tick_latency_max_us(void)
{
    return 0u;
}

/**
 * sys_tick_overruns
 * @brief   STUB: overrun count of a slot.
 * @param   slot  SYS_TICK_SLOT_*
 * @return  0 (stub)
 * @context any task
 */
uint32_t sys_tick_overruns(uint32_t slot)
{
    (void) slot;   /* stub: unused until the owner's version */

    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
