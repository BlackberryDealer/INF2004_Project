/**
 * @file    fault.c
 * @brief   Phase 0 STUB of FLG_FAULT (K12): no flag yet, no faults. Replaced by
 *          Buddy 2 in B2 Phase 0.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    Minimum-bit item 5 (shared fault bitmask)
 */

#include "fault.h"             /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * fault_init
 * @brief   STUB: will create K12 FLG_FAULT.
 * @return  E_OK
 * @context usermain
 */
ER fault_init(void)
{
    return E_OK;
}

/**
 * fault_set
 * @brief   STUB: set fault bits.
 * @param   bits  APP_FAULT_* bits to set
 * @return  none
 * @context any task or the radio task; never an ISR
 */
void fault_set(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * fault_clear
 * @brief   STUB: clear fault bits.
 * @param   bits  APP_FAULT_* bits to clear
 * @return  none
 * @context mission RESET; bridge WIFI
 */
void fault_clear(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * fault_get
 * @brief   STUB: current fault mask.
 * @return  0 (stub)
 * @context any task
 */
uint32_t fault_get(void)
{
    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
