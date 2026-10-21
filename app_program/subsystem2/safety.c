/**
 * @file    safety.c
 * @brief   Phase 0 STUB of FLG_SAFETY (K3): no flag yet, never active. Replaced
 *          by Buddy 2 in B2 Phase 0.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR13, NFR10a
 */

#include "safety.h"            /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

/* ---- private data ---------------------------------------------------- */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * safety_init
 * @brief   STUB: will create K3 FLG_SAFETY.
 * @return  E_OK
 * @context usermain
 */
ER safety_init(void)
{
    return E_OK;
}

/**
 * safety_set
 * @brief   STUB: set safety bits.
 * @param   bits  SAFETY_* bits to set
 * @return  none
 * @context any task
 */
void safety_set(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * safety_set_from_isr
 * @brief   STUB: set safety bits from an R1 callback.
 * @param   bits  SAFETY_* bits to set
 * @return  none
 * @context R1 callbacks
 */
void safety_set_from_isr(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * safety_clear
 * @brief   STUB: clear safety bits.
 * @param   bits  SAFETY_* bits to clear
 * @return  none
 * @context mission_task
 */
void safety_clear(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * safety_is_active
 * @brief   STUB: is any safety bit set?
 * @return  false (stub)
 * @context any task
 */
bool safety_is_active(void)
{
    return false;
}

/**
 * safety_get
 * @brief   STUB: current safety bits.
 * @return  0 (stub)
 * @context any task
 */
uint32_t safety_get(void)
{
    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
