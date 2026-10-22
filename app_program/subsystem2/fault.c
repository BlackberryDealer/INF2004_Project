/**
 * @file    fault.c
 * @brief   FLG_FAULT (K12): the shared fault bitmask, one bit per
 *          APP_FAULT_* condition (10 bits).
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    Minimum-bit requirement item 5 (faults are bits of one shared
 *          bitmask), FR17 (mission stops the robot on a stop-class fault)
 *
 * Any task (and the radio task, through mqtt_bridge) sets bits; mission
 * clears them on RESET and the bridge clears WIFI_LOSS on reconnect. A
 * condition that persists is set again every cycle by its owner, so a RESET
 * cannot hide it (Part D.13). The kernel event flag makes every operation
 * atomic. No ISR variant: faults are always raised from task context.
 */

#include "fault.h"             /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"

/* ---- private data ---------------------------------------------------- */

static ID gh_fault_flg = 0;    /* K12; 0 until fault_init() succeeds */

/* ---- public functions ------------------------------------------------- */

/**
 * fault_init
 * @brief   Creates K12 FLG_FAULT with every bit clear.
 * @return  E_OK, or the kernel error from tk_cre_flg()
 * @context usermain (before any task starts)
 */
ER fault_init(void)
{
    T_CFLG cflg = {0};
    ER     ercd = E_OK;
    ID     flg  = 0;

    cflg.flgatr  = TA_TFIFO | TA_WMUL;   /* several tasks may read     */
    cflg.iflgptn = 0u;                   /* no fault at power-on       */

    flg = tk_cre_flg(&cflg);
    if (E_OK < flg)
    {
        gh_fault_flg = flg;
    }
    else
    {
        ercd = flg;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * fault_set
 * @brief   Sets fault bits.
 * @param   bits  APP_FAULT_* bits to set (others ignored)
 * @return  none
 * @context any task or the radio task; never an ISR
 */
void fault_set(uint32_t bits)
{
    /* Set cannot fail with a valid ID. Cast: APP_FAULT_ALL fits UINT. */
    (void) tk_set_flg(gh_fault_flg, (UINT) (bits & APP_FAULT_ALL));
}

/**
 * fault_clear
 * @brief   Clears fault bits.
 * @param   bits  APP_FAULT_* bits to clear (others ignored)
 * @return  none
 * @context mission_task on RESET; mqtt_bridge for WIFI_LOSS
 */
void fault_clear(uint32_t bits)
{
    /* tk_clr_flg ANDs: pass the inverted mask. Cannot fail with a valid
       ID. Cast: a 32-bit mask fits UINT.                               */
    (void) tk_clr_flg(gh_fault_flg, (UINT) ~(bits & APP_FAULT_ALL));
}

/**
 * fault_get
 * @brief   Current fault mask.
 * @return  APP_FAULT_* bits (10 bits); 0 if the flag cannot be read
 * @context any task
 */
uint32_t fault_get(void)
{
    T_RFLG   rflg = {0};
    uint32_t bits = 0u;

    if (E_OK == tk_ref_flg(gh_fault_flg, &rflg))
    {
        bits = (uint32_t) rflg.flgptn & APP_FAULT_ALL; /* cast: UINT 32 b */
    }

    return bits;
}

/*** end of file ***/
