/**
 * @file    safety.c
 * @brief   FLG_SAFETY (K3): the safety interlock. While any bit is set,
 *          motion_task brakes at its next cycle (<= 10 ms + 1 ms).
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR13 (brake on impact), NFR10a (flag -> motors low <= 15 ms)
 *
 * Writers: obstacle_task (ESTOP), the IMU INT1 callback in R1 (IMPACT),
 * mission_task (FAULT). Only mission_task clears bits. Readers: any task.
 * The kernel event flag makes every set/clear/read atomic, so the bits need
 * no extra locking and can be set from interrupt context.
 */

#include "safety.h"            /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private data ---------------------------------------------------- */

static ID gh_safety_flg = 0;   /* K3; 0 until safety_init() succeeds */

/* ---- public functions ------------------------------------------------- */

/**
 * safety_init
 * @brief   Creates K3 FLG_SAFETY with every bit clear.
 * @return  E_OK, or the kernel error from tk_cre_flg()
 * @context usermain (before any task starts)
 */
ER safety_init(void)
{
    T_CFLG cflg = {0};
    ER     ercd = E_OK;
    ID     flg  = 0;

    cflg.flgatr  = TA_TFIFO | TA_WMUL;   /* several tasks may wait/read */
    cflg.iflgptn = 0u;                   /* no safety bit at power-on  */

    flg = tk_cre_flg(&cflg);
    if (E_OK < flg)
    {
        gh_safety_flg = flg;
    }
    else
    {
        ercd = flg;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * safety_set
 * @brief   Sets safety bits; motion brakes at its next cycle.
 * @param   bits  SAFETY_* bits to set (others ignored)
 * @return  none
 * @context any task
 */
void safety_set(uint32_t bits)
{
    /* Set cannot fail with a valid ID. Cast: SAFETY_ALL fits UINT. */
    (void) tk_set_flg(gh_safety_flg, (UINT) (bits & SAFETY_ALL));
}

/**
 * safety_set_from_isr
 * @brief   Sets safety bits from interrupt context (R1 callbacks).
 *          tk_set_flg() is one of the calls allowed in a handler (C.2).
 * @param   bits  SAFETY_* bits to set (others ignored)
 * @return  none
 * @context R1 callbacks only (interrupt context)
 */
void safety_set_from_isr(uint32_t bits)
{
    /* Set cannot fail with a valid ID; ISR-callable. Cast: fits UINT. */
    (void) tk_set_flg(gh_safety_flg, (UINT) (bits & SAFETY_ALL));
}

/**
 * safety_clear
 * @brief   Clears safety bits.
 * @param   bits  SAFETY_* bits to clear (others ignored)
 * @return  none
 * @context mission_task only
 */
void safety_clear(uint32_t bits)
{
    /* tk_clr_flg ANDs: pass the inverted mask. Cannot fail with a valid
       ID. Cast: a 32-bit mask fits UINT.                               */
    (void) tk_clr_flg(gh_safety_flg, (UINT) ~(bits & SAFETY_ALL));
}

/**
 * safety_is_active
 * @brief   Is any safety bit set?
 * @return  true while motion must stay braked
 * @context any task
 */
bool safety_is_active(void)
{
    return (0u != safety_get());
}

/**
 * safety_get
 * @brief   Current safety bits.
 * @return  SAFETY_* bits; 0 if the flag cannot be read
 * @context any task
 */
uint32_t safety_get(void)
{
    T_RFLG   rflg = {0};
    uint32_t bits = 0u;

    if (E_OK == tk_ref_flg(gh_safety_flg, &rflg))
    {
        bits = (uint32_t) rflg.flgptn & SAFETY_ALL; /* cast: UINT 32-bit */
    }

    return bits;
}

/*** end of file ***/
