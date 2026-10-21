/**
 * @file    telem.c
 * @brief   Phase 0 STUB of the telemetry task: telem_task loops, nothing is
 *          queued.
 * @owner   Buddy 1 (communications)
 * @kind    task
 * @reqs    FR10, FR12, NFR8
 */

#include "telem.h"             /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define TELEM_STACK_WORDS    (APP_STACK_TELEM_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_telem_stack[TELEM_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void telem_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * telem_init
 * @brief   STUB: will create K7 MBF_TELEM_EVT and K11 MBF_TELEM.
 * @return  E_OK
 * @context usermain
 */
ER telem_init(void)
{
    return E_OK;
}

/**
 * telem_start
 * @brief   Creates and starts telem_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER telem_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) telem_task, /* cast: kernel entry type */
                             APP_PRI_TELEM, g_telem_stack,
                             APP_STACK_TELEM_BYTES, "telem_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * telem_post_event
 * @brief   STUB: queue an event for telemetry (K7, TMO_POL).
 * @param   type   APP_EVT_*
 * @param   value  event value
 * @return  false (stub: nothing queued)
 * @context mission_task
 */
bool telem_post_event(uint32_t type, uint32_t value)
{
    (void) type;   /* stub: unused until the owner's version */
    (void) value;

    return false;
}

/**
 * telem_post_profile
 * @brief   STUB: queue an obstacle profile (K7, TMO_POL).
 * @param   p_profile  obstacle profile
 * @return  false (stub: nothing queued)
 * @context mission_task
 */
bool telem_post_profile(telem_bin_profile_t const * p_profile)
{
    (void) p_profile;   /* stub: unused until the owner's version */

    return false;
}

/**
 * telem_post_summary
 * @brief   STUB: queue the run summary (K7, TMO_POL).
 * @param   p_summary  end-of-run data
 * @return  false (stub: nothing queued)
 * @context mission_task
 */
bool telem_post_summary(telem_bin_summary_t const * p_summary)
{
    (void) p_summary;   /* stub: unused until the owner's version */

    return false;
}

/**
 * telem_drops
 * @brief   STUB: K7 + K11 + publish drop count.
 * @return  0 (stub)
 * @context any task
 */
uint32_t telem_drops(void)
{
    return 0u;
}

/**
 * telem_k11_id
 * @brief   STUB: ID of K11 for mqtt_bridge.c.
 * @return  0 (stub: K11 not created)
 * @context radio task
 */
ID telem_k11_id(void)
{
    return 0;
}

/* ---- private functions ------------------------------------------------ */

/**
 * telem_task
 * @brief   STUB: telemetry task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context telem_task
 */
static void telem_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
