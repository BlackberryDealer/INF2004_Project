/**
 * @file    line.c
 * @brief   Phase 0 STUB of the line subsystem: line_task loops.
 * @owner   Buddy 3 (line and barcode)
 * @kind    task
 * @reqs    FR1, FR14, FR19, NFR1 (Phase 1-2)
 */

#include "line.h"              /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define LINE_STACK_WORDS     (APP_STACK_LINE_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_line_stack[LINE_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void line_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * line_init
 * @brief   STUB: will create the line subsystem's objects (none planned).
 * @return  E_OK
 * @context usermain
 */
ER line_init(void)
{
    return E_OK;
}

/**
 * line_start
 * @brief   Creates and starts line_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER line_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) line_task, /* cast: kernel entry type */
                             APP_PRI_LINE, g_line_stack,
                             APP_STACK_LINE_BYTES, "line_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * line_task
 * @brief   STUB: 10 ms line sampling task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context line_task
 */
static void line_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
