/**
 * @file    motion_task.c
 * @brief   Phase 0 STUB of the Motion Executor: motion_task loops. Started by
 *          motion_start() in motion.c.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR15, NFR2, NFR3 (implemented in Phase 2)
 */

#include "motion_task.h"       /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- public functions ------------------------------------------------- */

/**
 * motion_task
 * @brief   STUB: 10 ms motion executor task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context motion_task
 */
void motion_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
