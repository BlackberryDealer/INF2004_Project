/**
 * @file    obstacle.c
 * @brief   Phase 0 STUB of the obstacle subsystem: obstacle_task loops.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 * @reqs    FR6-FR9, NFR6 (Phase 1-2)
 */

#include "obstacle.h"          /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define OBSTACLE_STACK_WORDS (APP_STACK_OBSTACLE_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_obstacle_stack[OBSTACLE_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void obstacle_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * obstacle_init
 * @brief   STUB: will create K1 SEM_ECHO.
 * @return  E_OK
 * @context usermain
 */
ER obstacle_init(void)
{
    return E_OK;
}

/**
 * obstacle_start
 * @brief   Creates and starts obstacle_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER obstacle_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) obstacle_task, /* cast: kernel entry type */
                             APP_PRI_OBSTACLE, g_obstacle_stack,
                             APP_STACK_OBSTACLE_BYTES, "obstacle_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * obstacle_task
 * @brief   STUB: 60 ms ultrasonic scanner task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context obstacle_task
 */
static void obstacle_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
