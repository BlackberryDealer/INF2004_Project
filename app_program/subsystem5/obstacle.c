/**
 * @file    obstacle.c
 * @brief   obstacle_task on R3 (Phase 0: empty 60 ms cycle that records
 *          release latency and checks in with sys_health).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    NFR17, NFR10b
 */

#include "obstacle.h"          /* own header first (BARR-C 4.3.c) */
#include "sys_tick.h"
#include "sys_health.h"

/* ---- private constants and types ------------------------------------ */

#define OBSTACLE_STACK_WORDS   (APP_STACK_OBSTACLE_BYTES / 8u)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_obstacle_stack[OBSTACLE_STACK_WORDS];
static ID       gh_obstacle_task = 0;

/* ---- private function prototypes ------------------------------------- */

static void obstacle_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * obstacle_init
 * @brief   Phase 0: nothing to create yet.
 * @return  E_OK
 * @context usermain (init phase)
 */
ER obstacle_init(void)
{
    return E_OK;
}

/**
 * obstacle_start
 * @brief   Creates and starts obstacle_task on its static stack.
 * @return  E_OK, or the kernel error
 * @context usermain (start phase)
 */
ER obstacle_start(void)
{
    ER ercd = E_OK;

    gh_obstacle_task = app_start_task((FP) obstacle_task, /* cast: kernel */
                                      APP_PRI_OBSTACLE, g_obstacle_stack,
                                      APP_STACK_OBSTACLE_BYTES,
                                      "obstacle_task");
    if (gh_obstacle_task < E_OK)
    {
        ercd = (ER) gh_obstacle_task;  /* cast: negative ID is an error */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * obstacle_task
 * @brief   Phase 0 body: wakes on R3 every 60 ms, records latency and
 *          checks in.
 * @param   stacd   unused (kernel start code)
 * @param   p_exinf unused
 * @context obstacle_task (priority 15)
 */
static void obstacle_task(INT stacd, void * p_exinf)
{
    (void) stacd;    /* unused: kernel start code */
    (void) p_exinf;  /* unused: no extended info  */

    (void) sys_tick_register(SYS_TICK_SLOT_OBSTACLE,
                             tk_get_tid());  /* valid slot constant */
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);  /* woken by R3 */
        sys_tick_cycle_start(SYS_TICK_SLOT_OBSTACLE);
        sys_health_check_in(SYS_HEALTH_BIT_OBSTACLE);
    }
}

/*** end of file ***/
