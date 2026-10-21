/**
 * @file    barcode.c
 * @brief   Phase 0 STUB of the barcode subsystem: barcode_task loops.
 * @owner   Buddy 3 (line and barcode)
 * @kind    task
 * @reqs    FR2, FR3, NFR5 (Phase 1-2)
 */

#include "barcode.h"           /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define BARCODE_STACK_WORDS  (APP_STACK_BARCODE_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_barcode_stack[BARCODE_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void barcode_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * barcode_init
 * @brief   STUB: will create K2 SEM_BAR and the edge ring.
 * @return  E_OK
 * @context usermain
 */
ER barcode_init(void)
{
    return E_OK;
}

/**
 * barcode_start
 * @brief   Creates and starts barcode_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER barcode_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) barcode_task, /* cast: kernel entry type */
                             APP_PRI_BARCODE, g_barcode_stack,
                             APP_STACK_BARCODE_BYTES, "barcode_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * barcode_map_set
 * @brief   STUB: remap one barcode character to a command.
 * @param   char_index  Code 39 character index 0..42
 * @param   cmd         APP_BAR_CMD_*
 * @return  none
 * @context mission_task
 */
void barcode_map_set(uint32_t char_index, uint32_t cmd)
{
    (void) char_index;   /* stub: unused until the owner's version */
    (void) cmd;
}

/**
 * barcode_branch_seq
 * @brief   STUB: +1 per long single bar seen.
 * @return  0 (stub)
 * @context line_task
 */
uint32_t barcode_branch_seq(void)
{
    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/**
 * barcode_task
 * @brief   STUB: barcode decoder task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context barcode_task
 */
static void barcode_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
