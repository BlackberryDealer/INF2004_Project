/**
 * @file    imu.c
 * @brief   Phase 0 STUB of the IMU subsystem: imu_task loops.
 * @owner   Buddy 4 (IMU and terrain)
 * @kind    task
 * @reqs    FR4, FR5, FR13, FR18, NFR7 (Phase 1-2)
 */

#include "imu.h"               /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define IMU_STACK_WORDS      (APP_STACK_IMU_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_imu_stack[IMU_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void imu_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * imu_init
 * @brief   STUB: will create the IMU subsystem's objects (none planned).
 * @return  E_OK
 * @context usermain
 */
ER imu_init(void)
{
    return E_OK;
}

/**
 * imu_start
 * @brief   Creates and starts imu_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER imu_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) imu_task, /* cast: kernel entry type */
                             APP_PRI_IMU, g_imu_stack,
                             APP_STACK_IMU_BYTES, "imu_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * imu_task
 * @brief   STUB: 10 ms IMU and terrain task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context imu_task
 */
static void imu_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
