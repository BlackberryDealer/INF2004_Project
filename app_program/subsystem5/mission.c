/**
 * @file    mission.c
 * @brief   Phase 0 STUB of the mission: mission_task loops; events are not
 *          queued.
 * @owner   Buddy 5 (shared services, obstacle, mission)
 * @kind    task
 * @reqs    FR3, FR14, FR16, FR17 (Phase 0 core, Phase 2)
 */

#include "mission.h"           /* own header first (BARR-C 4.3.c) */
#include "../app.h"

/* ---- private constants and types ------------------------------------ */

#define MISSION_STACK_WORDS  (APP_STACK_MISSION_BYTES / 8u)
#define STUB_TASK_DELAY_MS   (1000u) /* stub pass period */

/* ---- private data ---------------------------------------------------- */

static uint64_t g_mission_stack[MISSION_STACK_WORDS];

/* ---- private function prototypes ------------------------------------- */

static void mission_task(INT stacd, void * p_exinf);

/* ---- public functions ------------------------------------------------- */

/**
 * mission_init
 * @brief   STUB: will create K4 MBF_MISSION_EVT and K10 FLG_MODE.
 * @return  E_OK
 * @context usermain
 */
ER mission_init(void)
{
    return E_OK;
}

/**
 * mission_start
 * @brief   Creates and starts mission_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER mission_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) mission_task, /* cast: kernel entry type */
                             APP_PRI_MISSION, g_mission_stack,
                             APP_STACK_MISSION_BYTES, "mission_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * mission_post_event
 * @brief   STUB: post one event word to K4 (TMO_POL).
 * @param   type   APP_EVT_*
 * @param   value  event value
 * @return  false (stub: not queued)
 * @context any task or the radio task
 */
bool mission_post_event(uint32_t type, uint32_t value)
{
    (void) type;   /* stub: unused until the owner's version */
    (void) value;

    return false;
}

/**
 * mission_evt_barcode_value
 * @brief   STUB: builds a BARCODE / MAP event value.
 * @param   cmd         APP_BAR_CMD_*
 * @param   char_index  Code 39 index 0..42
 * @return  0 (stub)
 * @context any task
 */
uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    (void) cmd;   /* stub: unused until the owner's version */
    (void) char_index;

    return 0u;
}

/**
 * mission_mode_set
 * @brief   STUB: set mode request bits.
 * @param   bits  APP_MODE_* bits
 * @return  none
 * @context mission_task
 */
void mission_mode_set(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * mission_mode_clear
 * @brief   STUB: clear mode request bits.
 * @param   bits  APP_MODE_* bits
 * @return  none
 * @context mission_task
 */
void mission_mode_clear(uint32_t bits)
{
    (void) bits;   /* stub: unused until the owner's version */
}

/**
 * mission_mode_is_set
 * @brief   STUB: is a mode bit set?
 * @param   bit  one APP_MODE_* bit
 * @return  false (stub)
 * @context any task
 */
bool mission_mode_is_set(uint32_t bit)
{
    (void) bit;   /* stub: unused until the owner's version */

    return false;
}

/**
 * mission_queue_high_water
 * @brief   STUB: observed K4 peak.
 * @return  0 (stub)
 * @context any task
 */
uint32_t mission_queue_high_water(void)
{
    return 0u;
}

/* ---- private functions ------------------------------------------------ */

/**
 * mission_task
 * @brief   STUB: mission statechart task. Sleeps 1 s per pass so the boot
 *          log shows the task running. Replaced in Phase 1-2.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context mission_task
 */
static void mission_task(INT stacd, void * p_exinf)
{
    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        (void) tk_dly_tsk(STUB_TASK_DELAY_MS); /* stub: Phase 1 */
    }
}

/*** end of file ***/
