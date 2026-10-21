/**
 * @file    sys_tick.c
 * @brief   R2 (10 ms) and R3 (60 ms) cyclic handlers, release latency,
 *          overrun counters and the microsecond time base.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    NFR1, NFR2, NFR17 (release latency), NFR11
 */

/* SDK headers come before our own header here (exception to BARR-C
   4.3.c): the kernel's rp2040 sysdef.h, pulled in by app.h, defines
   GPIO_IN/GPIO_OUT and other hardware macros that break the SDK
   headers if they are parsed afterwards. See SPEC-QUESTION Q1.     */
#include "hardware/timer.h"    /* time_us_32(): 1 MHz system timer */
#include "sys_tick.h"          /* own header, after the SDK (Q1) */

/* ---- private constants and types ------------------------------------ */

#define SYS_TICK_FAST_MS     (10u)    /* R2 period and phase */
#define SYS_TICK_SLOW_MS     (60u)    /* R3 period and phase */
#define SYS_TICK_REL_FAST    (0u)     /* g_rel_us index for R2 */
#define SYS_TICK_REL_SLOW    (1u)     /* g_rel_us index for R3 */
#define SYS_TICK_REL_COUNT   (2u)
#define SYS_TICK_FAST_SLOTS  (3u)     /* slots 0..2 are woken by R2 */
#define SYS_TICK_LAT_SAT_US  (4095u)  /* 12 bits (E7)  */
#define SYS_TICK_OVR_SAT     (255u)   /* 8 bits        */

/* ---- private data ---------------------------------------------------- */

/* E5: release stamps; [0] written only by R2, [1] only by R3. */
static volatile uint32_t g_rel_us[SYS_TICK_REL_COUNT] = {0u, 0u};

/* Written once per task by sys_tick_register(), read by handlers. */
static volatile ID g_tick_tid[SYS_TICK_SLOT_COUNT] = {0, 0, 0, 0};

/* E7: slot N written only by its own task. */
static volatile uint16_t g_lat_max_us[SYS_TICK_SLOT_COUNT] =
{
    0u, 0u, 0u, 0u
};
static volatile uint8_t  g_overruns[SYS_TICK_SLOT_COUNT] = {0u, 0u, 0u, 0u};

static ID gh_r2_cyc = 0;
static ID gh_r3_cyc = 0;

/* ---- private function prototypes ------------------------------------- */

static void sys_tick_10ms_handler(void * p_exinf);
static void sys_tick_60ms_handler(void * p_exinf);
static ER   sys_tick_create(FP p_handler, uint32_t period_ms, ID * ph_cyc);
static void sys_tick_wake(uint32_t slot);

/* ---- public functions ------------------------------------------------- */

/**
 * sys_tick_init
 * @brief   Creates R2 (10 ms) and R3 (60 ms), both stopped.
 * @return  E_OK, or the first kernel error
 * @context usermain (init phase)
 */
ER sys_tick_init(void)
{
    ER ercd = sys_tick_create((FP) sys_tick_10ms_handler, /* cast: kernel */
                              SYS_TICK_FAST_MS, &gh_r2_cyc);

    if (E_OK == ercd)
    {
        ercd = sys_tick_create((FP) sys_tick_60ms_handler, /* cast: kernel */
                               SYS_TICK_SLOW_MS, &gh_r3_cyc);
    }

    return ercd;
}

/**
 * sys_tick_start
 * @brief   Starts R2 and R3.
 * @return  E_OK, or the first kernel error
 * @context usermain (start phase)
 */
ER sys_tick_start(void)
{
    ER ercd = tk_sta_cyc(gh_r2_cyc);

    if (E_OK == ercd)
    {
        ercd = tk_sta_cyc(gh_r3_cyc);
    }

    return ercd;
}

/**
 * sys_tick_register
 * @brief   Records which task a slot's handler wakes.
 * @param   slot    SYS_TICK_SLOT_*
 * @param   task_id the calling task's ID (tk_get_tid())
 * @return  E_OK, or E_PAR for an unknown slot
 * @context periodic task, once before its loop
 */
ER sys_tick_register(uint32_t slot, ID task_id)
{
    ER ercd = E_PAR;

    if (slot < SYS_TICK_SLOT_COUNT)
    {
        g_tick_tid[slot] = task_id;
        ercd             = E_OK;
    }

    return ercd;
}

/**
 * sys_tick_cycle_start
 * @brief   Records release latency and overruns for this cycle.
 * @param   slot SYS_TICK_SLOT_* of the calling task
 * @context the task registered on that slot, right after tk_slp_tsk()
 */
void sys_tick_cycle_start(uint32_t slot)
{
    uint32_t const now_us  = sys_tick_now_us();
    uint32_t       rel_us  = 0u;
    uint32_t       lat_us  = 0u;
    INT            queued  = 0;
    uint32_t       overrun = 0u;

    if (slot < SYS_TICK_SLOT_COUNT)
    {
        if (slot < SYS_TICK_FAST_SLOTS)
        {
            rel_us = g_rel_us[SYS_TICK_REL_FAST];
        }
        else
        {
            rel_us = g_rel_us[SYS_TICK_REL_SLOW];
        }

        lat_us = now_us - rel_us;   /* unsigned: correct across a wrap */
        if (lat_us > SYS_TICK_LAT_SAT_US)
        {
            lat_us = SYS_TICK_LAT_SAT_US;
        }
        if (lat_us > (uint32_t) g_lat_max_us[slot])
        {
            g_lat_max_us[slot] = (uint16_t) lat_us; /* cast: <= 4095 */
        }

        queued = tk_can_wup(TSK_SELF);
        if (queued > 0)
        {
            overrun = (uint32_t) g_overruns[slot]
                      + (uint32_t) queued;  /* cast: queued > 0 */
            if (overrun > SYS_TICK_OVR_SAT)
            {
                overrun = SYS_TICK_OVR_SAT;
            }
            g_overruns[slot] = (uint8_t) overrun; /* cast: <= 255 */
        }
    }
}

/**
 * sys_tick_now_us
 * @brief   Free-running 1 MHz time; subtract unsigned (wraps at 71 min).
 * @return  microseconds since boot, modulo 2^32
 * @context any (task, handler or ISR)
 */
uint32_t sys_tick_now_us(void)
{
    return time_us_32();
}

/**
 * sys_tick_latency_max_us
 * @brief   Worst release latency seen by any slot.
 * @return  microseconds, saturated at 4095
 * @context any task
 */
uint32_t sys_tick_latency_max_us(void)
{
    uint32_t worst_us = 0u;

    for (uint32_t idx = 0u; idx < SYS_TICK_SLOT_COUNT; idx++)
    {
        if ((uint32_t) g_lat_max_us[idx] > worst_us)
        {
            worst_us = (uint32_t) g_lat_max_us[idx];
        }
    }

    return worst_us;
}

/**
 * sys_tick_overruns
 * @brief   Queued wake-ups (missed cycles) counted for one slot.
 * @param   slot SYS_TICK_SLOT_*
 * @return  count saturated at 255; 0 for an unknown slot
 * @context any task
 */
uint32_t sys_tick_overruns(uint32_t slot)
{
    uint32_t count = 0u;

    if (slot < SYS_TICK_SLOT_COUNT)
    {
        count = (uint32_t) g_overruns[slot];
    }

    return count;
}

/* ---- private functions ------------------------------------------------ */

/**
 * sys_tick_10ms_handler
 * @brief   R2: stamps the release and wakes line, motion and IMU tasks.
 * @param   p_exinf unused
 * @context cyclic handler (interrupt context)
 */
static void sys_tick_10ms_handler(void * p_exinf)
{
    (void) p_exinf;  /* unused: no extended info */

    g_rel_us[SYS_TICK_REL_FAST] = sys_tick_now_us();
    for (uint32_t idx = 0u; idx < SYS_TICK_FAST_SLOTS; idx++)
    {
        sys_tick_wake(idx);
    }
}

/**
 * sys_tick_60ms_handler
 * @brief   R3: stamps the release and wakes obstacle_task.
 * @param   p_exinf unused
 * @context cyclic handler (interrupt context)
 */
static void sys_tick_60ms_handler(void * p_exinf)
{
    (void) p_exinf;  /* unused: no extended info */

    g_rel_us[SYS_TICK_REL_SLOW] = sys_tick_now_us();
    sys_tick_wake(SYS_TICK_SLOT_OBSTACLE);
}

/**
 * sys_tick_create
 * @brief   Creates one stopped cyclic handler.
 * @param   p_handler handler entry
 * @param   period_ms period and first phase, ms
 * @param   ph_cyc    receives the handler ID
 * @return  E_OK, or the kernel error
 * @context usermain (init phase)
 */
static ER sys_tick_create(FP p_handler, uint32_t period_ms, ID * ph_cyc)
{
    T_CCYC ccyc = {0};
    ER     ercd = E_OK;

    ccyc.cycatr = TA_HLNG;              /* no TA_STA: started later */
    ccyc.cychdr = p_handler;
    ccyc.cyctim = (RELTIM) period_ms;   /* cast: kernel time type */
    ccyc.cycphs = (RELTIM) period_ms;   /* cast: kernel time type */
    *ph_cyc     = tk_cre_cyc(&ccyc);

    if (*ph_cyc < E_OK)
    {
        ercd = (ER) *ph_cyc;  /* cast: negative ID is an error code */
    }

    return ercd;
}

/**
 * sys_tick_wake
 * @brief   Wakes the task registered on a slot, if any.
 * @param   slot SYS_TICK_SLOT_*
 * @context cyclic handler (interrupt context)
 */
static void sys_tick_wake(uint32_t slot)
{
    ID const task_id = g_tick_tid[slot];

    if (task_id > 0)
    {
        (void) tk_wup_tsk(task_id);  /* E_QOVR is counted as an overrun */
    }
}

/*** end of file ***/
