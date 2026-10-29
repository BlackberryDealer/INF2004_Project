/**
 * @file    sys_tick.c
 * @brief   R2 (10 ms) and R3 (60 ms) cyclic handlers, release latency,
 *          overrun counters and the microsecond time base.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    NFR1, NFR2, NFR17 (release latency), NFR11
 */

/* Kernel side only (build rule, BARR_C_DEVIATIONS.md): no SDK headers;
   the 1 MHz timer is read with in_w(TIMER_TIMERAWL).                 */
#include "sys_tick.h"          /* own header first (BARR-C 4.3.c) */

/* ---- private constants and types ------------------------------------ */

#define SYS_TICK_FAST_MS     (10u)    /* R2 period and phase */
#define SYS_TICK_SLOW_MS     (60u)    /* R3 period and phase */
#define SYS_TICK_REL_FAST    (0u)     /* g_rel_us index for R2 */
#define SYS_TICK_REL_SLOW    (1u)     /* g_rel_us index for R3 */
#define SYS_TICK_REL_COUNT   (2u)
#define SYS_TICK_FAST_SLOTS  (3u)     /* slots 0..2 are woken by R2 */
#define SYS_TICK_LAT_SAT_US  (4095u)  /* 12 bits (E7)  */
#define SYS_TICK_OVR_SAT     (255u)   /* 8 bits        */

/* The TIMER counts only from the watchdog tick generator (RP2040
   datasheet 4.7.2). The SDK starts it in runtime_init_clocks.c, which
   this template does not link, and nothing in the kernel starts it
   (fix brief, Fix 4). clk_ref = 12 MHz XOSC (cpu_clock.c), so 12
   cycles = 1 us. Writing the same value again is harmless.           */
#define SYS_TICK_WDT_TICK        (0x40058000u + 0x2Cu)  /* WATCHDOG_TICK */
#define SYS_TICK_WDT_TICK_ENABLE (0x00000200u)
#define SYS_TICK_CLK_REF_MHZ     (12u)                  /* XOSC, cpu_clock.c */

/* ---- private data ---------------------------------------------------- */

/* Low 16 bits of the release stamps: [0] written only by R2, [1] only by
   R3. 16 bits are enough: a latency only matters below 65 ms (it
   saturates at 4095 us, and a longer delay also shows as an overrun). */
static volatile uint16_t g_rel_us16[SYS_TICK_REL_COUNT] = {0u, 0u};

/* Written once per task by sys_tick_register(), read by handlers. */
static volatile ID g_tick_tid[SYS_TICK_SLOT_COUNT] = {0, 0, 0, 0};

/* E7: slot N written only by its own task. */
static volatile uint16_t g_lat_max_us[SYS_TICK_SLOT_COUNT] =
{
    0u, 0u, 0u, 0u
};
static volatile uint8_t  g_overruns[SYS_TICK_SLOT_COUNT] = {0u, 0u, 0u, 0u};

/* mission_task (the watchdog feeder), also woken by R2 but kept out of
   the latency and overrun statistics (NFR17 covers the 4 slots). */
static volatile ID g_feeder_tid = 0;

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
 * @brief   Starts the 1 us timer tick, then creates R2 (10 ms) and R3
 *          (60 ms), both stopped.
 * @return  E_OK, or the first kernel error
 * @context usermain (init phase, before anything reads the timer)
 */
ER sys_tick_init(void)
{
    ER ercd = E_OK;

    out_w(SYS_TICK_WDT_TICK, SYS_TICK_WDT_TICK_ENABLE | SYS_TICK_CLK_REF_MHZ);
    ercd = sys_tick_create((FP) sys_tick_10ms_handler, /* cast: kernel */
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
 * sys_tick_register_feeder
 * @brief   Records the watchdog feeder (mission_task) for the R2 wake.
 *          With the 10 ms kernel tick (CNF_TIMER_PERIOD) a 10 ms receive
 *          timeout waits 10-20 ms, which left only ~10 ms of the 30 ms
 *          watchdog window (on-car/emulator trace 2026-10-29); R2 makes
 *          the feeder's passes 10 ms apart like the other fast tasks.
 * @param   task_id the calling task's ID (tk_get_tid())
 * @return  none
 * @context mission_task, once before its loop
 */
void sys_tick_register_feeder(ID task_id)
{
    g_feeder_tid = task_id;
}

/**
 * sys_tick_cycle_start
 * @brief   Records release latency and overruns for this cycle.
 * @param   slot SYS_TICK_SLOT_* of the calling task
 * @context the task registered on that slot, right after tk_slp_tsk()
 */
void sys_tick_cycle_start(uint32_t slot)
{
    /* cast: keep the low 16 bits of the 1 MHz time */
    uint16_t const now16   = (uint16_t) sys_tick_now_us();
    uint16_t       rel16   = 0u;
    uint32_t       lat_us  = 0u;
    INT            queued  = 0;
    uint32_t       overrun = 0u;

    if (slot < SYS_TICK_SLOT_COUNT)
    {
        if (slot < SYS_TICK_FAST_SLOTS)
        {
            rel16 = g_rel_us16[SYS_TICK_REL_FAST];
        }
        else
        {
            rel16 = g_rel_us16[SYS_TICK_REL_SLOW];
        }

        /* cast: explicit 16-bit modular subtraction, correct across a wrap */
        lat_us = (uint32_t) (uint16_t) (now16 - rel16);
        if (lat_us > SYS_TICK_LAT_SAT_US)
        {
            lat_us = SYS_TICK_LAT_SAT_US;
        }
        if (lat_us > (uint32_t) g_lat_max_us[slot])   /* cast: widen */
        {
            g_lat_max_us[slot] = (uint16_t) lat_us; /* cast: <= 4095 */
        }

        queued = tk_can_wup(TSK_SELF);
        if (queued > 0)
        {
            /* casts: widen; queued > 0 */
            overrun = (uint32_t) g_overruns[slot] + (uint32_t) queued;
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
    return (uint32_t) in_w(TIMER_TIMERAWL);  /* cast: UW is 32-bit; raw low
                                               word, no latch side effect */
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
        if ((uint32_t) g_lat_max_us[idx] > worst_us)   /* cast: widen */
        {
            worst_us = (uint32_t) g_lat_max_us[idx];   /* cast: widen */
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
        count = (uint32_t) g_overruns[slot];   /* cast: widen */
    }

    return count;
}

/* ---- private functions ------------------------------------------------ */

/**
 * sys_tick_10ms_handler
 * @brief   R2: stamps the release and wakes line, motion and IMU tasks,
 *          then mission_task (the watchdog feeder).
 * @param   p_exinf unused
 * @context cyclic handler (interrupt context)
 */
static void sys_tick_10ms_handler(void * p_exinf)
{
    (void) p_exinf;  /* unused: no extended info */

    /* cast: keep the low 16 bits */
    g_rel_us16[SYS_TICK_REL_FAST] = (uint16_t) sys_tick_now_us();
    for (uint32_t idx = 0u; idx < SYS_TICK_FAST_SLOTS; idx++)
    {
        sys_tick_wake(idx);
    }
    if (g_feeder_tid > 0)
    {
        (void) tk_wup_tsk(g_feeder_tid); /* E_QOVR: it is behind anyway */
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

    /* cast: keep the low 16 bits */
    g_rel_us16[SYS_TICK_REL_SLOW] = (uint16_t) sys_tick_now_us();
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
