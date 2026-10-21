/**
 * @file    sys_health.c
 * @brief   K8 FLG_ALIVE check-ins and the hardware watchdog. mission_task
 *          feeds the watchdog only with fresh check-ins from all four
 *          periodic tasks.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    NFR10b (watchdog), B5.0.3
 *
 * Reset bounds (reviews R09, V13-01, V14-03):
 *  - Normal operation (30 ms window). Feeds come <= 20 ms apart (mission
 *    pass <= 10 ms + task phase <= 10 ms) and obstacle's own check-in
 *    age never exceeds 61 ms, so 30 ms and 65 ms leave margin. From the
 *    moment a task hangs:
 *      a 10 ms task right after its check-in: at most one more feed,
 *        <= 10 ms later -> reset <= 10 + 30 = 40 ms;
 *      obstacle_task right after its check-in: feeds go on until its
 *        age reaches 65 ms -> reset <= 65 + 30 = 95 ms;
 *      mission_task (the feeder) -> reset <= 30 ms.
 *    So every hang resets within 95 ms <= 100 ms (NFR10b).
 *  - Startup (500 ms window). Nothing is fed until all four tasks have
 *    checked in once; then the window switches to 30 ms. A task that
 *    never starts resets the board 500 ms after BOOT.
 *
 * The SDK's watchdog.c does not build in this template (it pulls in
 * pico/bootrom.h -> pico/flash.h, which the template lacks), so the
 * watchdog is driven through its registers exactly as the SDK's
 * _watchdog_enable() does for the RP2040, with the same window macros
 * (B5.0.3 fallback).
 */

/* SDK headers come before our own header here (exception to BARR-C
   4.3.c): the kernel's rp2040 sysdef.h, pulled in by app.h, defines
   GPIO_IN/GPIO_OUT and other hardware macros that break the SDK
   headers if they are parsed afterwards. See SPEC-QUESTION Q1.     */
#include "hardware/address_mapped.h"   /* hw_set_bits, hw_clear_bits */
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"
#include "sys_health.h"        /* own header, after the SDK (Q1) */
#include "drv_led_alive.h"
#include "sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define SYS_HEALTH_FAST_BITS    (SYS_HEALTH_BIT_LINE | SYS_HEALTH_BIT_MOTION \
                                 | SYS_HEALTH_BIT_IMU)
#define SYS_HEALTH_US_PER_MS    (1000u)

/* RP2040 erratum E1: the counter decrements twice per 1 us tick, so the
   load is 2 x the delay in us (the SDK's WATCHDOG_XFACTOR).           */
#define SYS_HEALTH_WDT_XFACTOR  (2u)
#define SYS_HEALTH_LOAD_NORMAL  (SYS_HEALTH_WDT_MS * SYS_HEALTH_US_PER_MS \
                                 * SYS_HEALTH_WDT_XFACTOR)
#define SYS_HEALTH_LOAD_START   (SYS_HEALTH_WDT_START_MS                  \
                                 * SYS_HEALTH_US_PER_MS                   \
                                 * SYS_HEALTH_WDT_XFACTOR)
#define SYS_HEALTH_WDT_DBG_BITS (WATCHDOG_CTRL_PAUSE_DBG0_BITS            \
                                 | WATCHDOG_CTRL_PAUSE_DBG1_BITS          \
                                 | WATCHDOG_CTRL_PAUSE_JTAG_BITS)
#define SYS_HEALTH_WDT_NONBOOT  (0x6ab73121u)  /* SDK: "not a reboot" */
#define SYS_HEALTH_SCRATCH_IDX  (4u)
#define SYS_HEALTH_REASON_BITS  (WATCHDOG_REASON_TIMER_BITS               \
                                 | WATCHDOG_REASON_FORCE_BITS)

#if (SYS_HEALTH_LOAD_START > WATCHDOG_LOAD_BITS)
#error "Watchdog startup window too long for the 24-bit LOAD field"
#endif
#if (SYS_HEALTH_LOAD_NORMAL >= SYS_HEALTH_LOAD_START)
#error "The normal watchdog window must be shorter than the startup one"
#endif

/* ---- private data ---------------------------------------------------- */

static ID gh_alive_flg = 0;

/* E5: obstacle_task's own check-in time (one aligned 32-bit store,
   written only by obstacle_task) and its "has checked in" flag.    */
static volatile uint32_t g_obs_checkin_us      = 0u;
static volatile uint8_t  gb_obs_has_checked_in = 0u;

/* Written only by mission_task. */
static bool     gb_is_wdt_active = false;
static bool     gb_is_started    = false;
static uint32_t g_wdt_load       = SYS_HEALTH_LOAD_START;

/* ---- private function prototypes ------------------------------------- */

static void sys_health_wdt_start(uint32_t load);

/* ---- public functions ------------------------------------------------- */

/**
 * sys_health_init
 * @brief   Creates FLG_ALIVE (K8), sets up the alive LED and prints why
 *          the board last reset.
 * @return  E_OK, or the kernel error
 * @context usermain (init phase)
 */
ER sys_health_init(void)
{
    T_CFLG cflg = {0};
    ER     ercd = E_OK;

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    gh_alive_flg = tk_cre_flg(&cflg);

    if (gh_alive_flg < E_OK)
    {
        ercd = (ER) gh_alive_flg;  /* cast: negative ID is an error code */
    }
    drv_led_alive_init();
    /* cast: hardware reason bits, 2 bits */
    app_log("RESET cause watchdog %d\n",
            (int32_t) (watchdog_hw->reason & SYS_HEALTH_REASON_BITS), 0, 0);

    return ercd;
}

/**
 * sys_health_check_in
 * @brief   Marks one periodic task as alive. Call once per cycle, right
 *          after sys_tick_cycle_start(), before any wait in the cycle.
 *          OBSTACLE also stores its own check-in time.
 * @param   task_bit SYS_HEALTH_BIT_*
 * @context line_task, motion_task, imu_task, obstacle_task
 */
void sys_health_check_in(uint32_t task_bit)
{
    if (SYS_HEALTH_BIT_OBSTACLE == task_bit)
    {
        g_obs_checkin_us      = sys_tick_now_us();   /* one 32-bit store */
        gb_obs_has_checked_in = 1u;
    }
    /* set cannot fail with a valid ID; cast: kernel pattern type */
    (void) tk_set_flg(gh_alive_flg, (UINT) (task_bit & SYS_HEALTH_BIT_ALL));
}

/**
 * sys_health_service
 * @brief   Startup: once LINE, MOTION and IMU are set and obstacle has
 *          checked in, switch to the 30 ms window. Normal: feed only if
 *          the three fast bits are set and obstacle's check-in is at most
 *          65 ms old; then clear the fast bits and toggle the alive LED.
 * @context mission_task, every pass (<= 10 ms apart)
 */
void sys_health_service(void)
{
    T_RFLG         rflg     = {0};
    uint32_t const obs_us   = g_obs_checkin_us;          /* one load */
    uint32_t const age_us   = sys_tick_now_us() - obs_us; /* wrap-safe */
    uint32_t const limit_us = SYS_HEALTH_OBS_MAX_AGE_MS * SYS_HEALTH_US_PER_MS;
    bool           b_fast   = false;
    bool           b_feed   = false;

    if (E_OK == tk_ref_flg(gh_alive_flg, &rflg))
    {
        /* cast: kernel pattern to fixed width */
        b_fast = (SYS_HEALTH_FAST_BITS
                  == (((uint32_t) rflg.flgptn) & SYS_HEALTH_FAST_BITS));
    }

    if (!gb_is_started)
    {
        if (b_fast && (0u != gb_obs_has_checked_in) && gb_is_wdt_active)
        {
            sys_health_wdt_start(SYS_HEALTH_LOAD_NORMAL); /* 30 ms window */
            gb_is_started = true;
            b_feed        = true;
        }
    }
    else if (b_fast && (age_us <= limit_us))
    {
        watchdog_hw->load = g_wdt_load;                   /* feed */
        b_feed            = true;
    }
    else
    {
        /* not fed: a stale check-in is never used twice */
    }

    if (b_feed)
    {
        /* tk_clr_flg ANDs: pass the inverted mask; cast: kernel type */
        (void) tk_clr_flg(gh_alive_flg, (UINT) ~SYS_HEALTH_FAST_BITS);
        drv_led_alive_toggle();
    }
}

/**
 * sys_health_enable_watchdog
 * @brief   Starts the 500 ms startup window and does not feed it; the
 *          switch to 30 ms happens in sys_health_service().
 * @context mission_task (BOOT entry, first boot only, motors braked)
 */
void sys_health_enable_watchdog(void)
{
    sys_health_wdt_start(SYS_HEALTH_LOAD_START);
    gb_is_wdt_active = true;
}

/**
 * sys_health_is_started
 * @brief   True once all four tasks have checked in for the first time
 *          and the normal 30 ms window runs.
 * @return  gb_is_started
 * @context mission_task (BOOT waits for it)
 */
bool sys_health_is_started(void)
{
    return gb_is_started;
}

/* ---- private functions ------------------------------------------------ */

/**
 * sys_health_wdt_start
 * @brief   (Re)starts the watchdog with a load value, paused while a
 *          debugger halts the cores. Same writes as the SDK's
 *          watchdog_enable(); relies on the 1 MHz watchdog tick that the
 *          system timer (sys_tick_now_us) also needs.
 * @param   load LOAD field value (2 x window in us, erratum E1)
 * @context mission_task
 */
static void sys_health_wdt_start(uint32_t load)
{
    g_wdt_load = load;
    watchdog_hw->scratch[SYS_HEALTH_SCRATCH_IDX] = SYS_HEALTH_WDT_NONBOOT;
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    /* reset everything except the oscillators when it fires */
    hw_set_bits(&psm_hw->wdsel,
                PSM_WDSEL_BITS
                & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
    hw_set_bits(&watchdog_hw->ctrl, SYS_HEALTH_WDT_DBG_BITS);
    watchdog_hw->load = load;
    hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
}

/*** end of file ***/
