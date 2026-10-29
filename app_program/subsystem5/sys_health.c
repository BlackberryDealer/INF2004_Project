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
 *  - Radio bring-up (SHARED fix, flash audit 2026-10-22). The CYW43 task
 *    runs at priority 4, above every application task, and its bring-up
 *    holds the CPU for long stretches (firmware download, a busy wait of
 *    up to 150 ms in cyw43_ll.c). A 30 ms window armed during that time
 *    would reset the board on every boot. So after the first check-ins
 *    the 500 ms window stays and is fed (same fresh-check-in rule) until
 *    the radio reports init_complete, or SYS_HEALTH_RADIO_WAIT_MS has
 *    passed since BOOT; only then does it switch to 30 ms. mission stays
 *    in BOOT (motors braked) for that time, so the 95 ms RUN bound is
 *    unchanged. Builds without the radio (WIFI=none) switch at once.
 *
 * The SDK's watchdog.c does not build in this template (it pulls in
 * pico/bootrom.h -> pico/flash.h, which the template lacks), so the
 * watchdog is driven through its registers exactly as the SDK's
 * _watchdog_enable() does for the RP2040, with the same window macros
 * (B5.0.3 fallback).
 */

/* Kernel side only (build rule, BARR_C_DEVIATIONS.md): no SDK headers;
   the watchdog and PSM registers are reached with in_w/out_w/set_w.  */
#include "sys_health.h"        /* own header first (BARR-C 4.3.c) */
#include "drv_led_alive.h"
#include "sys_tick.h"
#if defined(TM_WIFI_CYW43) && (TM_WIFI_CYW43 != 0)
/* Plain stdint header (no SDK or kernel types): radio init status. */
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#endif

/* ---- private constants and types ------------------------------------ */

#define SYS_HEALTH_FAST_BITS    (SYS_HEALTH_BIT_LINE | SYS_HEALTH_BIT_MOTION \
                                 | SYS_HEALTH_BIT_IMU)
#define SYS_HEALTH_US_PER_MS    (1000u)

/* Watchdog and PSM registers (values as in the SDK's regs/watchdog.h,
   regs/psm.h, addressmap.h), driven with the kernel's in_w/out_w and the
   atomic set_w/clr_w aliases.                                          */
#define SYS_HEALTH_WDT_BASE         (0x40058000u)
#define SYS_HEALTH_WDT_CTRL         (SYS_HEALTH_WDT_BASE + 0x00u)
#define SYS_HEALTH_WDT_LOAD         (SYS_HEALTH_WDT_BASE + 0x04u)
#define SYS_HEALTH_WDT_REASON       (SYS_HEALTH_WDT_BASE + 0x08u)
#define SYS_HEALTH_WDT_SCRATCH0     (SYS_HEALTH_WDT_BASE + 0x0Cu)
#define SYS_HEALTH_WDT_REG_BYTES    (4u)
#define SYS_HEALTH_WDT_CTRL_ENABLE  (0x40000000u)
#define SYS_HEALTH_WDT_PAUSE_JTAG   (0x01000000u)
#define SYS_HEALTH_WDT_PAUSE_DBG0   (0x02000000u)
#define SYS_HEALTH_WDT_PAUSE_DBG1   (0x04000000u)
#define SYS_HEALTH_WDT_LOAD_MASK    (0x00FFFFFFu)
#define SYS_HEALTH_WDT_REASON_TIMER (0x00000001u)
#define SYS_HEALTH_WDT_REASON_FORCE (0x00000002u)
#define SYS_HEALTH_PSM_WDSEL        (0x40010000u + 0x08u)
#define SYS_HEALTH_PSM_WDSEL_ALL    (0x0001FFFFu)
#define SYS_HEALTH_PSM_WDSEL_ROSC   (0x00000001u)
#define SYS_HEALTH_PSM_WDSEL_XOSC   (0x00000002u)

/* RP2040 erratum E1: the counter decrements twice per 1 us tick, so the
   load is 2 x the delay in us (the SDK's WATCHDOG_XFACTOR).           */
#define SYS_HEALTH_WDT_XFACTOR  (2u)
#define SYS_HEALTH_LOAD_NORMAL  (SYS_HEALTH_WDT_MS * SYS_HEALTH_US_PER_MS \
                                 * SYS_HEALTH_WDT_XFACTOR)
#define SYS_HEALTH_LOAD_START   (SYS_HEALTH_WDT_START_MS                  \
                                 * SYS_HEALTH_US_PER_MS                   \
                                 * SYS_HEALTH_WDT_XFACTOR)
#define SYS_HEALTH_WDT_DBG_BITS (SYS_HEALTH_WDT_PAUSE_DBG0                \
                                 | SYS_HEALTH_WDT_PAUSE_DBG1              \
                                 | SYS_HEALTH_WDT_PAUSE_JTAG)
#define SYS_HEALTH_WDT_NONBOOT  (0x6ab73121u)  /* SDK: "not a reboot" */
#define SYS_HEALTH_SCRATCH_IDX  (4u)
#define SYS_HEALTH_REASON_BITS  (SYS_HEALTH_WDT_REASON_TIMER              \
                                 | SYS_HEALTH_WDT_REASON_FORCE)

#if (SYS_HEALTH_LOAD_START > SYS_HEALTH_WDT_LOAD_MASK)
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
static uint32_t g_boot_us        = 0u;   /* sys_health_enable_watchdog() */
/* Bring-up diagnosis: longest time between two normal-window feeds. The
   kernel tick is 10 ms (CNF_TIMER_PERIOD), so mission_task's 10 ms
   receive timeout can make passes up to ~20 ms apart against the 30 ms
   window; this measures the real margin on the car. */
static uint32_t g_last_feed_us    = 0u;
static uint32_t g_feed_gap_max_us = 0u;

/* ---- private function prototypes ------------------------------------- */

static void sys_health_wdt_start(uint32_t load);
static bool sys_health_radio_settled(void);

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
            (int32_t) (in_w(SYS_HEALTH_WDT_REASON)
                       & SYS_HEALTH_REASON_BITS), 0, 0);

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
            if (sys_health_radio_settled())
            {
                sys_health_wdt_start(SYS_HEALTH_LOAD_NORMAL); /* 30 ms */
                gb_is_started = true;
            }
            else
            {
                out_w(SYS_HEALTH_WDT_LOAD, g_wdt_load); /* feed 500 ms */
            }
            b_feed = true;
        }
    }
    else if (b_fast && (age_us <= limit_us))
    {
        uint32_t const now_us = sys_tick_now_us();

        out_w(SYS_HEALTH_WDT_LOAD, g_wdt_load);           /* feed */
        b_feed            = true;
        if ((0u != g_last_feed_us)
            && ((now_us - g_last_feed_us) > g_feed_gap_max_us))
        {
            g_feed_gap_max_us = now_us - g_last_feed_us;  /* wrap-safe */
        }
        g_last_feed_us = now_us;
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
    g_boot_us = sys_tick_now_us();
    sys_health_wdt_start(SYS_HEALTH_LOAD_START);
    gb_is_wdt_active = true;
}

/**
 * sys_health_feed_gap_max_us
 * @brief   Longest time between two feeds in the normal 30 ms window
 *          since the last reset of the figure (bring-up diagnosis).
 * @param   b_reset  true to start a new measurement after reading
 * @return  microseconds (0 before two normal feeds)
 * @context mission_task (the only caller of sys_health_service)
 */
uint32_t sys_health_feed_gap_max_us(bool b_reset)
{
    uint32_t const gap_us = g_feed_gap_max_us;

    if (b_reset)
    {
        g_feed_gap_max_us = 0u;
    }
    return gap_us;
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
    out_w(SYS_HEALTH_WDT_SCRATCH0
          + (SYS_HEALTH_WDT_REG_BYTES * SYS_HEALTH_SCRATCH_IDX),
          SYS_HEALTH_WDT_NONBOOT);
    clr_w(SYS_HEALTH_WDT_CTRL, SYS_HEALTH_WDT_CTRL_ENABLE);
    /* reset everything except the oscillators when it fires */
    set_w(SYS_HEALTH_PSM_WDSEL,
          SYS_HEALTH_PSM_WDSEL_ALL
          & ~(SYS_HEALTH_PSM_WDSEL_ROSC | SYS_HEALTH_PSM_WDSEL_XOSC));
    set_w(SYS_HEALTH_WDT_CTRL, SYS_HEALTH_WDT_DBG_BITS);
    out_w(SYS_HEALTH_WDT_LOAD, load);
    set_w(SYS_HEALTH_WDT_CTRL, SYS_HEALTH_WDT_CTRL_ENABLE);
}

/**
 * sys_health_radio_settled
 * @brief   True once the CYW43 task has finished its bring-up (success or
 *          failure, init_complete), or SYS_HEALTH_RADIO_WAIT_MS after BOOT
 *          so a radio that never reports cannot hold BOOT forever.
 * @return  true when the 30 ms window may be armed
 * @context mission_task
 */
static bool sys_health_radio_settled(void)
{
    bool b_is_settled = true;

#if defined(TM_WIFI_CYW43) && (TM_WIFI_CYW43 != 0)
    T_CYW43_UTK_STATUS status = {0};
    uint32_t const     age_us = sys_tick_now_us() - g_boot_us; /* wraps */

    cyw43_utk_get_status(&status);
    b_is_settled = (0u != status.init_complete)
                   || (age_us >= (SYS_HEALTH_RADIO_WAIT_MS
                                  * SYS_HEALTH_US_PER_MS));
#endif

    return b_is_settled;
}

/*** end of file ***/
