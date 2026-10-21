/**
 * @file    sys_health.c
 * @brief   K8 FLG_ALIVE: periodic tasks check in, mission feeds the
 *          watchdog only when all four have run.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    NFR10b (watchdog 100 ms)
 *
 * The SDK's watchdog.c does not build in this template (it pulls in
 * pico/bootrom.h -> pico/flash.h, which the template lacks), so the
 * watchdog is driven through its registers exactly as the SDK's
 * _watchdog_enable() does for the RP2040 (B5.0.3 fallback).
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

/* ---- private constants and types ------------------------------------ */

#define SYS_HEALTH_WDT_MS   (100u)   /* NFR10b */

/* RP2040 erratum E1: the counter decrements twice per 1 us tick, so the
   load is 2 x the delay in us (the SDK's WATCHDOG_XFACTOR).           */
#define SYS_HEALTH_WDT_XFACTOR  (2u)
#define SYS_HEALTH_WDT_LOAD     (SYS_HEALTH_WDT_MS * 1000u               \
                                 * SYS_HEALTH_WDT_XFACTOR)
#define SYS_HEALTH_WDT_DBG_BITS (WATCHDOG_CTRL_PAUSE_DBG0_BITS           \
                                 | WATCHDOG_CTRL_PAUSE_DBG1_BITS         \
                                 | WATCHDOG_CTRL_PAUSE_JTAG_BITS)
#define SYS_HEALTH_WDT_NONBOOT  (0x6ab73121u)  /* SDK: "not a reboot" */
#define SYS_HEALTH_SCRATCH_IDX  (4u)

#if (SYS_HEALTH_WDT_LOAD > WATCHDOG_LOAD_BITS)
#error "Watchdog delay too long for the 24-bit LOAD field"
#endif

/* ---- private data ---------------------------------------------------- */

static ID   gh_alive_flg     = 0;
static bool gb_is_wdt_active = false;  /* written only by mission_task */

/* ---- private function prototypes ------------------------------------- */

/* ---- public functions ------------------------------------------------- */

/**
 * sys_health_init
 * @brief   Creates FLG_ALIVE (K8) and the alive LED output.
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

    return ercd;
}

/**
 * sys_health_check_in
 * @brief   Marks one periodic task as alive for this round.
 * @param   task_bit SYS_HEALTH_BIT_*
 * @context line_task, motion_task, imu_task, obstacle_task
 */
void sys_health_check_in(uint32_t task_bit)
{
    /* set cannot fail with a valid ID; cast: kernel pattern type */
    (void) tk_set_flg(gh_alive_flg, (UINT) (task_bit & SYS_HEALTH_BIT_ALL));
}

/**
 * sys_health_service
 * @brief   If all four tasks checked in: feed the watchdog (once enabled),
 *          clear the bits and toggle the alive LED.
 * @context mission_task, every pass
 */
void sys_health_service(void)
{
    T_RFLG rflg = {0};

    if (E_OK == tk_ref_flg(gh_alive_flg, &rflg))
    {
        /* cast: kernel pattern to fixed width */
        if (SYS_HEALTH_BIT_ALL
            == (((uint32_t) rflg.flgptn) & SYS_HEALTH_BIT_ALL))
        {
            if (gb_is_wdt_active)
            {
                watchdog_hw->load = SYS_HEALTH_WDT_LOAD;   /* feed */
            }
            /* tk_clr_flg ANDs: pass the inverted mask */
            (void) tk_clr_flg(gh_alive_flg,
                              (UINT) ~SYS_HEALTH_BIT_ALL); /* cast: kernel */
            drv_led_alive_toggle();
        }
    }
}

/**
 * sys_health_enable_watchdog
 * @brief   Starts the 100 ms hardware watchdog (paused while debugging).
 *          Same sequence of writes as the SDK's watchdog_enable(100, true).
 *          Relies on the watchdog tick running at 1 MHz, which the 1 MHz
 *          system timer (sys_tick_now_us) also needs.
 * @context mission_task (BOOT entry)
 */
void sys_health_enable_watchdog(void)
{
    watchdog_hw->scratch[SYS_HEALTH_SCRATCH_IDX] = SYS_HEALTH_WDT_NONBOOT;
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    /* reset everything except the oscillators when it fires */
    hw_set_bits(&psm_hw->wdsel,
                PSM_WDSEL_BITS
                & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
    hw_set_bits(&watchdog_hw->ctrl, SYS_HEALTH_WDT_DBG_BITS);
    watchdog_hw->load = SYS_HEALTH_WDT_LOAD;
    hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    gb_is_wdt_active = true;
}

/* ---- private functions ------------------------------------------------ */

/*** end of file ***/
