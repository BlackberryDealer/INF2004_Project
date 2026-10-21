/**
 * @file    mission.c
 * @brief   Mission queue K4, mode flags K10 and mission_task
 *          (Phase 0/1 subset: prints every event, services the watchdog).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    min-bit item 7 (queue messages), NFR10b
 */

#include "mission.h"           /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../subsystem2/fault.h"
#include "sys_health.h"
#include "drv_buzzer.h"
#include "drv_ws2812.h"

/* ---- private constants and types ------------------------------------ */

/* K4 baseline message (natural types): 12 B, 16 B per entry with the
   4-byte kernel header, 16 entries = 256 B.                          */
typedef struct
{
    uint32_t type;      /* APP_EVT_* */
    int32_t  arg_a;     /* event value */
    int32_t  arg_b;     /* spare (always 0) */
} mission_evt_t;

#define MISSION_K4_ENTRIES     (16u)
#define MISSION_EVT_MSG_BYTES  (12u)   /* sizeof(mission_evt_t) */
#define MISSION_MBF_HDR_BYTES  (4u)    /* kernel header per message */
#define MISSION_MBF_BYTES      (MISSION_K4_ENTRIES                     \
                                * (MISSION_MBF_HDR_BYTES               \
                                   + MISSION_EVT_MSG_BYTES))
#define MISSION_RCV_TMO_MS     (10)    /* one pass at least every 10 ms */
#define MISSION_STACK_WORDS    (APP_STACK_MISSION_BYTES / 8u)
#define MISSION_RGB_BOOT_LEVEL (32u)   /* dim white at boot */

/* ---- private data ---------------------------------------------------- */

static uint32_t g_mission_mbf_buf[MISSION_MBF_BYTES / 4u];
static uint64_t g_mission_stack[MISSION_STACK_WORDS];

static ID gh_mission_mbf  = 0;
static ID gh_mode_flg     = 0;
static ID gh_mission_task = 0;

/* Statistic only: several producers may update it, and a pre-empted
   update can at worst record a slightly smaller value.               */
static volatile uint32_t g_queue_high_water = 0u;

/* ---- private function prototypes ------------------------------------- */

static void mission_task(INT stacd, void * p_exinf);
static void mission_note_high_water(void);

/* ---- public functions ------------------------------------------------- */

/**
 * mission_init
 * @brief   Creates K4 MBF_MISSION_EVT and K10 FLG_MODE.
 * @return  E_OK, or the first kernel error
 * @context usermain (init phase)
 */
ER mission_init(void)
{
    T_CMBF cmbf = {0};
    T_CFLG cflg = {0};
    ER     ercd = E_OK;

    cmbf.mbfatr    = TA_TFIFO | TA_USERBUF;
    cmbf.bufsz     = (SZ) MISSION_MBF_BYTES;      /* cast: kernel size */
    cmbf.maxmsz    = (INT) MISSION_EVT_MSG_BYTES; /* cast: kernel size */
    cmbf.bufptr    = g_mission_mbf_buf;
    gh_mission_mbf = tk_cre_mbf(&cmbf);

    if (gh_mission_mbf < E_OK)
    {
        ercd = (ER) gh_mission_mbf;  /* cast: negative ID is an error */
    }
    else
    {
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0u;
        gh_mode_flg  = tk_cre_flg(&cflg);
        if (gh_mode_flg < E_OK)
        {
            ercd = (ER) gh_mode_flg;  /* cast: negative ID is an error */
        }
    }

    return ercd;
}

/**
 * mission_start
 * @brief   Creates and starts mission_task on its static stack.
 * @return  E_OK, or the kernel error
 * @context usermain (start phase, called last)
 */
ER mission_start(void)
{
    ER ercd = E_OK;

    gh_mission_task = app_start_task((FP) mission_task, /* cast: kernel */
                                     APP_PRI_MISSION, g_mission_stack,
                                     APP_STACK_MISSION_BYTES,
                                     "mission_task");
    if (gh_mission_task < E_OK)
    {
        ercd = (ER) gh_mission_task;  /* cast: negative ID is an error */
    }

    return ercd;
}

/**
 * mission_post_event
 * @brief   Queues one event for mission_task without ever blocking.
 * @param   type  APP_EVT_*
 * @param   value event value (D.13)
 * @return  true if queued; false for an unknown type or a full K4
 *          (a full K4 also sets APP_FAULT_QUEUE_OVERFLOW)
 * @context any task, including the radio task (never an ISR)
 */
bool mission_post_event(uint32_t type, uint32_t value)
{
    mission_evt_t msg       = {0u, 0, 0};
    bool          b_is_sent = false;

    if (type < APP_EVT_COUNT)
    {
        msg.type  = type;
        msg.arg_a = (int32_t) value;  /* cast: values < 2048 (D.13) */
        if (E_OK == tk_snd_mbf(gh_mission_mbf, &msg,
                               (INT) MISSION_EVT_MSG_BYTES, /* cast: size */
                               TMO_POL))
        {
            b_is_sent = true;
            mission_note_high_water();
        }
        else
        {
            fault_set(APP_FAULT_QUEUE_OVERFLOW);
        }
    }

    return b_is_sent;
}

/**
 * mission_evt_barcode_value
 * @brief   Packs a BARCODE / MAP event value.
 * @param   cmd        APP_BAR_CMD_* (bits 6-7)
 * @param   char_index Code 39 index 0..42 (bits 0-5)
 * @return  the 8-bit event value
 * @context any task
 */
uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    return ((char_index & 0x3Fu) | ((cmd & 0x3u) << 6u));
}

/**
 * mission_mode_set
 * @brief   Sets mode request bits in FLG_MODE (K10).
 * @param   bits APP_MODE_* mask
 * @context mission_task only
 */
void mission_mode_set(uint32_t bits)
{
    /* set cannot fail with a valid ID */
    (void) tk_set_flg(gh_mode_flg,
                      (UINT) (bits & APP_MODE_ALL)); /* cast: kernel */
}

/**
 * mission_mode_clear
 * @brief   Clears mode request bits in FLG_MODE (K10).
 * @param   bits APP_MODE_* mask
 * @context mission_task only
 */
void mission_mode_clear(uint32_t bits)
{
    /* tk_clr_flg ANDs: pass the inverted mask */
    (void) tk_clr_flg(gh_mode_flg,
                      (UINT) ~(bits & APP_MODE_ALL)); /* cast: kernel */
}

/**
 * mission_mode_is_set
 * @brief   Tests one mode request bit.
 * @param   bit APP_MODE_* (one bit)
 * @return  true if the bit is set
 * @context any task
 */
bool mission_mode_is_set(uint32_t bit)
{
    T_RFLG rflg     = {0};
    bool   b_is_set = false;

    if (E_OK == tk_ref_flg(gh_mode_flg, &rflg))
    {
        b_is_set = (0u != (((uint32_t) rflg.flgptn) & bit));
    }

    return b_is_set;
}

/**
 * mission_queue_high_water
 * @brief   Most K4 buffer bytes ever in use.
 * @return  bytes (each queued event costs 16 B in the baseline)
 * @context any task
 */
uint32_t mission_queue_high_water(void)
{
    return g_queue_high_water;
}

/* ---- private functions ------------------------------------------------ */

/**
 * mission_task
 * @brief   Phase 1 body: sets up buzzer and RGB, then prints each event
 *          and services the watchdog.
 * @param   stacd   unused (kernel start code)
 * @param   p_exinf unused
 * @context mission_task (priority 16)
 */
static void mission_task(INT stacd, void * p_exinf)
{
    mission_evt_t msg  = {0u, 0, 0};
    INT           size = 0;

    (void) stacd;    /* unused: kernel start code */
    (void) p_exinf;  /* unused: no extended info  */

    drv_buzzer_init();
    if (drv_ws2812_init())   /* false: runs on without RGB (fault set) */
    {
        drv_ws2812_set(MISSION_RGB_BOOT_LEVEL, MISSION_RGB_BOOT_LEVEL,
                       MISSION_RGB_BOOT_LEVEL);   /* white: booted */
    }

    for (;;)
    {
        size = tk_rcv_mbf(gh_mission_mbf, &msg, MISSION_RCV_TMO_MS);
        if (size > 0)
        {
            app_log("EVT %d %d\n", (int32_t) msg.type, /* cast: < 24 */
                    msg.arg_a, 0);
        }
        sys_health_service();
    }
}

/**
 * mission_note_high_water
 * @brief   Updates the K4 high-water mark after a successful send.
 * @context any posting task
 */
static void mission_note_high_water(void)
{
    T_RMBF   rmbf     = {0};
    uint32_t used     = 0u;

    if (E_OK == tk_ref_mbf(gh_mission_mbf, &rmbf))
    {
        used = MISSION_MBF_BYTES
               - (uint32_t) rmbf.frbufsz;  /* cast: 0..bufsz */
        if (used > g_queue_high_water)
        {
            g_queue_high_water = used;
        }
    }
}

/*** end of file ***/
