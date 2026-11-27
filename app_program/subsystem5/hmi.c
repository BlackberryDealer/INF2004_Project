/**
 * @file    hmi.c
 * @brief   Buttons (debounce, 2 s RESET hold), RGB colour per system
 *          state and buzzer patterns. Runs inside mission_task.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR11 (buttons), FR16 (LED, buzzer)
 */

#include "hmi.h"               /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "cfg_mission.h"
#include "drv_button.h"
#include "drv_buzzer.h"
#include "drv_ws2812.h"

/* ---- private constants and types ------------------------------------ */

#define HMI_LED_OFF      (0u)
#define HMI_LED_WHITE    (1u)
#define HMI_LED_YELLOW   (2u)
#define HMI_LED_GREEN    (3u)
#define HMI_LED_BLUE     (4u)
#define HMI_LED_RED      (5u)
#define HMI_LED_COUNT    (6u)
#define HMI_LED_UNKNOWN  (HMI_LED_COUNT) /* forces the first write */

#define HMI_LEVEL        (32u)         /* brightness, 0..255 */

/* Blink and beep timing with shifts instead of division (10 ms path):
   2^18 us = 262 ms half-period (~1.9 Hz blink, the "2 Hz" of B5.2.7);
   2^19 us = 524 ms for the FAULT alarm's ~500 ms on/off.            */
#define HMI_BLINK_SHIFT  (18u)
#define HMI_ALARM_SHIFT  (19u)
#define HMI_BEEP_US      (100000u)     /* one beep: 100 ms        */
#define HMI_BEEP2_ON_US  (200000u)     /* second beep starts      */
#define HMI_BEEP2_OFF_US (300000u)     /* second beep ends        */

#define HMI_BEEP_NONE    (0u)
#define HMI_BEEP_ONE     (1u)
#define HMI_BEEP_TWO     (2u)

#define HMI_STATE_NONE   (APP_SYS_FAULT + 1u) /* no state shown yet */

/* PRV_MIS_G rows must hold every code and count stored here. */
#if (HMI_LED_UNKNOWN != PRV_MIS_G_LED_MAX)                              \
    || (HMI_STATE_NONE != PRV_MIS_G_PREV_SYS_MAX)                       \
    || (HMI_BEEP_TWO != PRV_MIS_G_BEEP_MAX)                             \
    || (MISSION_BTN_DEBOUNCE_TICKS > PRV_MIS_G_BTN_RUN_MAX)             \
    || (MISSION_BTN_RESET_HOLD_TICKS != PRV_MIS_G_BTN_RESET_MAX)
#error "A PRV_MIS_G row no longer covers its HMI code or count"
#endif

/* ---- private data ---------------------------------------------------- */

/* RGB levels per colour code: {red, green, blue} */
static uint8_t const g_hmi_rgb[HMI_LED_COUNT][3] =
{
    {0u, 0u, 0u},                           /* off    */
    {HMI_LEVEL, HMI_LEVEL, HMI_LEVEL},      /* white  */
    {HMI_LEVEL, HMI_LEVEL, 0u},             /* yellow */
    {0u, HMI_LEVEL, 0u},                    /* green  */
    {0u, 0u, HMI_LEVEL},                    /* blue   */
    {HMI_LEVEL, 0u, 0u},                    /* red    */
};

/* All written only by mission_task. PRV_MIS_G: BTN_RUN, BTN_RESET,
   RESET_SENT, WARNING, LED (starts unknown), BUZZ, PREV_SYS (starts
   none), BEEP. The two times are E5 timer images (E_MIS_US).        */
/* casts: PRV_MIS_G LED (0..6) and PREV_SYS (0..8) are positive codes */
static uint32_t g_hmi_word =
    ((uint32_t) HMI_LED_UNKNOWN << PRV_MIS_G_LED_POS)
    | ((uint32_t) HMI_STATE_NONE << PRV_MIS_G_PREV_SYS_POS);
static uint32_t g_entry_us      = 0u;
static uint32_t g_beep_start_us = 0u;

/* ---- private function prototypes ------------------------------------- */

static uint32_t hmi_colour(uint32_t sys_state, bool b_is_blink_on);
static bool     hmi_buzz(uint32_t sys_state, uint32_t now_us,
                         uint32_t in_state_us);
static uint32_t hmi_get(uint32_t pos, uint32_t width);
static void     hmi_put(uint32_t pos, uint32_t width, uint32_t value);
static bool     hmi_is_warning(void);

/* ---- public functions ------------------------------------------------- */

/**
 * hmi_init
 * @brief   Buttons as inputs with pull-ups, buzzer silent, RGB loaded
 *          (runs on without RGB if no PIO space is left).
 * @return  true if the RGB LED is usable; false: mission_task raises
 *          APP_FAULT_PIO_LOAD
 * @context mission_task, before its loop
 */
bool hmi_init(void)
{
    bool b_is_rgb_ok = false;

    drv_button_init();
    drv_buzzer_init();
    b_is_rgb_ok = drv_ws2812_init();   /* false: no PIO space, no RGB */

    return b_is_rgb_ok;
}

/**
 * hmi_poll
 * @brief   Debounces both buttons; call every 10 ms.
 * @return  HMI_NONE, HMI_RUN_PRESS (released after >= 3 stable pressed
 *          polls) or HMI_RESET_HOLD (once per hold, after 200 polls)
 * @context mission_task
 */
uint32_t hmi_poll(void)
{
    uint32_t result    = HMI_NONE;
    uint32_t run_cnt   = hmi_get(PRV_MIS_G_BTN_RUN_POS,
                                 PRV_MIS_G_BTN_RUN_BITS);
    uint32_t reset_cnt = hmi_get(PRV_MIS_G_BTN_RESET_POS,
                                 PRV_MIS_G_BTN_RESET_BITS);
    bool     b_sent    = (0u != hmi_get(PRV_MIS_G_RESET_SENT_POS,
                                        PRV_MIS_G_RESET_SENT_BITS));

    if (drv_button_run_pressed())
    {
        if (run_cnt < MISSION_BTN_DEBOUNCE_TICKS)
        {
            run_cnt++;
        }
    }
    else
    {
        if (run_cnt >= MISSION_BTN_DEBOUNCE_TICKS)
        {
            result = HMI_RUN_PRESS;
        }
        run_cnt = 0u;
    }

    if (drv_button_reset_pressed())
    {
        if (reset_cnt < MISSION_BTN_RESET_HOLD_TICKS)
        {
            reset_cnt++;
        }
        if ((reset_cnt >= MISSION_BTN_RESET_HOLD_TICKS) && (!b_sent))
        {
            b_sent = true;
            result = HMI_RESET_HOLD;      /* wins over a RUN press */
        }
    }
    else
    {
        reset_cnt = 0u;
        b_sent    = false;
    }

    hmi_put(PRV_MIS_G_BTN_RUN_POS, PRV_MIS_G_BTN_RUN_BITS, run_cnt);
    hmi_put(PRV_MIS_G_BTN_RESET_POS, PRV_MIS_G_BTN_RESET_BITS, reset_cnt);
    hmi_put(PRV_MIS_G_RESET_SENT_POS, PRV_MIS_G_RESET_SENT_BITS,
            b_sent ? 1u : 0u);

    return result;
}

/**
 * hmi_show
 * @brief   Sets the RGB colour and buzzer for the current state. The LED
 *          is written only when its colour changes.
 * @param   sys_state APP_SYS_*
 * @param   now_us    sys_tick_now_us() of this pass
 * @context mission_task, every pass
 */
void hmi_show(uint32_t sys_state, uint32_t now_us)
{
    uint32_t in_state_us = 0u;
    uint32_t code        = HMI_LED_OFF;
    bool     b_is_buzz   = false;

    if (sys_state != hmi_get(PRV_MIS_G_PREV_SYS_POS, PRV_MIS_G_PREV_SYS_BITS))
    {
        hmi_put(PRV_MIS_G_PREV_SYS_POS, PRV_MIS_G_PREV_SYS_BITS, sys_state);
        g_entry_us   = now_us;
        if (APP_SYS_BOOT == sys_state)
        {
            hmi_put(PRV_MIS_G_BEEP_POS, PRV_MIS_G_BEEP_BITS, HMI_BEEP_ONE);
            g_beep_start_us = now_us;
        }
        else if (APP_SYS_FINISHED == sys_state)
        {
            hmi_put(PRV_MIS_G_BEEP_POS, PRV_MIS_G_BEEP_BITS, HMI_BEEP_TWO);
            g_beep_start_us = now_us;
        }
        else
        {
            /* other states start no beep pattern */
        }
    }
    in_state_us = now_us - g_entry_us;     /* unsigned: wrap-safe */

    code = hmi_colour(sys_state,
                      0u == ((in_state_us >> HMI_BLINK_SHIFT) & 1u));
    if (code != hmi_get(PRV_MIS_G_LED_POS, PRV_MIS_G_LED_BITS))
    {
        hmi_put(PRV_MIS_G_LED_POS, PRV_MIS_G_LED_BITS, code);
        drv_ws2812_set(g_hmi_rgb[code][0], g_hmi_rgb[code][1],
                       g_hmi_rgb[code][2]);
    }

    b_is_buzz = hmi_buzz(sys_state, now_us, in_state_us);
    if (b_is_buzz != (0u != hmi_get(PRV_MIS_G_BUZZ_POS, PRV_MIS_G_BUZZ_BITS)))
    {
        hmi_put(PRV_MIS_G_BUZZ_POS, PRV_MIS_G_BUZZ_BITS, b_is_buzz ? 1u : 0u);
        if (b_is_buzz)
        {
            drv_buzzer_on();
        }
        else
        {
            drv_buzzer_off();
        }
    }
}

/**
 * hmi_set_warning
 * @brief   Turns the 1 s start warning (buzzer on, LED flashing blue) on
 *          or off. See SPEC-QUESTION Q4 in hmi.h.
 * @param   b_is_on true from START until RUN begins
 * @context mission_task
 */
void hmi_set_warning(bool b_is_on)
{
    hmi_put(PRV_MIS_G_WARNING_POS, PRV_MIS_G_WARNING_BITS, b_is_on ? 1u : 0u);
}

/* ---- private functions ------------------------------------------------ */

/**
 * hmi_colour
 * @brief   Colour code for a state (B5.2.7 table).
 * @param   sys_state     APP_SYS_*
 * @param   b_is_blink_on true in the "on" half of the blink period
 * @return  HMI_LED_*
 * @context mission_task
 */
static uint32_t hmi_colour(uint32_t sys_state, bool b_is_blink_on)
{
    uint32_t code = HMI_LED_RED;

    switch (sys_state)
    {
        case APP_SYS_BOOT:
            code = HMI_LED_WHITE;
            break;
        case APP_SYS_CAL_IMU:
        case APP_SYS_CAL_IR:
            code = b_is_blink_on ? HMI_LED_YELLOW : HMI_LED_OFF;
            break;
        case APP_SYS_READY:
            if (hmi_is_warning())
            {
                code = b_is_blink_on ? HMI_LED_BLUE : HMI_LED_OFF;
            }
            else
            {
                code = HMI_LED_GREEN;
            }
            break;
        case APP_SYS_RUN:
            code = HMI_LED_BLUE;
            break;
        case APP_SYS_FINISHED:
            code = b_is_blink_on ? HMI_LED_GREEN : HMI_LED_OFF;
            break;
        case APP_SYS_LINE_LOST_STOP:
            code = b_is_blink_on ? HMI_LED_BLUE : HMI_LED_OFF;
            break;
        case APP_SYS_FAULT:
            code = b_is_blink_on ? HMI_LED_RED : HMI_LED_OFF;
            break;
        default:
            code = HMI_LED_RED;
            break;
    }

    return code;
}

/**
 * hmi_buzz
 * @brief   Whether the buzzer should sound now: boot beep (100 ms), start
 *          warning (continuous), two beeps on FINISHED, FAULT alarm.
 * @param   sys_state   APP_SYS_*
 * @param   now_us      this pass
 * @param   in_state_us time since the state was entered
 * @return  true = on
 * @context mission_task
 */
static bool hmi_buzz(uint32_t sys_state, uint32_t now_us,
                     uint32_t in_state_us)
{
    uint32_t const beep_us   = now_us - g_beep_start_us;  /* wrap-safe */
    uint32_t const beep_kind = hmi_get(PRV_MIS_G_BEEP_POS,
                                       PRV_MIS_G_BEEP_BITS);
    bool           b_is_buzz = false;

    if (APP_SYS_FAULT == sys_state)
    {
        b_is_buzz = (0u == ((in_state_us >> HMI_ALARM_SHIFT) & 1u));
    }
    else if ((APP_SYS_READY == sys_state) && hmi_is_warning())
    {
        b_is_buzz = true;
    }
    else if (HMI_BEEP_ONE == beep_kind)
    {
        b_is_buzz = (beep_us < HMI_BEEP_US);
    }
    else if (HMI_BEEP_TWO == beep_kind)
    {
        b_is_buzz = (beep_us < HMI_BEEP_US)
                    || ((beep_us >= HMI_BEEP2_ON_US)
                        && (beep_us < HMI_BEEP2_OFF_US));
    }
    else
    {
        b_is_buzz = false;
    }

    if ((HMI_BEEP_NONE != beep_kind) && (beep_us >= HMI_BEEP2_OFF_US))
    {
        /* pattern finished */
        hmi_put(PRV_MIS_G_BEEP_POS, PRV_MIS_G_BEEP_BITS, HMI_BEEP_NONE);
    }

    return b_is_buzz;
}

/**
 * hmi_get / hmi_put
 * @brief   One field of the packed HMI word (PRV_MIS_G).
 * @param   pos    field position (PRV_MIS_G_*_POS)
 * @param   width  field width (PRV_MIS_G_*_BITS)
 * @param   value  new value, inside the row range (put only)
 * @return  field value (get only)
 * @context mission_task
 */
static uint32_t hmi_get(uint32_t pos, uint32_t width)
{
    return app_bits_get(g_hmi_word, pos, width);
}

static void hmi_put(uint32_t pos, uint32_t width, uint32_t value)
{
    g_hmi_word = app_bits_put(g_hmi_word, pos, width, value);
}

/**
 * hmi_is_warning
 * @brief   The 1 s start warning flag (PRV_MIS_G WARNING).
 * @return  true while the warning is on
 * @context mission_task
 */
static bool hmi_is_warning(void)
{
    return (0u != hmi_get(PRV_MIS_G_WARNING_POS, PRV_MIS_G_WARNING_BITS));
}

/*** end of file ***/
