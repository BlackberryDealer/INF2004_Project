/**
 * @file    hmi.c
 * @brief   Buttons (debounce, 2 s RESET hold), RGB colour per system
 *          state and buzzer patterns. Runs inside mission_task.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 * @reqs    FR11 (buttons), FR16 (LED, buzzer)
 */

#include "hmi.h"               /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "../app_types.h"
#include "cfg_mission.h"
#include "drv_buzzer.h"
#include "drv_ws2812.h"
#include "hardware/gpio.h"

/* ---- private constants and types ------------------------------------ */

#define HMI_LED_OFF      (0u)
#define HMI_LED_WHITE    (1u)
#define HMI_LED_YELLOW   (2u)
#define HMI_LED_GREEN    (3u)
#define HMI_LED_BLUE     (4u)
#define HMI_LED_RED      (5u)
#define HMI_LED_COUNT    (6u)
#define HMI_LED_UNKNOWN  (0xFFu)       /* forces the first write */

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

#define HMI_STATE_NONE   (0xFFu)

/* ---- private data ---------------------------------------------------- */

/* GRB levels per colour code: {red, green, blue} */
static uint8_t const g_hmi_rgb[HMI_LED_COUNT][3] =
{
    {0u, 0u, 0u},                           /* off    */
    {HMI_LEVEL, HMI_LEVEL, HMI_LEVEL},      /* white  */
    {HMI_LEVEL, HMI_LEVEL, 0u},             /* yellow */
    {0u, HMI_LEVEL, 0u},                    /* green  */
    {0u, 0u, HMI_LEVEL},                    /* blue   */
    {HMI_LEVEL, 0u, 0u},                    /* red    */
};

/* All written only by mission_task. */
static uint32_t g_run_cnt       = 0u;
static uint32_t g_reset_cnt     = 0u;
static bool     gb_reset_sent   = false;
static bool     gb_is_warning   = false;
static uint32_t g_led_code      = HMI_LED_UNKNOWN;
static bool     gb_is_buzz_on   = false;
static uint32_t g_prev_state    = HMI_STATE_NONE;
static uint32_t g_entry_us      = 0u;
static uint32_t g_beep_kind     = HMI_BEEP_NONE;
static uint32_t g_beep_start_us = 0u;

/* ---- private function prototypes ------------------------------------- */

static uint32_t hmi_colour(uint32_t sys_state, bool b_is_blink_on);
static bool     hmi_buzz(uint32_t sys_state, uint32_t now_us,
                         uint32_t in_state_us);
static bool     hmi_is_pressed(uint32_t pin);

/* ---- public functions ------------------------------------------------- */

/**
 * hmi_init
 * @brief   Buttons as inputs with pull-ups, buzzer silent, RGB loaded
 *          (runs on without RGB if no PIO space is left).
 * @context mission_task, before its loop
 */
void hmi_init(void)
{
    gpio_init(PIN_BTN_RUN);
    gpio_set_dir(PIN_BTN_RUN, false);
    gpio_pull_up(PIN_BTN_RUN);
    gpio_init(PIN_BTN_RESET);
    gpio_set_dir(PIN_BTN_RESET, false);
    gpio_pull_up(PIN_BTN_RESET);
    drv_buzzer_init();
    (void) drv_ws2812_init();   /* false: fault set inside, no RGB */
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
    uint32_t result = HMI_NONE;

    if (hmi_is_pressed(PIN_BTN_RUN))
    {
        if (g_run_cnt < MISSION_BTN_DEBOUNCE_TICKS)
        {
            g_run_cnt++;
        }
    }
    else
    {
        if (g_run_cnt >= MISSION_BTN_DEBOUNCE_TICKS)
        {
            result = HMI_RUN_PRESS;
        }
        g_run_cnt = 0u;
    }

    if (hmi_is_pressed(PIN_BTN_RESET))
    {
        if (g_reset_cnt < MISSION_BTN_RESET_HOLD_TICKS)
        {
            g_reset_cnt++;
        }
        if ((g_reset_cnt >= MISSION_BTN_RESET_HOLD_TICKS) && (!gb_reset_sent))
        {
            gb_reset_sent = true;
            result        = HMI_RESET_HOLD;   /* wins over a RUN press */
        }
    }
    else
    {
        g_reset_cnt   = 0u;
        gb_reset_sent = false;
    }

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

    if (sys_state != g_prev_state)
    {
        g_prev_state = sys_state;
        g_entry_us   = now_us;
        if (APP_SYS_BOOT == sys_state)
        {
            g_beep_kind     = HMI_BEEP_ONE;
            g_beep_start_us = now_us;
        }
        else if (APP_SYS_FINISHED == sys_state)
        {
            g_beep_kind     = HMI_BEEP_TWO;
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
    if (code != g_led_code)
    {
        g_led_code = code;
        drv_ws2812_set(g_hmi_rgb[code][0], g_hmi_rgb[code][1],
                       g_hmi_rgb[code][2]);
    }

    b_is_buzz = hmi_buzz(sys_state, now_us, in_state_us);
    if (b_is_buzz != gb_is_buzz_on)
    {
        gb_is_buzz_on = b_is_buzz;
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
    gb_is_warning = b_is_on;
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
            if (gb_is_warning)
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
    bool           b_is_buzz = false;

    if (APP_SYS_FAULT == sys_state)
    {
        b_is_buzz = (0u == ((in_state_us >> HMI_ALARM_SHIFT) & 1u));
    }
    else if ((APP_SYS_READY == sys_state) && gb_is_warning)
    {
        b_is_buzz = true;
    }
    else if (HMI_BEEP_ONE == g_beep_kind)
    {
        b_is_buzz = (beep_us < HMI_BEEP_US);
    }
    else if (HMI_BEEP_TWO == g_beep_kind)
    {
        b_is_buzz = (beep_us < HMI_BEEP_US)
                    || ((beep_us >= HMI_BEEP2_ON_US)
                        && (beep_us < HMI_BEEP2_OFF_US));
    }
    else
    {
        b_is_buzz = false;
    }

    if ((HMI_BEEP_NONE != g_beep_kind) && (beep_us >= HMI_BEEP2_OFF_US))
    {
        g_beep_kind = HMI_BEEP_NONE;      /* pattern finished */
    }

    return b_is_buzz;
}

/**
 * hmi_is_pressed
 * @brief   Robo Pico buttons pull the pin to GND: pressed reads 0
 *          (MEASURE: confirm on the board).
 * @param   pin PIN_BTN_RUN or PIN_BTN_RESET
 * @return  true while pressed
 * @context mission_task
 */
static bool hmi_is_pressed(uint32_t pin)
{
    return !gpio_get(pin);
}

/*** end of file ***/
