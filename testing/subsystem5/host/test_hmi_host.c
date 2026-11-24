/*
 * test_hmi_host.c
 * Host unit tests for app_program/subsystem5/hmi.c with mocked button,
 * buzzer and WS2812 drivers (no Pico SDK or kernel headers).
 * Verifies button debounce, RUN-on-release, 2-second RESET hold, priority
 * when both buttons act together, LED state colours/blinking and buzzer
 * patterns. Host-only test code, outside BARR-C scope.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "host_test.h"
#include "app_types.h"
#include "subsystem5/cfg_mission.h"
#include "subsystem5/hmi.h"
#include "subsystem5/drv_button.h"
#include "subsystem5/drv_buzzer.h"
#include "subsystem5/drv_ws2812.h"

/* ---- hardware driver mocks ------------------------------------------- */

static bool     gb_run_down      = false;
static bool     gb_reset_down    = false;
static bool     gb_rgb_ok        = true;
static uint32_t g_btn_init_cnt   = 0u;
static uint32_t g_buzz_init_cnt  = 0u;
static uint32_t g_rgb_init_cnt   = 0u;
static bool     gb_buzz_on       = false;
static uint32_t g_buzz_changes   = 0u;
static uint32_t g_rgb_r          = 0u;
static uint32_t g_rgb_g          = 0u;
static uint32_t g_rgb_b          = 0u;
static uint32_t g_rgb_writes     = 0u;

void drv_button_init(void) { g_btn_init_cnt++; }
bool drv_button_run_pressed(void) { return gb_run_down; }
bool drv_button_reset_pressed(void) { return gb_reset_down; }

void drv_buzzer_init(void) { g_buzz_init_cnt++; gb_buzz_on = false; }
void drv_buzzer_on(void) { gb_buzz_on = true; g_buzz_changes++; }
void drv_buzzer_off(void) { gb_buzz_on = false; g_buzz_changes++; }

bool drv_ws2812_init(void) { g_rgb_init_cnt++; return gb_rgb_ok; }
void drv_ws2812_set(uint32_t r, uint32_t g, uint32_t b)
{
    g_rgb_r = r;
    g_rgb_g = g;
    g_rgb_b = b;
    g_rgb_writes++;
}

static void release_both(void)
{
    gb_run_down   = false;
    gb_reset_down = false;
    (void) hmi_poll();
    (void) hmi_poll();
}

int main(void)
{
    HOST_INIT();

    /* 1. Driver initialisation and RGB status propagation */
    gb_rgb_ok = false;
    HOST_CHECK("S5-U85", !hmi_init() && (1u == g_btn_init_cnt)
               && (1u == g_buzz_init_cnt) && (1u == g_rgb_init_cnt),
               "hmi_init calls button/buzzer/ws2812 init and returns false when RGB fails");
    gb_rgb_ok = true;
    HOST_CHECK("S5-U85", hmi_init(),
               "hmi_init returns true when WS2812 loads");

    /* 2. Contact bounce (< MISSION_BTN_DEBOUNCE_TICKS) is ignored */
    {
        bool b_ok = true;
        release_both();
        gb_run_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_DEBOUNCE_TICKS - 1u); i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        gb_run_down = false;
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        /* Bouncing press: 2 down, 1 up, 2 down, 1 up -> never reaches 3 */
        gb_run_down = true;
        b_ok = b_ok && (HMI_NONE == hmi_poll()) && (HMI_NONE == hmi_poll());
        gb_run_down = false;
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        gb_run_down = true;
        b_ok = b_ok && (HMI_NONE == hmi_poll()) && (HMI_NONE == hmi_poll());
        gb_run_down = false;
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        HOST_CHECK("S5-U85", b_ok,
                   "short glitches and bouncing RUN presses produce HMI_NONE");
    }

    /* 3. RUN fires once on release after >= 3 stable polls */
    {
        bool b_ok = true;
        release_both();
        gb_run_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_DEBOUNCE_TICKS + 3u); i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        gb_run_down = false;
        b_ok = b_ok && (HMI_RUN_PRESS == hmi_poll());
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        HOST_CHECK("S5-U85", b_ok,
                   "stable RUN press fires HMI_RUN_PRESS once on release");
    }

    /* 4. RESET held < 2 s (199 ticks) does not fire */
    {
        bool b_ok = true;
        release_both();
        gb_reset_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_RESET_HOLD_TICKS - 1u); i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        gb_reset_down = false;
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        HOST_CHECK("S5-U85", b_ok,
                   "RESET released after 199 ticks (< 2 s) produces HMI_NONE");
    }

    /* 5. RESET held 200 ticks (2 s) fires once per hold, re-arms after release */
    {
        bool b_ok = true;
        release_both();
        gb_reset_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_RESET_HOLD_TICKS - 1u); i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        b_ok = b_ok && (HMI_RESET_HOLD == hmi_poll());
        /* Holding longer must not re-fire */
        for (uint32_t i = 0u; i < 10u; i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        /* Release and hold another 200 ticks -> fires again */
        gb_reset_down = false;
        b_ok = b_ok && (HMI_NONE == hmi_poll());
        gb_reset_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_RESET_HOLD_TICKS - 1u); i++)
        {
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        b_ok = b_ok && (HMI_RESET_HOLD == hmi_poll());
        release_both();
        HOST_CHECK("S5-U85", b_ok,
                   "RESET fires HMI_RESET_HOLD once at 200 ticks and re-arms on release");
    }

    /* 6. RESET hold wins when RUN is released on the same tick */
    {
        bool b_ok = true;
        release_both();
        gb_reset_down = true;
        for (uint32_t i = 0u; i < (MISSION_BTN_RESET_HOLD_TICKS - 1u); i++)
        {
            if (i >= (MISSION_BTN_RESET_HOLD_TICKS - 1u - MISSION_BTN_DEBOUNCE_TICKS))
            {
                gb_run_down = true;
            }
            b_ok = b_ok && (HMI_NONE == hmi_poll());
        }
        gb_run_down = false; /* released on the exact 200th RESET tick */
        b_ok = b_ok && (HMI_RESET_HOLD == hmi_poll());
        release_both();
        HOST_CHECK("S5-U85", b_ok,
                   "HMI_RESET_HOLD takes priority over simultaneous HMI_RUN_PRESS");
    }

    /* 7. LED colours, redundant-write suppression, warning and buzzer patterns */
    {
        uint32_t t = 1000000u;
        uint32_t w0 = 0u;
        /* BOOT: white LED + single 100 ms beep */
        hmi_show(APP_SYS_BOOT, t);
        w0 = g_rgb_writes;
        HOST_CHECK("S5-U85", (g_rgb_r == 32u) && (g_rgb_g == 32u)
                   && (g_rgb_b == 32u) && gb_buzz_on,
                   "BOOT sets white LED and starts 100 ms boot beep");
        hmi_show(APP_SYS_BOOT, t + 110000u);
        HOST_CHECK("S5-U85", (g_rgb_writes == w0) && (!gb_buzz_on),
                   "BOOT suppresses unchanged LED write and stops beep after 100 ms");

        /* READY + start warning: blue blink + continuous buzzer */
        hmi_set_warning(true);
        hmi_show(APP_SYS_READY, t + 200000u);
        HOST_CHECK("S5-U85", (g_rgb_r == 0u) && (g_rgb_g == 0u)
                   && (g_rgb_b == 32u) && gb_buzz_on,
                   "READY with start warning flashes blue and sounds continuous buzzer");
        hmi_set_warning(false);
        hmi_show(APP_SYS_READY, t + 250000u);
        HOST_CHECK("S5-U85", (g_rgb_r == 0u) && (g_rgb_g == 32u)
                   && (g_rgb_b == 0u) && (!gb_buzz_on),
                   "READY without warning shows steady green and silences buzzer");

        /* FINISHED: two 100 ms beeps (0..100 ms, 200..300 ms) */
        hmi_show(APP_SYS_FINISHED, t + 400000u);
        bool b_two = gb_buzz_on;
        hmi_show(APP_SYS_FINISHED, t + 550000u);
        b_two = b_two && (!gb_buzz_on);
        hmi_show(APP_SYS_FINISHED, t + 650000u);
        b_two = b_two && gb_buzz_on;
        hmi_show(APP_SYS_FINISHED, t + 750000u);
        b_two = b_two && (!gb_buzz_on);
        HOST_CHECK("S5-U85", b_two,
                   "FINISHED produces two 100 ms beeps separated by 100 ms silence");

        /* FAULT: red blink + 524 ms alarm buzzer */
        hmi_show(APP_SYS_FAULT, t + 1000000u);
        bool b_fault = (g_rgb_r == 32u) && (g_rgb_g == 0u) && (g_rgb_b == 0u)
                       && gb_buzz_on;
        hmi_show(APP_SYS_FAULT, t + 1000000u + (1u << 19));
        b_fault = b_fault && (!gb_buzz_on);
        HOST_CHECK("S5-U85", b_fault,
                   "FAULT drives red LED and alternating alarm buzzer");
    }

    return (0 == HOST_SUMMARY("hmi")) ? 0 : 1;
}
