/**
 * @file    test_bar_ao_default_host.c
 * @brief   S3-U55: barcode.c built as the default image (BAR_AO=1): start
 *          launches R4 instead of the GP7 interrupt, the reported level is
 *          the analog detector's, and flat, curled and bumped strips decode
 *          through the real decoder. Reuses the S3 simulation's mocks and
 *          sensor model (its own main() is renamed and not run).
 * @owner   Buddy 3 (line, barcode)
 */

#define main sim_line_barcode_main_unused
#include "test_line_barcode_sim_host.c"
#undef main

#if (1u != BAR_AO)
#error "S3-U55 must be built with BAR_AO=1"
#endif

int main(void)
{
    HOST_INIT();
    uint32_t ok_flat = 0u;
    uint32_t ok_curl = 0u;
    uint32_t wrong   = 0u;

    sim_reset();
    HOST_CHECK("S3-U55", (1u == g_sim_ao_starts)
               && (0u == g_sim_gpio_attaches),
               "BAR_AO=1: barcode_start() starts R4, not the GP7 interrupt");

    for (uint32_t i = 0u; i < 4u; i++)
    {
        barcode_ao_sample(400u, 1000000u + (i * BAR_AO_PERIOD_US));
    }
    gb_sim_bar_pin = true;                 /* DO pin says black: ignored */
    HOST_CHECK("S3-U55", 0u == barcode_do_level(),
               "on white: barcode_do_level() is the detector, not the pin");
    for (uint32_t i = 4u; i < 8u; i++)
    {
        barcode_ao_sample(3400u, 1000000u + (i * BAR_AO_PERIOD_US));
    }
    gb_sim_bar_pin = false;
    HOST_CHECK("S3-U55", (1u == barcode_do_level())
               && (3400u == barcode_ao_code()),
               "on black: level 1 and the last code 3400 reported");

    for (uint32_t ch = 0u; ch < CODE39_STAR; ch++)
    {
        sim_reset();
        g_sim_ao_seed = 61u + ch;
        sim_feed_barcode_ao(ch, 400u, 3400u, 4000u, 40u, true);
        ok_flat += sim_ao_read_ok(ch) ? 1u : 0u;
        for (uint32_t kind = 0u; kind < 3u; kind++)
        {
            sim_reset();
            g_sim_ao_seed = 71u + ch + kind;
            sim_feed_barcode_curl(ch, kind, true);
            ok_curl += sim_ao_read_ok(ch) ? 1u : 0u;
            for (uint32_t i = 0u; i < g_sim_evt_cnt; i++)
            {
                uint32_t const type = g_sim_evts[i].type;

                if (((APP_EVT_BARCODE == type) || (APP_EVT_UNMAPPED == type))
                    && ((g_sim_evts[i].value & 0x3Fu) != ch))
                {
                    wrong++;
                }
            }
        }
    }
    printf("[INFO] S3-U55 default build reads: flat %u/43, curled %u/129\n",
           ok_flat, ok_curl);
    HOST_CHECK("S3-U55", (43u == ok_flat) && (129u == ok_curl)
               && (0u == wrong),
               "default build: flat 43/43, curled/bumped 129/129, none"
               " wrong");

    return (0 == HOST_SUMMARY("bar_ao_default")) ? 0 : 1;
}
