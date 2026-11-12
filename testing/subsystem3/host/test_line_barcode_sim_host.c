/*
 *  test_line_barcode_sim_host.c
 *  Host task simulation for subsystem 3 (line.c + barcode.c):
 *    S3-U07  Immediate decode on the 29th width (no quiet-zone wait;
 *            event posted within one 10 ms wake after the final edge)
 *    S3-U08  barcode_map_set(43, 0) and (10, 4) rejected; valid set is
 *            visible to the next decode
 *    S3-U09  Rejoin mode: 10 mm mark, then 20 mm white, then real line ->
 *            two LINE_FOUND events, the second with the real crossing length
 *    S3-U10  Ring entry pack/unpack at gaps 0, 1, 0x7FFE, 0x7FFF and
 *            saturation (> 0x7FFF -> 0x7FFF)
 *    S3-U11  Dropped edge detected from the packed MSG_EDGE_LEVEL bit
 *            (consecutive identical levels beyond BAR_LEVEL_FIX_MAX discard
 *            capture to IDLE)
 *    S3-U14  Full ISR -> barcode_task -> line_task branch sequence test
 *            for both early (b_branch_recent) and late (CONFIRM) orders
 *    S3-U15  Rejoin at 45 deg: white -> grey edge -> black still reports
 *            LINE_FOUND
 *    S3-U17  LM393 chatter filter in barcode_task (glitch merge + level fix)
 *    S3-U18  Small barcode with optical bias + leading stray edges decodes
 *            via 29-element sliding window
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Include the real task implementations to drive single cycles directly. */
#include "line.c"
#include "barcode.c"

#define SIM_EVT_MAX (32u)

typedef struct
{
    uint32_t type;
    uint32_t value;
} sim_evt_t;

static sim_evt_t           g_sim_evts[SIM_EVT_MAX];
static uint32_t            g_sim_evt_cnt    = 0u;
static uint32_t            g_sim_mode_mask  = 0u;
static uint32_t            g_sim_fault_mask = 0u;
static uint32_t            g_sim_raw_l      = 500u;
static uint32_t            g_sim_raw_r      = 500u;
static uint32_t            g_sim_path       = 0u;
static uint32_t            g_sim_bar_age_us = 10000u;
static int32_t             g_sim_speed_l    = 36; /* ~250 mm/s */
static int32_t             g_sim_speed_r    = 36;
static bool                gb_sim_bar_pin   = false;
static status_board_line_t g_sim_line_out   = {0, 0u, 0u, false};
static drv_gpio_irq_cb_t   gp_sim_bar_cb    = (drv_gpio_irq_cb_t) 0;

/* ---- Kernel & platform mocks ----------------------------------------- */

ID tk_cre_sem(T_CSEM const * pk_csem)
{
    (void) pk_csem;
    return 1;
}

ER tk_sig_sem(ID semid, INT cnt)
{
    (void) semid;
    (void) cnt;
    return E_OK;
}

ER tk_wai_sem(ID semid, INT cnt, TMO tmout)
{
    (void) semid;
    (void) cnt;
    (void) tmout;
    return E_OK;
}

ER tk_slp_tsk(TMO tmout)
{
    (void) tmout;
    return E_OK;
}

ID tk_get_tid(void)
{
    return 1;
}

UW in_w(UW port)
{
    (void) port;
    return gb_sim_bar_pin ? (1u << PIN_BAR_DO) : 0u;
}

static char   g_sim_log[4096];        /* console capture (S3-U22) */
static size_t g_sim_log_len = 0u;

void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    if (g_sim_log_len < (sizeof(g_sim_log) - 200u))
    {
        int const n = snprintf(&g_sim_log[g_sim_log_len],
                               sizeof(g_sim_log) - g_sim_log_len,
                               p_format, arg_a, arg_b, arg_c);
        if (n > 0)
        {
            g_sim_log_len += (size_t) n;
        }
    }
}

ID app_start_task(FP p_entry, PRI priority, uint64_t * p_stack,
                  uint32_t stack_bytes, char const * p_name)
{
    (void) p_entry;
    (void) priority;
    (void) p_stack;
    (void) stack_bytes;
    (void) p_name;
    return 1;
}

ER drv_gpio_irq_attach(uint32_t pin, uint32_t edges,
                       drv_gpio_irq_cb_t p_callback)
{
    (void) pin;
    (void) edges;
    gp_sim_bar_cb = p_callback;
    return E_OK;
}

void fault_set(uint32_t bit)
{
    g_sim_fault_mask |= bit;
}

void drv_ir_adc_init(void)
{
}

void drv_ir_adc_read(uint32_t * p_left, uint32_t * p_right)
{
    *p_left  = g_sim_raw_l;
    *p_right = g_sim_raw_r;
}

ER sys_tick_register(uint32_t slot, ID task_id)
{
    (void) slot;
    (void) task_id;
    return E_OK;
}

void sys_tick_cycle_start(uint32_t slot)
{
    (void) slot;
}

uint32_t sys_tick_now_us(void)
{
    return g_bar_last_us + g_sim_bar_age_us;
}

void sys_health_check_in(uint32_t task_bit)
{
    (void) task_bit;
}

/* The board is packed (status_board.c); this mock ignores the snapshot
   and serves the simulated motion values directly.                    */
void status_board_read(status_board_t * p_snapshot)
{
    (void) p_snapshot;
}

uint32_t status_board_path(status_board_t const * p_snap)
{
    (void) p_snap;
    return g_sim_path;
}

int32_t status_board_pos_l(status_board_t const * p_snap)
{
    (void) p_snap;
    return 0;
}

int32_t status_board_pos_r(status_board_t const * p_snap)
{
    (void) p_snap;
    return 0;
}

int32_t status_board_speed_l(status_board_t const * p_snap)
{
    (void) p_snap;
    return g_sim_speed_l;
}

int32_t status_board_speed_r(status_board_t const * p_snap)
{
    (void) p_snap;
    return g_sim_speed_r;
}

void status_board_write_line(status_board_line_t const * p_part)
{
    g_sim_line_out = *p_part;
}

bool mission_mode_is_set(uint32_t bit)
{
    return (0u != (g_sim_mode_mask & bit));
}

uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index)
{
    return ((char_index & 0x3Fu) | ((cmd & 0x03u) << 6u));
}

bool mission_post_event(uint32_t type, uint32_t value)
{
    if (g_sim_evt_cnt < SIM_EVT_MAX)
    {
        g_sim_evts[g_sim_evt_cnt].type  = type;
        g_sim_evts[g_sim_evt_cnt].value = value;
        g_sim_evt_cnt++;
    }
    return true;
}

/* ---- Simulation helpers ---------------------------------------------- */

static void sim_reset(void)
{
    gb_bar_reread_always = false;   /* older tests read at cruise; S3-U40 */
    g_sim_evt_cnt    = 0u;
    g_sim_mode_mask  = 0u;
    g_sim_fault_mask = 0u;
    g_sim_raw_l      = 500u;
    g_sim_raw_r      = 500u;
    g_sim_path       = 0u;
    g_sim_bar_age_us = 10000u;
    g_sim_speed_l    = 36;
    g_sim_speed_r    = 36;
    gb_sim_bar_pin   = false;
    (void) line_init();
    (void) barcode_init();
    (void) barcode_start();
}

static void sim_calibrate_line(void)
{
    g_sim_mode_mask = APP_MODE_CAL_IR;
    g_sim_raw_l     = 500u;
    g_sim_raw_r     = 500u;
    line_cycle();
    g_sim_raw_l = 3500u;
    g_sim_raw_r = 3500u;
    line_cycle();
    g_sim_mode_mask = 0u;
    g_sim_raw_l     = 2000u;
    g_sim_raw_r     = 2000u;
    line_cycle();
    g_sim_evt_cnt = 0u; /* clear CAL_DONE event */
}

/* Reference Code 39 patterns to emit realistic timestamped edges. */
static uint16_t const g_sim_patterns[44] =
{
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u,
    0x130u, 0x070u, 0x025u, 0x124u, 0x064u,
    0x109u, 0x049u, 0x148u, 0x019u, 0x118u,
    0x058u, 0x00Du, 0x10Cu, 0x04Cu, 0x01Cu,
    0x103u, 0x043u, 0x142u, 0x013u, 0x112u,
    0x052u, 0x007u, 0x106u, 0x046u, 0x016u,
    0x181u, 0x0C1u, 0x1C0u, 0x091u, 0x190u,
    0x0D0u, 0x085u, 0x184u, 0x0C4u, 0x0A8u,
    0x0A2u, 0x08Au, 0x02Au, 0x094u
};

/* Replay a 29-element `*<char>*` barcode through barcode_edge_isr.
   At cpw = 36 (~233 mm/s): narrow 12000 us, wide 30000 us. */
static uint32_t sim_feed_barcode_edges(uint32_t char_idx, uint32_t start_us)
{
    uint32_t durations_us[BAR_ELEMENTS] = {0u};
    uint32_t chars[3]                   = { CODE39_STAR, char_idx, CODE39_STAR };
    uint32_t c_idx                      = 0u;
    uint32_t elem                       = 0u;
    uint32_t now_us                     = start_us;

    for (c_idx = 0u; c_idx < 3u; c_idx++)
    {
        uint16_t const pat = g_sim_patterns[chars[c_idx]];
        uint32_t       bit = 0u;

        for (bit = 0u; bit < 9u; bit++)
        {
            bool const b_wide = (0u != (pat & (uint16_t) (1u << (8u - bit))));

            durations_us[elem] = b_wide ? 30000u : 12000u;
            elem++;
        }
        if (c_idx < 2u)
        {
            durations_us[elem] = 12000u; /* narrow inter-char gap */
            elem++;
        }
    }

    /* Leading edge (end of quiet zone -> black bar 0) */
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);

    for (elem = 0u; elem < BAR_ELEMENTS; elem++)
    {
        now_us        += durations_us[elem];
        gb_sim_bar_pin = !gb_sim_bar_pin;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
    }

    return now_us;
}

/* Replay logged widths through the IRQ and task at the mocked wheel speed. */
static void sim_bar_widths(uint8_t const * widths, uint32_t count,
                            uint32_t now_us, bool b_leading_high)
{
    uint32_t const start = g_sim_path;
    uint32_t total_um = 0u;

    gb_sim_bar_pin = b_leading_high;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
    barcode_cycle();
    for (uint32_t i = 0u; i < count; i++)
    {
        now_us += ((uint32_t) widths[i] * 5000000u)
                  / ((uint32_t) g_sim_speed_l * APP_UM_PER_COUNT);
        total_um += (uint32_t) widths[i] * 100u;
        g_sim_path = start + (total_um + APP_UM_PER_COUNT / 2u)
                             / APP_UM_PER_COUNT;
        gb_sim_bar_pin = !gb_sim_bar_pin;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        barcode_cycle();
    }
}

/* S3-U37: feed raw ISR edges. gap > 0: rising edge (DO now high),
   gap < 0: falling edge. Path follows each edge's speed (cpw).       */
static void sim_feed_raw(int32_t const * p_gap_us, uint8_t const * p_cpw,
                         uint32_t count, uint32_t * p_now_us)
{
    for (uint32_t i = 0u; i < count; i++)
    {
        uint32_t const gap = (p_gap_us[i] < 0) ? (uint32_t) (-p_gap_us[i])
                                               : (uint32_t) p_gap_us[i];

        *p_now_us    += gap;
        g_sim_bar_age_us = 0u;           /* task runs at the edge */
        g_sim_speed_l = (int32_t) p_cpw[i];
        g_sim_speed_r = g_sim_speed_l;
        g_sim_path   += (gap * (uint32_t) p_cpw[i]) / 50000u;
        gb_sim_bar_pin = (p_gap_us[i] > 0);
        barcode_edge_isr((p_gap_us[i] > 0) ? DRV_GPIO_IRQ_RISE
                                           : DRV_GPIO_IRQ_FALL, *p_now_us);
        barcode_cycle();
    }
}

/* S3-U37: '*' X '*' at a fixed speed with LM393 chatter: every second
   transition is followed by echo edges 1.05 ms (and every fourth by a
   second one 2.1 ms) later, and every third edge reports the wrong
   level. Returns the number of raw edges written.                    */
static uint32_t sim_chatter_frame(uint32_t pattern, bool b_reverse,
                                  uint32_t cpw, int32_t * p_gap_us,
                                  uint8_t * p_cpw)
{
    static uint16_t const frame_chars[3] = {0x094u, 0u, 0x094u};
    uint8_t  widths[BAR_ELEMENTS];
    uint32_t times[2u * (BAR_ELEMENTS + 1u) + 8u];
    bool     levels[2u * (BAR_ELEMENTS + 1u) + 8u];
    uint32_t n_ev  = 0u;
    uint32_t t_us  = 0u;
    uint32_t n_out = 0u;
    uint32_t prev  = 0u;

    for (uint32_t c = 0u; c < 3u; c++)
    {
        uint32_t const bits = (1u == c) ? pattern : frame_chars[c];

        for (uint32_t e = 0u; e < 9u; e++)
        {
            bool const b_wide = (0u != (bits & (1u << (8u - e))));
            bool const b_bar  = (0u == (e % 2u));

            widths[(c * 10u) + e] = b_bar ? (b_wide ? 110u : 25u)
                                          : (b_wide ? 150u : 55u);
        }
        if (c < 2u)
        {
            widths[(c * 10u) + 9u] = 55u;   /* inter-character gap */
        }
    }
    if (b_reverse)
    {
        for (uint32_t i = 0u; i < (BAR_ELEMENTS / 2u); i++)
        {
            uint8_t const tmp = widths[i];

            widths[i] = widths[BAR_ELEMENTS - 1u - i];
            widths[BAR_ELEMENTS - 1u - i] = tmp;
        }
    }

    for (uint32_t k = 0u; k <= BAR_ELEMENTS; k++)
    {
        bool const b_now_bar = (0u == (k % 2u)) && (k < BAR_ELEMENTS);

        times[n_ev]  = t_us;
        levels[n_ev] = ((k % 3u) == 1u) ? !b_now_bar : b_now_bar;
        n_ev++;
        if (0u == (k % 2u))
        {
            times[n_ev]  = t_us + 1050u;
            levels[n_ev] = !levels[n_ev - 1u];
            n_ev++;
            if (0u == (k % 4u))
            {
                times[n_ev]  = t_us + 2100u;
                levels[n_ev] = b_now_bar;
                n_ev++;
            }
        }
        if (k < BAR_ELEMENTS)
        {
            t_us += ((uint32_t) widths[k] * 5000000u)
                    / (cpw * APP_UM_PER_COUNT);
        }
    }

    prev = 0u;
    for (uint32_t i = 0u; i < n_ev; i++)
    {
        int32_t const gap = (0u == i) ? 600000 : (int32_t) (times[i] - prev);

        p_gap_us[n_out] = levels[i] ? gap : -gap;
        p_cpw[n_out]    = (uint8_t) cpw;
        n_out++;
        prev = times[i];
    }
    return n_out;
}

static bool in_rng(int value, int lo, int hi)
{
    return (value >= lo) && (value <= hi);
}

/* S3-U22: the first n_elem elements of '*<char>*' only (edges missed by
   the sensor), then nothing: the 30 mm quiet zone ends the capture.   */
static void sim_feed_truncated(uint32_t char_idx, uint32_t n_elem)
{
    uint32_t chars[3] = { CODE39_STAR, char_idx, CODE39_STAR };
    uint32_t now_us   = 50000u;
    uint32_t elem     = 0u;

    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
    for (uint32_t c_idx = 0u; (c_idx < 3u) && (elem < n_elem); c_idx++)
    {
        uint16_t const pat = g_sim_patterns[chars[c_idx]];

        for (uint32_t bit = 0u; (bit < 10u) && (elem < n_elem); bit++)
        {
            bool const b_wide = (bit < 9u)
                && (0u != (pat & (uint16_t) (1u << (8u - bit))));

            if ((9u == bit) && (2u == c_idx))
            {
                break;                       /* no gap after the last char */
            }
            now_us        += b_wide ? 30000u : 12000u;
            gb_sim_bar_pin = !gb_sim_bar_pin;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
            elem++;
        }
    }
}

/* Same barcode as sim_feed_barcode_edges, but with LM393-style chatter
   (on-car 2026-11-03): at element misread_elem the ISR reads the OLD
   level and the real level follows 800 us later; after element
   bounce_elem a flicker edge back to the old level comes 1500 us later,
   then the real level again 1700 us later. A nonzero false_pulse_us
   inserts a pulse of that duration 3 ms before six real transitions. */
static uint32_t sim_feed_barcode_chatter(uint32_t char_idx,
                                         uint32_t start_us,
                                         uint32_t misread_elem,
                                         uint32_t bounce_elem,
                                         uint32_t false_pulse_us)
{
    uint32_t durations_us[BAR_ELEMENTS] = {0u};
    uint32_t chars[3]                   = { CODE39_STAR, char_idx, CODE39_STAR };
    uint32_t c_idx                      = 0u;
    uint32_t elem                       = 0u;
    uint32_t now_us                     = start_us;
    bool     b_level                    = true;

    for (c_idx = 0u; c_idx < 3u; c_idx++)
    {
        uint16_t const pat = g_sim_patterns[chars[c_idx]];
        uint32_t       bit = 0u;

        for (bit = 0u; bit < 9u; bit++)
        {
            bool const b_wide = (0u != (pat & (uint16_t) (1u << (8u - bit))));

            durations_us[elem] = b_wide ? 30000u : 12000u;
            elem++;
        }
        if (c_idx < 2u)
        {
            durations_us[elem] = 12000u;
            elem++;
        }
    }

    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);

    for (elem = 0u; elem < BAR_ELEMENTS; elem++)
    {
        bool const b_old = b_level;

        now_us += durations_us[elem];
        b_level = !b_level;
        if ((false_pulse_us != 0u) && (elem >= 3u) && (elem < 9u))
        {
            /* Inside BAR_SETTLE_US before the real edge (S3-U37). */
            uint32_t const lead_us = 3000u;

            gb_sim_bar_pin = b_level;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us - lead_us);
            gb_sim_bar_pin = b_old;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH,
                             now_us - lead_us + false_pulse_us);
        }
        if (elem == misread_elem)
        {
            gb_sim_bar_pin = b_old;            /* read mid-flicker */
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
            gb_sim_bar_pin = b_level;          /* settles 800 us later */
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us + 800u);
        }
        else
        {
            gb_sim_bar_pin = b_level;
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        }
        if (elem == bounce_elem)
        {
            gb_sim_bar_pin = b_old;            /* flicker back ... */
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us + 1500u);
            gb_sim_bar_pin = b_level;          /* ... and settle */
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us + 1700u);
        }
    }

    return now_us;
}

/* S3-U18: Feed a half-scale (small) barcode with heavy optical bar/space
   asymmetry (narrow bar ~0.5 mm = 4400 us, wide bar ~2.8 mm = 24000 us,
   narrow space ~2.3 mm = 19600 us, wide space ~4.6 mm = 39200 us at cpw=18,
   cruise speed; S3-U37 folds edges closer than BAR_SETTLE_US)
   and optional leading stray edges (e.g. tape/paper edge before '*'). */
static uint32_t sim_feed_small_biased_barcode(uint32_t char_idx,
                                              uint32_t start_us,
                                              uint32_t stray_pairs)
{
    uint32_t durations_us[BAR_ELEMENTS] = {0u};
    uint32_t chars[3]                   = { CODE39_STAR, char_idx, CODE39_STAR };
    uint32_t c_idx                      = 0u;
    uint32_t elem                       = 0u;
    uint32_t now_us                     = start_us;
    uint32_t s_idx                      = 0u;

    for (c_idx = 0u; c_idx < 3u; c_idx++)
    {
        uint16_t const pat = g_sim_patterns[chars[c_idx]];
        uint32_t       bit = 0u;

        for (bit = 0u; bit < 9u; bit++)
        {
            bool const b_wide = (0u != (pat & (uint16_t) (1u << (8u - bit))));
            bool const b_bar  = (0u == (bit & 1u));

            if (b_bar)
            {
                durations_us[elem] = b_wide ? 24000u : 4400u;
            }
            else
            {
                durations_us[elem] = b_wide ? 39200u : 19600u;
            }
            elem++;
        }
        if (c_idx < 2u)
        {
            durations_us[elem] = 19600u; /* narrow space inter-char gap */
            elem++;
        }
    }

    /* Optional stray black/white pair(s) right before the barcode */
    for (s_idx = 0u; s_idx < stray_pairs; s_idx++)
    {
        gb_sim_bar_pin = true;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        now_us += 8000u;
        gb_sim_bar_pin = false;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        now_us += 12000u;
    }

    /* Leading edge of barcode '*' bar 0 */
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);

    for (elem = 0u; elem < BAR_ELEMENTS; elem++)
    {
        now_us        += durations_us[elem];
        gb_sim_bar_pin = !gb_sim_bar_pin;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
    }

    return now_us;
}

/* S3-U10: Test ring entry packing via barcode_edge_isr and direct MSG_EDGE
   round-trip at gaps 0, 1, 0x7FFE, 0x7FFF and saturation (> 0x7FFF). */
static bool sim_test_u10_ring_pack(void)
{
    static uint32_t const gaps[5]    = {0u, 1u, 0x7FFEu, 0x7FFFu, 0x9999u};
    static uint32_t const exp_gap[5] = {0u, 1u, 0x7FFEu, 0x7FFFu, 0x7FFFu};
    uint32_t              idx        = 0u;
    bool                  b_ok       = (128u == sizeof(g_bar_ring));

    for (idx = 0u; idx < 5u; idx++)
    {
        uint32_t const lvl = idx & 1u;
        uint32_t const sat = (gaps[idx] > (uint32_t) MSG_EDGE_GAP_MAX)
                                 ? (uint32_t) MSG_EDGE_GAP_MAX
                                 : gaps[idx];
        uint32_t word = app_bits_put(0u, MSG_EDGE_GAP_POS,
                                     MSG_EDGE_GAP_BITS, sat);
        uint16_t ring_entry = 0u;

        word       = app_bits_put(word, MSG_EDGE_LEVEL_POS,
                                  MSG_EDGE_LEVEL_BITS, lvl);
        ring_entry = (uint16_t) word;

        if ((exp_gap[idx] != app_bits_get((uint32_t) ring_entry,
                                          MSG_EDGE_GAP_POS,
                                          MSG_EDGE_GAP_BITS)) ||
            (lvl != app_bits_get((uint32_t) ring_entry,
                                 MSG_EDGE_LEVEL_POS,
                                 MSG_EDGE_LEVEL_BITS)))
        {
            b_ok = false;
        }
    }

    /* Also verify ISR saturation of gap > 0x7FFF (1,000,000 us -> 32767) */
    sim_reset();
    g_bar_last_us  = 1000u;
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 1001000u);
    if ((0x7FFFu != app_bits_get((uint32_t) g_bar_ring[0],
                                 MSG_EDGE_GAP_POS, MSG_EDGE_GAP_BITS)) ||
        (1u != app_bits_get((uint32_t) g_bar_ring[0],
                            MSG_EDGE_LEVEL_POS, MSG_EDGE_LEVEL_BITS)))
    {
        b_ok = false;
    }

    return b_ok;
}

/* S3-U11: Consecutive edges with the same level bit beyond BAR_LEVEL_FIX_MAX
   (missed edges) discard the capture and return the decoder state to IDLE. */
static bool sim_test_u11_dropped_edge(void)
{
    uint32_t idx    = 0u;
    uint32_t now_us = 10000u;

    sim_reset();
    g_sim_path = 200u;

    /* First rising edge -> enters CAPTURING (LAST_LEVEL = 1) */
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);

    /* Feed BAR_LEVEL_FIX_MAX + 1 edges that each match LAST_LEVEL */
    for (idx = 0u; idx <= BAR_LEVEL_FIX_MAX; idx++)
    {
        now_us        += 12000u;
        gb_sim_bar_pin = (0u == (idx & 1u));
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
    }

    barcode_cycle();

    /* S3-U37: bar and space alternate; the DO level read during
       chatter is ignored, so repeated levels keep the capture going. */
    return ((BAR_STATE_CAPTURING == app_bits_get(g_bar_prv_a,
                                                 PRV_BAR_A_STATE_POS,
                                                 PRV_BAR_A_STATE_BITS)) &&
            ((BAR_LEVEL_FIX_MAX + 1u) == app_bits_get(g_bar_prv_a,
                                                      PRV_BAR_A_N_ELEM_POS,
                                                      PRV_BAR_A_N_ELEM_BITS)) &&
            (0u == g_sim_evt_cnt));
}

int main(void)
{
    uint32_t exp_val     = 0u;
    uint32_t mark_counts = 0u;
    uint32_t line_counts = 0u;

    HOST_INIT();

    /* ---- S3-U07 & S3-U08 --------------------------------------------- */
    sim_reset();
    g_sim_path = 100u;

    /* S3-U07: Feed 29 elements of '*A*' and run ONE barcode_cycle() without
       advancing path by the 30 mm quiet zone. Must decode immediately! */
    (void) sim_feed_barcode_edges(10u, 50000u);
    barcode_cycle();

    exp_val = mission_evt_barcode_value(APP_BAR_CMD_LEFT, 10u);
    HOST_CHECK("S3-U07",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "29th element triggers immediate APP_EVT_BARCODE without quiet zone wait");

    /* Advance past 150 mm cooldown (500 counts) to return to IDLE */
    g_sim_path += 500u;
    barcode_cycle();
    g_sim_evt_cnt = 0u;

    /* S3-U08: Invalid map calls rejected; valid remap of 'A' (10) -> UTURN
       and new map of 'E' (14) -> RIGHT take effect on next decode */
    HOST_CHECK("S3-U08", !barcode_map_set(43u, APP_BAR_CMD_LEFT)
               && !barcode_map_set(10u, 4u),
               "out-of-range MAP values report failure");
    HOST_CHECK("S3-U08", barcode_map_set(10u, APP_BAR_CMD_UTURN)
               && barcode_map_set(14u, APP_BAR_CMD_RIGHT),
               "existing and new mappings report success");

    (void) sim_feed_barcode_edges(10u, 2000000u);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_UTURN, 10u);
    HOST_CHECK("S3-U08",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "invalid map_set rejected and valid remap of 'A' -> UTURN visible");

    g_sim_path += 500u;
    barcode_cycle();
    g_sim_evt_cnt = 0u;

    (void) sim_feed_barcode_edges(14u, 4000000u);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 14u);
    HOST_CHECK("S3-U08",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "newly mapped character 'E' -> RIGHT decoded and posted");

    HOST_CHECK("S3-U08", barcode_map_set(15u, APP_BAR_CMD_LEFT)
               && barcode_map_set(16u, APP_BAR_CMD_RIGHT)
               && barcode_map_set(17u, APP_BAR_CMD_STRAIGHT)
               && !barcode_map_set(18u, APP_BAR_CMD_LEFT)
               && barcode_map_set(15u, APP_BAR_CMD_RIGHT),
               "a full table rejects new letters and updates existing ones");

    /* ---- S3-U10 & S3-U11 (Phase 4 packed MSG_EDGE ring tests) -------- */
    HOST_CHECK("S3-U10", sim_test_u10_ring_pack(),
               "16-bit MSG_EDGE ring (128 B) packs/unpacks 0, 1, 0x7FFE, 0x7FFF and saturates");
    HOST_CHECK("S3-U11", sim_test_u11_dropped_edge(),
               "repeated MSG_EDGE_LEVEL bits no longer reset the capture (S3-U37)");

    /* ---- S3-U17 (on-car 2026-11-03: LM393 chatter at bar edges) ----- */
    sim_reset();
    g_sim_path = 100u;
    (void) sim_feed_barcode_chatter(13u, 50000u, 10u, BAR_ELEMENTS, 0u);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_UTURN, 13u);
    HOST_CHECK("S3-U17",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "'D' with one edge read mid-flicker still decodes as U-turn");

    sim_reset();
    g_sim_path = 100u;
    (void) sim_feed_barcode_chatter(13u, 50000u, BAR_ELEMENTS, 6u, 0u);
    barcode_cycle();
    HOST_CHECK("S3-U17",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "'D' with a flicker edge 1.5 ms after a bar edge still decodes");

    /* ---- S3-U18 (Small barcode with optical bias + leading stray edge) */
    sim_reset();
    g_sim_path = 100u;
    g_sim_speed_l = 18;
    g_sim_speed_r = 18;
    (void) sim_feed_small_biased_barcode(10u, 50000u, 0u);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_LEFT, 10u);
    HOST_CHECK("S3-U18",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "small half-scale '*A*' with 0.5 mm eroded narrow bars decodes as LEFT");

    sim_reset();
    g_sim_path = 100u;
    g_sim_speed_l = 18;
    g_sim_speed_r = 18;
    (void) sim_feed_small_biased_barcode(11u, 50000u, 1u);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 11u);
    HOST_CHECK("S3-U18",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "small '*B*' preceded by 2 stray edges decodes via 29-element sliding window");

    /* ---- S3-U09 (Rejoin mode, review F-02) --------------------------- */
    sim_reset();
    sim_calibrate_line();

    /* Enter rejoin mode (APP_MODE_JUNCTION_INHIBIT set, without LINE_LOST) */
    g_sim_mode_mask = APP_MODE_JUNCTION_INHIBIT;
    g_sim_path      = 100u;

    /* 3 white cycles (>= 2 required) */
    g_sim_raw_l = 500u;
    g_sim_raw_r = 500u;
    line_cycle();
    g_sim_path += 7u;
    line_cycle();
    g_sim_path += 7u;
    line_cycle();

    /* First black run: a 10 mm stray mark */
    mark_counts = odometry_mm_to_counts(10u);
    g_sim_raw_l = 3500u;
    g_sim_raw_r = 3500u;
    line_cycle(); /* run starts at g_sim_path */
    g_sim_path += mark_counts;
    line_cycle();

    /* 3 white cycles (20 mm of white between the mark and the real line) */
    g_sim_raw_l = 500u;
    g_sim_raw_r = 500u;
    line_cycle(); /* black run 1 ends here -> posts LINE_FOUND (~10 mm) */
    g_sim_path += 29u;
    line_cycle();
    g_sim_path += 29u;
    line_cycle();

    /* Second black run: the real line (45 mm crossing) */
    line_counts = odometry_mm_to_counts(45u);
    g_sim_raw_l = 3500u;
    g_sim_raw_r = 3500u;
    line_cycle(); /* run 2 starts here */
    g_sim_path += line_counts;
    line_cycle();

    /* End of second black run */
    g_sim_raw_l = 500u;
    g_sim_raw_r = 500u;
    line_cycle(); /* black run 2 ends here -> posts LINE_FOUND (~45 mm) */

    HOST_CHECK("S3-U09",
               (2u == g_sim_evt_cnt) &&
               (APP_EVT_LINE_FOUND == g_sim_evts[0].type) &&
               (APP_EVT_LINE_FOUND == g_sim_evts[1].type),
               "rejoin mode reports both the rejected mark and the real line");
    HOST_CHECK("S3-U09",
               (g_sim_evts[0].value >= 9u) && (g_sim_evts[0].value <= 11u) &&
               (g_sim_evts[1].value >= 44u) && (g_sim_evts[1].value <= 46u),
               "first LINE_FOUND ~10 mm, second LINE_FOUND ~45 mm");

    /* ---- S3-U15 (Rejoin at 45 deg, on-car log 2026-11-03) ------------ */
    /* White, then grey at the line edge, then black: the grey cycles must
       not cancel the white seen before, or the line is never reported. */
    sim_reset();
    sim_calibrate_line();
    g_sim_mode_mask = APP_MODE_JUNCTION_INHIBIT;
    g_sim_path      = 100u;
    g_sim_raw_l     = 500u;               /* white */
    g_sim_raw_r     = 500u;
    line_cycle();
    g_sim_path += 5u;
    line_cycle();
    g_sim_path += 5u;
    line_cycle();
    g_sim_raw_l = 2000u;                  /* grey edge, 2 cycles */
    g_sim_raw_r = 2000u;
    g_sim_path += 5u;
    line_cycle();
    g_sim_path += 5u;
    line_cycle();
    line_counts = odometry_mm_to_counts(25u);
    g_sim_raw_l = 3500u;                  /* black: 25 mm crossing */
    g_sim_raw_r = 3500u;
    g_sim_path += 5u;
    line_cycle();
    g_sim_path += line_counts;
    g_sim_raw_l = 2000u;                  /* grey far edge ends the run */
    g_sim_raw_r = 2000u;
    line_cycle();
    HOST_CHECK("S3-U15",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_LINE_FOUND == g_sim_evts[0].type) &&
               (g_sim_evts[0].value >= 24u) && (g_sim_evts[0].value <= 26u),
               "white, grey edge, black: LINE_FOUND ~25 mm is still posted");

    /* ---- S3-U19 (on-car 2026-11-06: both white while centred) -------- */
    /* The line lies between the sensors when the car is centred: 200 ms
       of both white must not post LINE_LOST (each one made mission stop
       and turn 60 deg: the shaking). A line that went black under a
       sensor and then vanished is still lost after 200 ms. A centred
       car is never lost, whatever the distance (no line end, S3-U44). */
    sim_reset();
    sim_calibrate_line();
    g_sim_path  = 1000u;
    g_sim_raw_l = 500u;                   /* both white, centred */
    g_sim_raw_r = 500u;
    for (uint32_t i = 0u; i < 40u; i++)   /* 400 ms, 40 mm */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U19", (0u == g_sim_evt_cnt)
               && (APP_LINE_ON == g_sim_line_out.line_status),
               "centred, both white 400 ms / 40 mm: no LINE_LOST, line ON");
    g_sim_raw_l = 2000u;                  /* graze the left edge (grey) */
    for (uint32_t i = 0u; i < 3u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    g_sim_raw_l = 500u;
    for (uint32_t i = 0u; i < 40u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U19", 0u == g_sim_evt_cnt,
               "grey graze, then 400 ms both white: still no LINE_LOST");
    for (uint32_t i = 0u; i < 2000u; i++) /* 2 m centred, no graze */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U44", (0u == g_sim_evt_cnt)
               && (APP_LINE_ON == g_sim_line_out.line_status),
               "centred 2 m without touching the line: no LINE_LOST, line ON");
    sim_reset();
    sim_calibrate_line();
    g_sim_path  = 1000u;
    g_sim_raw_l = 3500u;                  /* black under the left sensor */
    g_sim_raw_r = 500u;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    g_sim_raw_l = 500u;
    for (uint32_t i = 0u; i < (LINE_EXIT_LOST_MM + 5u); i++)
    {
        g_sim_path += odometry_mm_to_counts(4u);   /* >= 200 ms too */
        line_cycle();
    }
    HOST_CHECK("S3-U19", (1u == g_sim_evt_cnt)
               && (APP_EVT_LINE_LOST == g_sim_evts[0].type)
               && (APP_LINE_LOST == g_sim_line_out.line_status),
               "line black under a sensor, then gone: LINE_LOST");

    /* ---- S3-U20 (on-car 2026-11-06 14:38: shaking after the rejoin) -- */
    /* After the bypass: LINE LOST every 30-50 mm, LINE FOUND 90-190 ms
       later, each time a mission STOP + 60 deg pivot. The line had gone
       black under a sensor and on to the inside (the car was turning
       toward it), so 200 ms later it was still right there. A black
       exit is now lost only after LINE_EXIT_LOST_MM of travel (and 200
       ms), giving the recovery steer time to bring the line back. A
       centred car is never lost (S3-U44).                             */
    sim_reset();
    sim_calibrate_line();
    g_sim_path  = 1000u;
    g_sim_raw_l = 3500u;                  /* black under the left sensor */
    g_sim_raw_r = 500u;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    g_sim_raw_l = 500u;
    for (uint32_t i = 0u; i < 30u; i++)   /* 300 ms, 30 mm both white */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U20", 0u == g_sim_evt_cnt,
               "black exit, 300 ms but only 30 mm both white: no LINE_LOST");
    g_sim_raw_r = 2000u;                  /* recovery brings it back */
    g_sim_path += odometry_mm_to_counts(1u);
    line_cycle();
    g_sim_raw_r = 500u;
    for (uint32_t i = 0u; i < 100u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U20", 0u == g_sim_evt_cnt,
               "line back under the other sensor: no LINE_LOST at all");
    sim_reset();
    sim_calibrate_line();
    g_sim_path  = 1000u;
    g_sim_raw_l = 500u;                   /* centred straight, no graze */
    g_sim_raw_r = 500u;
    for (uint32_t i = 0u; i < 200u; i++)  /* 200 mm */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U20", 0u == g_sim_evt_cnt,
               "centred 200 mm without touching the line: no LINE_LOST");
    HOST_CHECK("S3-U20", LINE_EXIT_LOST_MM >= 50u,
               "loss distance after a black exit >= 50 mm");

    /* ---- S3-U44 (on-car 2026-11-12: every slow reread lost the line) */
    /* The 300 mm centred line-end rule counted the barcode rewind
       (reverse travel is in the path): rewind ~220 mm + lead-in ~110 mm
       posted LINE_LOST just before the barcode and the search ended the
       slow pass. The track has no line end, so a centred car is never
       lost; a black exit still is (S3-U20).                          */
    sim_reset();
    sim_calibrate_line();
    g_sim_path      = 1000u;
    g_sim_mode_mask = APP_MODE_BARCODE_REREAD | APP_MODE_BARCODE_INHIBIT
                      | APP_MODE_JUNCTION_INHIBIT;
    g_sim_raw_l     = 500u;               /* centred, both white */
    g_sim_raw_r     = 500u;
    for (uint32_t i = 0u; i < 220u; i++)  /* braking + rewind 220 mm */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    g_sim_mode_mask = APP_MODE_BARCODE_REREAD;   /* slow pass */
    for (uint32_t i = 0u; i < 400u; i++)  /* lead-in + barcode + quiet */
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U44", (0u == g_sim_evt_cnt)
               && (APP_LINE_ON == g_sim_line_out.line_status),
               "rewind 220 mm + slow pass 400 mm centred: no LINE_LOST");
    g_sim_mode_mask = 0u;                 /* reread over, cruise again */
    for (uint32_t i = 0u; i < 1000u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    HOST_CHECK("S3-U44", 0u == g_sim_evt_cnt,
               "then 1 m more centred at cruise: still no LINE_LOST");
    sim_reset();
    sim_calibrate_line();
    g_sim_path  = 1000u;
    g_sim_raw_r = 500u;
    g_sim_raw_l = 3500u;                  /* line drifts under the left */
    for (uint32_t i = 0u; i < 5u; i++)
    {
        g_sim_path += odometry_mm_to_counts(1u);
        line_cycle();
    }
    g_sim_raw_l = 500u;                   /* and leaves there */
    for (uint32_t i = 0u; i < (LINE_EXIT_LOST_MM + 5u); i++)
    {
        g_sim_path += odometry_mm_to_counts(4u);
        line_cycle();
    }
    HOST_CHECK("S3-U44", (1u == g_sim_evt_cnt)
               && (APP_EVT_LINE_LOST == g_sim_evts[0].type),
               "black exit, then gone: LINE_LOST as before");

    /* ---- S3-U45 (on-car 2026-11-12 15:09: no reread after a sideways
       first pass) ----------------------------------------------------- */
    /* The car crossed the strip while still settling on the line; the
       start guard never matched, so no rewind and no slow reread came.
       Any capture whose first four widths look like bars and spaces
       (1.2 to 25.4 mm) now asks for the rewind; flicker does not.     */
    {
        uint8_t const first_pass[] = {85u, 65u, 190u, 40u};  /* 15:09 */
        uint8_t const flicker[]    = {6u, 5u, 15u, 7u};       /* 15:10 */

        sim_reset();
        gb_bar_reread_always = true;
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(first_pass, sizeof(first_pass), 50000u, true);
        HOST_CHECK("S3-U45", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type),
                   "four bar-like widths without a start guard: rewind");

        sim_reset();
        gb_bar_reread_always = true;
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(flicker, sizeof(flicker), 50000u, true);
        HOST_CHECK("S3-U45", 0u == g_sim_evt_cnt,
                   "four flicker widths (0.5-1.5 mm, one < 1.2): no rewind");

        sim_reset();
        gb_bar_reread_always = false;     /* rewind only after a failure */
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(first_pass, sizeof(first_pass), 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U45", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type),
                   "the same capture dropped without a guard: rewind");
    }

    /* ---- S3-U22 (on-car 2026-11-06 14:49: barcodes never decoded) --- */
    /* Captures ended with 26 / 15 elements and nothing said which bars
       or spaces were missing. A dropped or invalid capture now prints
       its element widths (0.1 mm, bar first, 2 per line) so the sensor
       height / LM393 trimmer can be judged from one run.              */
    sim_reset();
    g_sim_path    = 100u;
    g_sim_log_len = 0u;
    g_sim_log[0]  = '\0';
    sim_feed_truncated(10u, 26u);         /* 'A' with 3 elements missed */
    barcode_cycle();
    g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM + 5u);
    barcode_cycle();
    HOST_CHECK("S3-U22", NULL != strstr(g_sim_log, "BAR capture dropped: 26"),
               "26-element capture dropped at the quiet zone (as on the car)");
    HOST_CHECK("S3-U22",
               (NULL != strstr(g_sim_log, "BAR widths 1+ (0.1 mm, bar first):"))
               && (NULL != strstr(g_sim_log, "BAR widths 25+ (0.1 mm, bar first):"))
               && (NULL == strstr(g_sim_log, "BAR widths 27+")),
               "its widths are printed, 2 per line, up to element 26");
    {
        char const * p_one  = strstr(g_sim_log, "BAR widths 1+");
        char const * p_two  = strstr(g_sim_log, "BAR widths 3+");
        int          w_a    = 0;
        int          w_b    = 0;
        int          w_c    = 0;
        int          w_d    = 0;
        bool         b_ok   = false;

        if ((NULL != p_one) && (NULL != p_two))
        {
            b_ok = (2 == sscanf(p_one, "BAR widths 1+ (0.1 mm, bar first):"
                                " %d %d", &w_a, &w_b))
                   && (2 == sscanf(p_two, "BAR widths 3+ (0.1 mm, bar"
                                   " first): %d %d", &w_c, &w_d));
        }
        /* '*' starts narrow bar, wide space, narrow bar, narrow space:
           12 ms / 30 ms at cpw 36 -> 2.7 mm / 6.9 mm                  */
        HOST_CHECK("S3-U22", b_ok && in_rng(w_a, 25, 29) && in_rng(w_b, 67, 71)
                   && in_rng(w_c, 25, 29) && in_rng(w_d, 25, 29),
                   "widths are the measured element widths (narrow, wide, narrow)");
    }

    /* S3-U23: actual 28 widths from the 11:41 export, with a plausible
       leading narrow bar only in the simulated complete slow pass. */
    {
        uint8_t const widths[BAR_ELEMENTS] = {
            60u, 27u, 146u, 27u, 148u, 27u, 58u, 108u, 57u, 28u,
            58u, 30u, 52u, 115u, 143u, 25u, 152u, 22u, 66u, 27u,
            63u, 18u, 145u, 24u, 162u, 31u, 50u, 126u, 51u
        };

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(&widths[1], 4u, 50000u, true);
        HOST_CHECK("S3-U23", 0u == g_sim_evt_cnt,
                   "four plausible widths do not justify a rewind");

        sim_reset();
        gb_bar_reread_always = true;      /* the car's setting (S3-U45) */
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(&widths[1], CODE39_START_ELEMENTS, 50000u, true);
        HOST_CHECK("S3-U23",
                   (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type)
                   && (g_sim_evts[0].value <= 68u),
                   "reverse guard with first bar missing requests a rewind");

        g_sim_mode_mask |= APP_MODE_BARCODE_REREAD | APP_MODE_BARCODE_INHIBIT;
        sim_bar_widths(widths, BAR_ELEMENTS, 600000u, true);
        HOST_CHECK("S3-U23", 1u == g_sim_evt_cnt,
                   "edges during the rewind are discarded");

        g_sim_evt_cnt = 0u;
        g_sim_mode_mask &= ~APP_MODE_BARCODE_INHIBIT;
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        barcode_map_set(15u, APP_BAR_CMD_RIGHT);   /* map F dynamically */
        sim_bar_widths(widths, BAR_ELEMENTS, 3000000u, true);
        HOST_CHECK("S3-U23",
                   (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE == g_sim_evts[0].type)
                   && (mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u)
                       == g_sim_evts[0].value),
                   "full slow reread decodes reverse F with its new map");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD;
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(&widths[1], BAR_ELEMENTS - 1u, 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U23",
                   (1u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type)
                   && (28u == g_sim_evts[0].value),
                   "a slow capture still missing the first bar is invalid");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD;
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        {
            uint8_t invalid_widths[BAR_ELEMENTS];
            memset(invalid_widths, 40, sizeof(invalid_widths));
            sim_bar_widths(invalid_widths, BAR_ELEMENTS, 50000u, true);
        }
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U23",
                   (1u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type)
                   && (BAR_ELEMENTS == g_sim_evts[0].value),
                   "29 elements without valid Code 39 also fail the reread");

        /* S3-U31: let an intact guard finish at cruise in both directions. */
        for (uint32_t direction = 0u; direction < 2u; direction++)
        {
            uint8_t replay[BAR_ELEMENTS];

            for (uint32_t i = 0u; i < BAR_ELEMENTS; i++)
            {
                replay[i] = (0u == direction)
                    ? widths[i] : widths[BAR_ELEMENTS - 1u - i];
            }

            sim_reset();
            g_sim_mode_mask = APP_MODE_MOTOR_EN;
            g_sim_speed_l = odometry_mm_s_to_cpw(120);
            g_sim_speed_r = g_sim_speed_l;
            (void) barcode_map_set(15u, APP_BAR_CMD_RIGHT);
            sim_bar_widths(replay, BAR_ELEMENTS, 50000u, true);
            HOST_CHECK("S3-U31",
                       (1u == g_sim_evt_cnt)
                       && (APP_EVT_BARCODE == g_sim_evts[0].type)
                       && (mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u)
                           == g_sim_evts[0].value),
                       (0u == direction)
                           ? "reconstructed F at cruise decodes without rewind"
                           : "opposite scan at cruise decodes without rewind");
        }
    }

    {
        uint8_t const noise[] = {11u, 12u, 12u, 13u};
        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(noise, sizeof(noise), 50000u, true);
        HOST_CHECK("S3-U24", 0u == g_sim_evt_cnt,
                   "four similar short elements do not trigger a rewind");
    }

    {
        uint8_t const yellow[] = {41u, 77u, 220u, 73u, 21u, 30u, 154u, 163u};
        uint8_t const noise[] = {
            41u, 77u, 101u, 73u, 21u, 30u, 154u, 163u, 45u, 60u
        };
        /* Forward guard without its first bar, then one data element. */
        uint8_t const missing_first[] = {
            75u, 30u, 30u, 75u, 30u, 75u, 30u, 30u, 30u, 30u
        };

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(yellow, sizeof(yellow), 50000u, true);
        HOST_CHECK("S3-U27", 0u == g_sim_evt_cnt,
                   "the eight reported yellow-tape widths cannot trigger");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(noise, sizeof(noise), 50000u, true);
        HOST_CHECK("S3-U27", 0u == g_sim_evt_cnt,
                   "mixed narrow and wide noise fails the guard pattern");

        sim_reset();
        gb_bar_reread_always = true;      /* the car's setting (S3-U45) */
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(missing_first, sizeof(missing_first), 50000u, true);
        HOST_CHECK("S3-U27", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type),
                   "a forward guard missing its first bar can retry");
    }

    /* S3-U34: a failed capture can retry (S3-U45: no start guard needed). */
    {
        static uint8_t const guard[] = {
            30u, 75u, 30u, 30u, 75u, 30u, 75u, 30u, 30u, 30u
        };
        static uint8_t const noise[] = {
            28u, 14u, 15u, 23u, 14u, 26u, 42u, 18u, 14u, 15u, 18u
        };
        int32_t const cruise_cpw = odometry_mm_s_to_cpw(120);

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = cruise_cpw;
        g_sim_speed_r = cruise_cpw;
        sim_bar_widths(guard, sizeof(guard), 50000u, true);
        HOST_CHECK("S3-U34", 0u == g_sim_evt_cnt,
                   "an intact guard alone lets the scan continue");

        /* Miss a falling edge. The next rise leaves the recorded levels
           one step out of phase until the mismatch budget is exhausted. */
        for (uint32_t i = 0u; i <= BAR_LEVEL_FIX_MAX; i++)
        {
            gb_sim_bar_pin = ((i & 1u) == 0u);
            g_sim_path += odometry_mm_to_counts(3u);
            barcode_edge_isr(DRV_GPIO_IRQ_BOTH, g_bar_last_us + 25000u);
            barcode_cycle();
        }
        HOST_CHECK("S3-U34", (0u == g_sim_evt_cnt)
                   && (BAR_STATE_CAPTURING == app_bits_get(
                       g_bar_prv_a, PRV_BAR_A_STATE_POS,
                       PRV_BAR_A_STATE_BITS)),
                   "repeated DO levels after an intact guard keep the capture"
                   " (S3-U37)");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = cruise_cpw;
        g_sim_speed_r = cruise_cpw;
        sim_bar_widths(guard, sizeof(guard), 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        barcode_cycle();
        HOST_CHECK("S3-U34", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type),
                   "quiet after a known start can request one rewind");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = cruise_cpw;
        g_sim_speed_r = cruise_cpw;
        {
            uint8_t invalid[BAR_ELEMENTS];

            memset(invalid, 15, sizeof(invalid));
            for (uint32_t i = 0u; i < sizeof(guard); i++)
            {
                invalid[i] = guard[i] / 2u;
            }
            sim_bar_widths(invalid, BAR_ELEMENTS, 50000u, true);
        }
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U34", (2u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type)
                   && (BAR_ELEMENTS == g_sim_evts[0].value)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[1].type),
                   "an invalid short frame reports failure before retrying");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = cruise_cpw;
        g_sim_speed_r = cruise_cpw;
        sim_bar_widths(noise, sizeof(noise), 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U45", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type),
                   "the recorded eleven-element fragment now asks for the reread");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD;
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(guard, sizeof(guard), 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U34", (1u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type),
                   "a failed slow reread does not request another rewind");
    }

    /* S3-U38: on-car 2026-11-10 21:37, a deliberately wrong barcode read
       as 29 elements. Its start guard has a 23.8 mm wide bar, so the
       guard check must accept it and the failed read must ask for a
       rewind (it was only logged as invalid). */
    {
        static uint8_t const misread[BAR_ELEMENTS] = {
            67u, 176u, 82u, 51u, 238u, 29u, 191u, 51u, 66u, 43u,
            154u, 81u, 73u, 47u, 64u, 173u, 63u, 30u, 193u, 54u,
            55u, 114u, 100u, 49u, 180u, 42u, 175u, 36u, 57u
        };

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(misread, BAR_ELEMENTS, 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U38", (2u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[1].type)
                   && (g_sim_evts[1].value >= 270u)
                   && (g_sim_evts[1].value <= 320u),
                   "a logged misread with a 23.8 mm guard bar asks for a"
                   " rewind of the whole strip");
    }

    /* S3-U40 (Hanz 2026-11-10 22:48): cruise is too fast for small,
       thin barcodes. An intact start guard at cruise always asks for a
       rewind, so the whole strip is read at the slow speed. Not during
       the slow reread itself. */
    {
        static uint8_t const guard[] = {
            30u, 75u, 30u, 30u, 75u, 30u, 75u, 30u, 30u, 30u
        };

        sim_reset();
        gb_bar_reread_always = true;
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(guard, sizeof(guard), 50000u, true);
        HOST_CHECK("S3-U40", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type)
                   && (g_sim_evts[0].value < 80u),
                   "an intact guard at cruise asks for a slow reread");

        sim_reset();
        gb_bar_reread_always = true;
        g_sim_mode_mask = APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD;
        g_sim_speed_l = odometry_mm_s_to_cpw(60);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(guard, sizeof(guard), 50000u, true);
        HOST_CHECK("S3-U40", 0u == g_sim_evt_cnt,
                   "the slow reread itself does not ask again");
    }

    /* S3-U29: actual final F pass, missing the first narrow bar. */
    {
        uint8_t const widths[] = {
            21u, 119u, 25u, 126u, 35u, 84u, 122u, 61u, 52u, 65u,
            25u, 33u, 101u, 115u, 26u, 203u, 31u, 75u, 32u, 77u,
            18u, 119u, 26u, 111u, 33u, 78u, 148u, 66u
        };

        sim_reset();
        gb_bar_reread_always = true;      /* the car's setting (S3-U45) */
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l = odometry_mm_s_to_cpw(120);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(widths, CODE39_START_ELEMENTS, 50000u, true);
        HOST_CHECK("S3-U29", (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE_PROBE == g_sim_evts[0].type)
                   && (g_sim_evts[0].value <= 71u),
                   "the measured guard requests a retry within 150 mm");

        sim_reset();
        g_sim_mode_mask = APP_MODE_MOTOR_EN | APP_MODE_BARCODE_REREAD;
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        sim_bar_widths(widths, sizeof(widths), 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM);
        barcode_cycle();
        HOST_CHECK("S3-U29", (1u == g_sim_evt_cnt)
                   && (APP_EVT_INVALID == g_sim_evts[0].type)
                   && (28u == g_sim_evts[0].value),
                   "a measured 28-element frame still cannot arm a turn");
    }

    /* S3-U30: six false pulses preceding real transitions. */
    sim_reset();
    g_sim_speed_l = odometry_mm_s_to_cpw(120);
    g_sim_speed_r = g_sim_speed_l;
    (void) barcode_map_set(15u, APP_BAR_CMD_RIGHT);
    (void) sim_feed_barcode_chatter(15u, 50000u, BAR_ELEMENTS, BAR_ELEMENTS,
                                    2000u);
    barcode_cycle();
    HOST_CHECK("S3-U30", (1u == g_sim_evt_cnt)
               && (APP_EVT_BARCODE == g_sim_evts[0].type)
               && (mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u)
                   == g_sim_evts[0].value),
               "short pulses do not fragment a complete measured F");

    /* Exercise the same rejection across separate task wakeups. */
    sim_reset();
    g_sim_speed_l = odometry_mm_s_to_cpw(120);
    g_sim_speed_r = g_sim_speed_l;
    g_sim_bar_age_us = 0u;
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_RISE, 50000u);
    barcode_cycle();
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_FALL, 52000u);
    barcode_cycle();
    g_sim_bar_age_us = 10000u;
    barcode_cycle();
    HOST_CHECK("S3-U30", (BAR_STATE_IDLE == app_bits_get(
                   g_bar_prv_a, PRV_BAR_A_STATE_POS, PRV_BAR_A_STATE_BITS)),
               "a short leading pulse leaves the scanner idle");

    sim_reset();
    g_sim_bar_age_us = 0u;
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_RISE, 50000u);
    barcode_cycle();
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_FALL, 50400u);
    g_sim_bar_age_us = 10000u;
    barcode_cycle();
    HOST_CHECK("S3-U30", (BAR_STATE_IDLE == app_bits_get(
                   g_bar_prv_a, PRV_BAR_A_STATE_POS, PRV_BAR_A_STATE_BITS)),
               "a rejected sub-millisecond return also leaves the scanner idle");

    /* S3-U32: the ISR drops a 400 us return, then sees the real edge. */
    sim_reset();
    g_sim_speed_l = odometry_mm_s_to_cpw(120);
    g_sim_speed_r = g_sim_speed_l;
    (void) barcode_map_set(15u, APP_BAR_CMD_RIGHT);
    (void) sim_feed_barcode_chatter(15u, 50000u, BAR_ELEMENTS, BAR_ELEMENTS,
                                    400u);
    barcode_cycle();
    HOST_CHECK("S3-U32", (1u == g_sim_evt_cnt)
               && (APP_EVT_BARCODE == g_sim_evts[0].type)
               && (mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u)
                   == g_sim_evts[0].value),
               "a rejected pulse return does not break the measured F");

    /* S3-U25: preserve the first edge and still read all 29 elements. */
    {
        static uint8_t const widths[BAR_ELEMENTS] = {
            55u, 17u, 148u, 19u, 151u, 18u, 66u, 113u, 64u, 25u,
            56u, 30u, 48u, 106u, 154u, 27u, 146u, 24u, 57u, 24u,
            66u, 18u, 145u, 22u, 142u, 29u, 57u, 126u, 50u
        };
        sim_reset();
        gb_sim_bar_pin = false;  /* pin already bounced after the rise */
        barcode_edge_isr(DRV_GPIO_IRQ_RISE, 50000u);
        gb_sim_bar_pin = true;
        barcode_edge_isr(DRV_GPIO_IRQ_FALL, 70000u);
        HOST_CHECK("S3-U25",
                   (1u == app_bits_get(g_bar_ring[0], MSG_EDGE_LEVEL_POS,
                                       MSG_EDGE_LEVEL_BITS))
                   && (0u == app_bits_get(g_bar_ring[1], MSG_EDGE_LEVEL_POS,
                                          MSG_EDGE_LEVEL_BITS)),
                   "single IRQ flags preserve edge direction across bounce");

        sim_reset();
        g_sim_speed_l = odometry_mm_s_to_cpw(70);
        g_sim_speed_r = g_sim_speed_l;
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        sim_bar_widths(&widths[1], 3u, 50000u, true);
        g_sim_mode_mask |= APP_MODE_BARCODE_REREAD | APP_MODE_BARCODE_INHIBIT;
        sim_bar_widths(widths, 2u, 600000u, true);
        HOST_CHECK("S3-U25",
                   (g_bar_head == g_bar_tail)
                   && (0u == app_bits_get(g_bar_prv_a,
                                           PRV_BAR_A_N_ELEM_POS,
                                           PRV_BAR_A_N_ELEM_BITS)),
                   "rewind discards queued edges and the initial element count");
        g_sim_mode_mask &= ~APP_MODE_BARCODE_INHIBIT;
        (void) barcode_map_set(15u, APP_BAR_CMD_RIGHT);
        sim_bar_widths(widths, BAR_ELEMENTS, 1500000u, false);
        HOST_CHECK("S3-U25",
                   (1u == g_sim_evt_cnt)
                   && (APP_EVT_BARCODE == g_sim_evts[0].type)
                   && (mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u)
                       == g_sim_evts[0].value),
                   "a falling first edge on reread still decodes all 29 elements");
    }

    /* ---- S3-U36 (on-car 2026-11-10 20:54: 8 of 29 elements kept) ---- */
    /* The edge counter saw the whole barcode, but only 8 widths reached
       the capture. A failed capture now also prints the raw edges it was
       given (task cycle, signed gap, speed used) so the merge point can
       be replayed on the host.                                        */
    {
        uint8_t const six[6] = {30u, 75u, 30u, 30u, 75u, 30u};
        uint32_t      lines  = 0u;
        uint32_t      cpw    = 0u;
        uint32_t      dt_us  = 0u;
        char          want[48];
        char const *  p_at   = NULL;

        sim_reset();
        g_sim_log_len   = 0u;
        g_sim_log[0]    = '\0';
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_speed_l   = odometry_mm_s_to_cpw(120);
        g_sim_speed_r   = g_sim_speed_l;
        g_sim_path      = 100u;
        sim_bar_widths(six, 6u, 50000u, true);
        g_sim_path += odometry_mm_to_counts(BAR_QUIET_MM + 5u);
        barcode_cycle();
        for (uint32_t i = 0u; i < 200u; i++)
        {
            barcode_cycle();
        }
        for (p_at = strstr(g_sim_log, "BAR E "); NULL != p_at;
             p_at = strstr(p_at + 1, "BAR E "))
        {
            lines++;
        }
        HOST_CHECK("S3-U36",
                   NULL != strstr(g_sim_log, "BAR edge dump: 7 edges"),
                   "a dropped capture announces a dump of its 7 raw edges");
        HOST_CHECK("S3-U36", 7u == lines,
                   "one line per raw edge, then the log is cleared");
        cpw   = (uint32_t) g_sim_speed_l;
        dt_us = (30u * 5000000u) / (cpw * APP_UM_PER_COUNT);
        (void) snprintf(want, sizeof(want), " -%u %u\n",
                        (unsigned) ((dt_us >> 4) << 4), (unsigned) cpw);
        HOST_CHECK("S3-U36", NULL != strstr(g_sim_log, want),
                   "the end of the first bar shows its gap (negative = now"
                   " white) and the speed used");
    }

    /* ---- S3-U37 (on-car 2026-11-10 21:09, edge dump) ----------------- */
    /* The 32 logged raw edges read as '*' then 'A' when chatter edges
       (1.0-2.1 ms after a transition) are folded in and bar / space come
       from alternation. 12 of them repeated the previous DO level, so the
       level-based capture dropped after 8 elements. Logged gaps are
       quantised to 16 us, so 15 us is added back to each.             */
    {
        static int32_t const dump_gap[32] = {
            -524272, -250528, -524272, 51312, -125760, 52928, 36560, 147120,
            -42768, 182320, -1056, -53600, 992, 83824, -43776, -992,
            153872, -43600, 64720, -33376, 73216, -1056, -182096, -992,
            -1008, 97328, -992, -43936, -992, 205728, 37360, 53632
        };
        static uint8_t const dump_cpw[32] = {
            12u, 20u, 22u, 20u, 20u, 21u, 20u, 19u, 18u, 14u, 14u, 14u,
            14u, 13u, 17u, 17u, 19u, 17u, 18u, 19u, 15u, 15u, 13u, 13u,
            13u, 13u, 13u, 12u, 12u, 19u, 20u, 20u
        };
        int32_t  gaps[64];
        uint8_t  cpws[64];
        uint32_t now_us = 0u;
        uint32_t n_raw  = 0u;
        uint32_t n_elem = 0u;
        bool     b_ok   = true;

        for (uint32_t i = 0u; i < 32u; i++)
        {
            gaps[i] = (dump_gap[i] < 0) ? (dump_gap[i] - 15)
                                        : (dump_gap[i] + 15);
            cpws[i] = dump_cpw[i];
        }
        sim_reset();
        g_sim_log_len   = 0u;
        g_sim_log[0]    = '\0';
        g_sim_mode_mask = APP_MODE_MOTOR_EN;
        g_sim_path      = 1000u;
        barcode_cycle();
        sim_feed_raw(gaps, cpws, 32u, &now_us);
        n_elem = app_bits_get(g_bar_prv_a, PRV_BAR_A_N_ELEM_POS,
                              PRV_BAR_A_N_ELEM_BITS);
        HOST_CHECK("S3-U37", (NULL == strstr(g_sim_log, "BAR capture dropped"))
                   && (21u == n_elem),
                   "logged 21:09 pass: 21 elements, no drop on chatter");
        HOST_CHECK("S3-U37",
                   code39_has_start_guard(g_bar_widths, CODE39_START_ELEMENTS),
                   "its first 10 elements are the '*' start guard");

        /* Full frames with the same chatter, both reading directions. */
        for (uint32_t d = 0u; d < 4u; d++)
        {
            uint32_t const pattern = (d < 2u) ? 0x109u : 0x058u;
            uint32_t const exp_val = (d < 2u)
                ? mission_evt_barcode_value(APP_BAR_CMD_LEFT, 10u)
                : mission_evt_barcode_value(APP_BAR_CMD_RIGHT, 15u);

            sim_reset();
            g_sim_mode_mask = APP_MODE_MOTOR_EN;
            g_sim_path      = 1000u;
            (void) barcode_map_set(15u, APP_BAR_CMD_RIGHT);
            barcode_cycle();
            now_us = 0u;
            n_raw  = sim_chatter_frame(pattern, (1u == (d % 2u)), 18u,
                                       gaps, cpws);
            sim_feed_raw(gaps, cpws, n_raw, &now_us);
            if (!((1u == g_sim_evt_cnt)
                  && (APP_EVT_BARCODE == g_sim_evts[0].type)
                  && (exp_val == g_sim_evts[0].value)))
            {
                b_ok = false;
            }
        }
        HOST_CHECK("S3-U37", b_ok,
                   "*A* and *F* with chatter and wrong DO levels decode in"
                   " both directions");
    }

    /* ---- S3-U14 (Full task barcode-branch + line junction integration) */
    sim_reset();
    sim_calibrate_line();

    /* Case 1: Early branch — barcode sensor sees a 16 mm black bar (69000 us
       at cpw = 36) 20 mm BEFORE the line sensors reach the junction. */
    g_sim_path     = 100u;
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 20000u); /* leading edge of branch */
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 89000u); /* trailing edge: >= 14 mm */
    barcode_cycle();

    HOST_CHECK("S3-U14", 1u == barcode_branch_seq(),
               "16 mm single bar increments barcode_branch_seq on trailing edge");

    /* Line task sees the new branch_seq at path = 100 */
    g_sim_raw_l = 2000u;
    g_sim_raw_r = 2000u;
    line_cycle();

    /* 20 mm later (<= LINE_BRANCH_EARLY_MM), right sensor saturates */
    g_sim_path += odometry_mm_to_counts(20u);
    g_sim_raw_l = 2000u; /* norm_l = 500 (sum = 1500 >= 1400) */
    g_sim_raw_r = 3500u; /* norm_r = 1000 (sat_r) */
    line_cycle();

    g_sim_path += odometry_mm_to_counts(14u); /* >= LINE_JUNCTION_MIN_MM */
    g_sim_raw_l = 500u;  /* white after: the cross-junction's arm ahead */
    g_sim_raw_r = 500u;
    line_cycle();

    g_sim_path += odometry_mm_to_counts(32u); /* >= LINE_JUNCTION_CONFIRM_MM */
    line_cycle();

    HOST_CHECK("S3-U14",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_JUNCTION == g_sim_evts[0].type) &&
               (APP_JCT_CROSS == g_sim_evts[0].value),
               "full task replay: early barcode branch + right sat -> APP_JCT_CROSS");

    /* Case 2: Late branch — line sensors enter CANDIDATE first, barcode
       sensor reports the >= 14 mm branch during CONFIRM. */
    g_sim_evt_cnt = 0u;
    g_sim_path    = 500u;
    g_sim_raw_l   = 2000u;
    g_sim_raw_r   = 3500u; /* sat_r */
    line_cycle();

    g_sim_path += odometry_mm_to_counts(14u); /* >= LINE_JUNCTION_MIN_MM */
    g_sim_raw_l = 2000u; /* line still present -> CONFIRM */
    g_sim_raw_r = 2000u;
    line_cycle();

    /* Barcode sensor reports branch during CONFIRM */
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 200000u);
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 269000u);
    barcode_cycle();

    g_sim_path += odometry_mm_to_counts(32u); /* >= LINE_JUNCTION_CONFIRM_MM */
    line_cycle();

    HOST_CHECK("S3-U14",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_JUNCTION == g_sim_evts[0].type) &&
               (APP_JCT_CROSS == g_sim_evts[0].value),
               "full task replay: late barcode branch + right sat -> APP_JCT_CROSS");

    sim_reset();
    sim_calibrate_line();

    g_sim_raw_l = 3500u;
    g_sim_raw_r = 500u;
    line_cycle();
    HOST_CHECK("S3-U28", g_sim_line_out.steering > 0,
               "ordinary one-sided black still turns toward the line");

    g_sim_raw_r = 3500u;
    line_cycle();
    g_sim_path += odometry_mm_to_counts(12u);
    g_sim_raw_l = 500u;
    line_cycle();
    HOST_CHECK("S3-U28", 0 == g_sim_line_out.steering,
               "a staggered cross edge cannot steer the car");

    g_sim_path += odometry_mm_to_counts(2u);
    line_cycle();
    HOST_CHECK("S3-U33", (0 == g_sim_line_out.steering)
               && line_track_steer_line_between(&g_steer),
               "the next cycle on the trailing stripe keeps steering centred");

    g_sim_raw_r = 500u;
    line_cycle();
    HOST_CHECK("S3-U28", (0 == g_sim_line_out.steering)
               && line_track_steer_line_between(&g_steer),
               "both white after the stripe does not start a recovery swing");

    g_sim_path += odometry_mm_to_counts(32u);
    g_sim_raw_l = 2000u;
    line_cycle();
    HOST_CHECK("S3-U28", (1u == g_sim_evt_cnt)
               && (APP_EVT_JUNCTION == g_sim_evts[0].type)
               && (APP_JCT_CROSS == g_sim_evts[0].value)
               && (0 == g_sim_line_out.steering),
               "continuing tape still confirms the cross and clears bias");

    g_sim_raw_r = 3500u;
    g_sim_raw_l = 500u;
    line_cycle();
    HOST_CHECK("S3-U28", g_sim_line_out.steering < 0,
               "line following responds normally after the cross");

    /* Both sensors see black, below saturation and too briefly to classify. */
    sim_reset();
    sim_calibrate_line();
    g_sim_path = odometry_mm_to_counts(100u);
    g_sim_raw_l = 2900u;
    g_sim_raw_r = 2900u;
    line_cycle();
    g_sim_path = odometry_mm_to_counts(105u);
    g_sim_raw_l = 500u;
    line_cycle();
    HOST_CHECK("S3-U33", (0 == g_sim_line_out.steering)
               && (0u == g_sim_evt_cnt),
               "a brief dark band clears steering without a junction event");
    g_sim_path = odometry_mm_to_counts(116u);
    line_cycle();
    HOST_CHECK("S3-U33", g_sim_line_out.steering < 0,
               "a lasting one-sided reading steers again after the hold");

    return (0 == HOST_SUMMARY("line_barcode_sim")) ? 0 : 1;
}
