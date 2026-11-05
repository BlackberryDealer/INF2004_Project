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

void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    (void) p_format;
    (void) arg_a;
    (void) arg_b;
    (void) arg_c;
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

void sys_health_check_in(uint32_t task_bit)
{
    (void) task_bit;
}

void status_board_read(status_board_t * p_snapshot)
{
    p_snapshot->motion.path_counts = g_sim_path;
    p_snapshot->motion.speed_l_cpw = g_sim_speed_l;
    p_snapshot->motion.speed_r_cpw = g_sim_speed_r;
}

uint32_t status_board_path(status_board_t const * p_snap)
{
    return p_snap->motion.path_counts;
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
    return p_snap->motion.speed_l_cpw;
}

int32_t status_board_speed_r(status_board_t const * p_snap)
{
    return p_snap->motion.speed_r_cpw;
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
    g_sim_evt_cnt    = 0u;
    g_sim_mode_mask  = 0u;
    g_sim_fault_mask = 0u;
    g_sim_raw_l      = 500u;
    g_sim_raw_r      = 500u;
    g_sim_path       = 0u;
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

/* Same barcode as sim_feed_barcode_edges, but with LM393-style chatter
   (on-car 2026-11-03): at element misread_elem the ISR reads the OLD
   level and the real level follows 800 us later; after element
   bounce_elem a flicker edge back to the old level comes 1500 us later,
   then the real level again 1700 us later.                          */
static uint32_t sim_feed_barcode_chatter(uint32_t char_idx,
                                         uint32_t start_us,
                                         uint32_t misread_elem,
                                         uint32_t bounce_elem)
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
   asymmetry (narrow bar ~0.5 mm = 2200 us, wide bar ~2.8 mm = 12000 us,
   narrow space ~2.3 mm = 9800 us, wide space ~4.6 mm = 19600 us at cpw=36)
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
                durations_us[elem] = b_wide ? 12000u : 2200u;
            }
            else
            {
                durations_us[elem] = b_wide ? 19600u : 9800u;
            }
            elem++;
        }
        if (c_idx < 2u)
        {
            durations_us[elem] = 9800u; /* narrow space inter-char gap */
            elem++;
        }
    }

    /* Optional stray black/white pair(s) right before the barcode */
    for (s_idx = 0u; s_idx < stray_pairs; s_idx++)
    {
        gb_sim_bar_pin = true;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        now_us += 4000u;
        gb_sim_bar_pin = false;
        barcode_edge_isr(DRV_GPIO_IRQ_BOTH, now_us);
        now_us += 6000u;
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

    return ((BAR_STATE_IDLE == app_bits_get(g_bar_prv_a, PRV_BAR_A_STATE_POS,
                                            PRV_BAR_A_STATE_BITS)) &&
            (0u == app_bits_get(g_bar_prv_a, PRV_BAR_A_N_ELEM_POS,
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
    barcode_map_set(43u, APP_BAR_CMD_LEFT);
    barcode_map_set(10u, 4u);
    barcode_map_set(10u, APP_BAR_CMD_UTURN);
    barcode_map_set(14u, APP_BAR_CMD_RIGHT);

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

    /* ---- S3-U10 & S3-U11 (Phase 4 packed MSG_EDGE ring tests) -------- */
    HOST_CHECK("S3-U10", sim_test_u10_ring_pack(),
               "16-bit MSG_EDGE ring (128 B) packs/unpacks 0, 1, 0x7FFE, 0x7FFF and saturates");
    HOST_CHECK("S3-U11", sim_test_u11_dropped_edge(),
               "dropped edge detected from packed MSG_EDGE_LEVEL bit resets capture to IDLE");

    /* ---- S3-U17 (on-car 2026-11-03: LM393 chatter at bar edges) ----- */
    sim_reset();
    g_sim_path = 100u;
    (void) sim_feed_barcode_chatter(13u, 50000u, 10u, BAR_ELEMENTS);
    barcode_cycle();
    exp_val = mission_evt_barcode_value(APP_BAR_CMD_UTURN, 13u);
    HOST_CHECK("S3-U17",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "'D' with one edge read mid-flicker still decodes as U-turn");

    sim_reset();
    g_sim_path = 100u;
    (void) sim_feed_barcode_chatter(13u, 50000u, BAR_ELEMENTS, 6u);
    barcode_cycle();
    HOST_CHECK("S3-U17",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_BARCODE == g_sim_evts[0].type) &&
               (exp_val == g_sim_evts[0].value),
               "'D' with a flicker edge 1.5 ms after a bar edge still decodes");

    /* ---- S3-U18 (Small barcode with optical bias + leading stray edge) */
    sim_reset();
    g_sim_path = 100u;
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

    /* ---- S3-U14 (Full task barcode-branch + line junction integration) */
    sim_reset();
    sim_calibrate_line();

    /* Case 1: Early branch — barcode sensor sees a 16 mm black bar (69000 us
       at cpw = 36) 20 mm BEFORE the line sensors reach the junction. */
    g_sim_path     = 100u;
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 10000u);
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

    /* 20 mm later (<= LINE_BRANCH_EARLY_MM), left sensor saturates */
    g_sim_path += odometry_mm_to_counts(20u);
    g_sim_raw_l = 3500u; /* norm_l = 1000 (sat_l) */
    g_sim_raw_r = 2000u; /* norm_r = 500 (sum = 1500 >= 1400) */
    line_cycle();

    g_sim_path += odometry_mm_to_counts(14u); /* >= LINE_JUNCTION_MIN_MM */
    g_sim_raw_l = 500u;  /* white after -> T-junction */
    g_sim_raw_r = 500u;
    line_cycle();

    g_sim_path += odometry_mm_to_counts(32u); /* >= LINE_JUNCTION_CONFIRM_MM */
    line_cycle();

    HOST_CHECK("S3-U14",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_JUNCTION == g_sim_evts[0].type) &&
               (APP_JCT_T == g_sim_evts[0].value),
               "full task replay: early barcode branch + left sat -> APP_JCT_T");

    /* Case 2: Late branch — line sensors enter CANDIDATE first, barcode
       sensor reports the >= 14 mm branch during CONFIRM. */
    g_sim_evt_cnt = 0u;
    g_sim_path    = 500u;
    g_sim_raw_l   = 3500u; /* sat_l */
    g_sim_raw_r   = 2000u;
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
               "full task replay: late barcode branch + left sat -> APP_JCT_CROSS");

    return (0 == HOST_SUMMARY("line_barcode_sim")) ? 0 : 1;
}
