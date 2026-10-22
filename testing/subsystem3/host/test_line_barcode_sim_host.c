/*
 *  test_line_barcode_sim_host.c
 *  Host task simulation for subsystem 3 (line.c + barcode.c):
 *    S3-U07  Immediate decode on the 29th width (no quiet-zone wait;
 *            event posted within one 10 ms wake after the final edge)
 *    S3-U08  barcode_map_set(43, 0) and (10, 4) rejected; valid set is
 *            visible to the next decode
 *    S3-U09  Rejoin mode: 10 mm mark, then 20 mm white, then real line ->
 *            two LINE_FOUND events, the second with the real crossing length
 *    S3-U14  Full ISR -> barcode_task -> line_task branch sequence test
 *            for both early (b_branch_recent) and late (CONFIRM) orders
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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
static uint32_t            g_sim_evt_cnt   = 0u;
static uint32_t            g_sim_mode_mask = 0u;
static uint32_t            g_sim_raw_l     = 500u;
static uint32_t            g_sim_raw_r     = 500u;
static uint32_t            g_sim_path      = 0u;
static int32_t             g_sim_speed_l   = 36; /* ~250 mm/s */
static int32_t             g_sim_speed_r   = 36;
static bool                gb_sim_bar_pin  = false;
static status_board_line_t g_sim_line_out  = {0, 0u, 0u, false};
static drv_gpio_irq_cb_t   gp_sim_bar_cb   = NULL;

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

UW in_w(UW port)
{
    (void) port;
    return gb_sim_bar_pin ? (1u << PIN_BAR_DO) : 0u;
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
    g_sim_evt_cnt   = 0u;
    g_sim_mode_mask = 0u;
    g_sim_raw_l     = 500u;
    g_sim_raw_r     = 500u;
    g_sim_path      = 0u;
    g_sim_speed_l   = 36;
    g_sim_speed_r   = 36;
    gb_sim_bar_pin  = false;
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
   At cpw = 36 (~251 mm/s): narrow 3.0 mm = 12000 us, wide 7.5 mm = 30000 us. */
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

/* ---- S3-U07 & S3-U08 ------------------------------------------------- */

static void test_s3_u07_u08(void)
{
    uint32_t end_us   = 0u;
    uint32_t exp_val  = 0u;

    sim_reset();
    g_sim_path = 100u;

    /* S3-U07: Feed 29 elements of '*A*' and run ONE barcode_cycle() without
       advancing path by the 30 mm quiet zone. Must decode immediately! */
    end_us = sim_feed_barcode_edges(10u, 50000u);
    (void) end_us;
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
}

/* ---- S3-U09 (Rejoin mode, review F-02) ------------------------------- */

static void test_s3_u09_rejoin(void)
{
    uint32_t mark_counts = 0u;
    uint32_t line_counts = 0u;

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
}

/* ---- S3-U14 (Full task barcode-branch + line junction integration) --- */

static void test_s3_u14_task_branch_orders(void)
{
    sim_reset();
    sim_calibrate_line();

    /* Case 1: Early branch — barcode sensor sees a 16 mm black bar (64000 us
       at ~250 mm/s) 20 mm BEFORE the line sensors reach the junction. */
    g_sim_path     = 100u;
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 10000u);
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 20000u); /* leading edge of branch */
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 84000u); /* trailing edge: 64 ms >= 14 mm */
    barcode_cycle();

    HOST_CHECK("S3-U14", 1u == barcode_branch_seq(),
               "16 mm single bar increments barcode_branch_seq on trailing edge");

    /* Line task sees the new branch_seq at path = 100 */
    g_sim_raw_l = 2000u;
    g_sim_raw_r = 2000u;
    line_cycle();

    /* 20 mm later (57 counts <= LINE_BRANCH_EARLY_MM), left sensor saturates */
    g_sim_path += 57u;
    g_sim_raw_l = 3500u; /* norm_l = 1000 (sat_l) */
    g_sim_raw_r = 2000u; /* norm_r = 500 (sum = 1500 >= 1400) */
    line_cycle();

    g_sim_path += 40u;   /* >= min_counts (29) */
    g_sim_raw_l = 500u;  /* white after -> T-junction */
    g_sim_raw_r = 500u;
    line_cycle();

    g_sim_path += 90u;   /* >= confirm_counts (86) */
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

    g_sim_path += 40u;
    g_sim_raw_l = 2000u; /* line still present -> CONFIRM */
    g_sim_raw_r = 2000u;
    line_cycle();

    /* Barcode sensor reports branch during CONFIRM */
    gb_sim_bar_pin = true;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 200000u);
    gb_sim_bar_pin = false;
    barcode_edge_isr(DRV_GPIO_IRQ_BOTH, 264000u);
    barcode_cycle();

    g_sim_path += 90u;
    line_cycle();

    HOST_CHECK("S3-U14",
               (1u == g_sim_evt_cnt) &&
               (APP_EVT_JUNCTION == g_sim_evts[0].type) &&
               (APP_JCT_CROSS == g_sim_evts[0].value),
               "full task replay: late barcode branch + left sat -> APP_JCT_CROSS");
}

int main(void)
{
    HOST_INIT();
    test_s3_u07_u08();
    test_s3_u09_rejoin();
    test_s3_u14_task_branch_orders();
    return (0 == HOST_SUMMARY("line_barcode_sim")) ? 0 : 1;
}
