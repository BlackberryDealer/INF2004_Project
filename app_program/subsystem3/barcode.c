/**
 * @file    barcode.c
 * @brief   Barcode edge ISR, K2 ring buffer, decoder state machine, and map.
 * @owner   Buddy 3 (line, barcode)
 * @kind    task
 * @reqs    FR2, FR3, NFR5
 */

#include "barcode.h"           /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_bits.h"
#include "../app_layout.h"
#include "../board_pins.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "../subsystem5/drv_gpio_irq.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/status_board.h"
#include "cfg_barcode.h"
#include "code39.h"

/* ---- private constants and types ------------------------------------ */

#define BARCODE_STACK_WORDS  (APP_STACK_BARCODE_BYTES / 8u)
#define BAR_DO_PIN_MASK      (1u << PIN_BAR_DO)
#define BAR_SEM_TIMEOUT_MS   (10)
#define BAR_GAP_SHIFT        (4u)
#define BAR_LOG_MIN_ELEM     (4u)  /* shorter captures are line/branch edges */
#define BAR_GLITCH_01MM      (BAR_GLITCH_UM / 100u)
#define BAR_BRANCH_MIN_01MM  (BAR_BRANCH_MIN_MM * 10u)
#define BAR_WIDTH_DIVISOR    (312500u)
#define BAR_MAP_EMPTY        (0xFFu)
#define BAR_MAP_EMPTY_CHAR   ((uint32_t) PRV_BAR_MAP_CHAR_MAX)

#define BAR_STATE_IDLE       (0u)
#define BAR_STATE_CAPTURING  (1u)
#define BAR_STATE_DECODING   (2u)
#define BAR_STATE_COOLDOWN   (3u)

/* ---- private data ---------------------------------------------------- */

static uint64_t          g_barcode_stack[BARCODE_STACK_WORDS];
static ID                gh_bar_sem                   = 0;

/* K2 packed 16-bit edge ring (MSG_EDGE layout: 2 B x 64 = 128 B). */
static volatile uint16_t g_bar_ring[BAR_RING_ENTRIES];
static volatile uint8_t  g_bar_head                   = 0u; /* E6 ISR writer */
static volatile uint8_t  g_bar_tail                   = 0u; /* E6 task writer */
static volatile uint8_t  g_bar_overflow               = 0u; /* SH_BAR_OVF */
static uint32_t          g_bar_last_us                = 0u; /* E5 ISR-private */
static volatile uint8_t  g_bar_branch_seq             = 0u; /* SH_BAR */
static volatile uint32_t g_bar_edge_cnt               = 0u; /* ISR only */

/* Packed private state words (PRV_BAR_A, PRV_BAR_B, PRV_BAR_W). */
static uint32_t          g_bar_prv_a                  = 0u;
static uint32_t          g_bar_prv_b                  = 0u;
static uint32_t          g_bar_fix_cnt                = 0u; /* level fixes */
static uint32_t          g_bar_carry_16us             = 0u; /* flicker time */
static uint8_t           g_bar_widths[BAR_CAPTURE_MAX];

/* Default character-to-command map: A->LEFT, B->RIGHT, C->STRAIGHT, D->UTURN */
static uint8_t const g_bar_default_map[BAR_MAP_ENTRIES] =
{
    (uint8_t) (10u | (APP_BAR_CMD_LEFT << PRV_BAR_MAP_CMD_POS)),
    (uint8_t) (11u | (APP_BAR_CMD_RIGHT << PRV_BAR_MAP_CMD_POS)),
    (uint8_t) (12u | (APP_BAR_CMD_STRAIGHT << PRV_BAR_MAP_CMD_POS)),
    (uint8_t) (13u | (APP_BAR_CMD_UTURN << PRV_BAR_MAP_CMD_POS)),
    BAR_MAP_EMPTY,
    BAR_MAP_EMPTY,
    BAR_MAP_EMPTY,
    BAR_MAP_EMPTY
};

static volatile uint8_t g_bar_map[BAR_MAP_ENTRIES];

/* ---- private function prototypes ------------------------------------- */

static void     barcode_task(INT stacd, void * p_exinf);
static void     barcode_edge_isr(uint32_t events, uint32_t now_us);
static void     barcode_init_state(void);
static void     barcode_cycle(void);
static uint32_t barcode_avg_cpw(int32_t speed_l, int32_t speed_r);
static void     barcode_process_edge(uint32_t gap_16us, uint32_t level,
                                     uint32_t cpw, uint32_t path);
static void     barcode_edge_idle(uint32_t level, uint32_t path);
static void     barcode_edge_capturing(uint32_t gap_16us, uint32_t level,
                                       uint32_t cpw, uint32_t path);
static void     barcode_store_width(uint32_t n_elem, uint8_t width);
static void     barcode_step_decoding(uint32_t path);
static void     barcode_check_timeouts(uint32_t path);
static bool     barcode_lookup_cmd(uint32_t char_index, uint32_t * p_cmd);
static void     barcode_mark_set(uint32_t path);
static uint32_t barcode_mark_get(void);
static void     barcode_state_set(uint32_t state, uint32_t n_elem);

/* ---- public functions ------------------------------------------------- */

/**
 * barcode_init
 * @brief   Creates K2 SEM_BAR and initialises the edge ring and command map.
 * @return  E_OK or kernel error code from tk_cre_sem().
 * @context usermain
 */
ER barcode_init(void)
{
    ER     ercd = E_OK;
    T_CSEM csem = {0};

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = (INT) BAR_RING_ENTRIES; /* cast: 64 fits kernel INT */
    gh_bar_sem   = tk_cre_sem(&csem);

    if (E_OK > gh_bar_sem)
    {
        ercd = gh_bar_sem;
    }
    else
    {
        barcode_init_state();
    }

    return ercd;
}

/**
 * barcode_start
 * @brief   Creates barcode_task and attaches barcode_edge_isr to PIN_BAR_DO.
 * @return  E_OK or kernel error code.
 * @context usermain
 */
ER barcode_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) barcode_task, /* cast: kernel entry type */
                             APP_PRI_BARCODE, g_barcode_stack,
                             APP_STACK_BARCODE_BYTES, "barcode_task");

    if (E_OK > tid)
    {
        ercd = tid;
    }
    else
    {
        ercd = drv_gpio_irq_attach(PIN_BAR_DO, DRV_GPIO_IRQ_BOTH,
                                   barcode_edge_isr);
    }

    return ercd;
}

/**
 * barcode_map_set
 * @brief   Replaces or adds a character-to-command mapping (single-byte store).
 * @param   char_index  Code 39 character index (0..42).
 * @param   cmd         Command code (APP_BAR_CMD_*, 0..3).
 * @return  none
 * @context mission_task
 */
void barcode_map_set(uint32_t char_index, uint32_t cmd)
{
    if ((char_index < APP_C39_CHAR_COUNT) && (cmd <= APP_BAR_CMD_UTURN))
    {
        uint32_t packed = app_bits_put(0u, PRV_BAR_MAP_CHAR_POS,
                                       PRV_BAR_MAP_CHAR_BITS, char_index);
        uint8_t  new_entry   = 0u;
        uint32_t idx         = 0u;
        uint32_t empty_slot  = BAR_MAP_ENTRIES;
        bool     b_was_found = false;

        packed = app_bits_put(packed, PRV_BAR_MAP_CMD_POS,
                              PRV_BAR_MAP_CMD_BITS, cmd);
        /* cast: PRV_BAR_MAP_USED_BITS == 8 fits in uint8_t */
        new_entry = (uint8_t) packed;

        for (idx = 0u; idx < BAR_MAP_ENTRIES; idx++)
        {
            uint32_t const entry    = (uint32_t) g_bar_map[idx]; /* cast */
            uint32_t const cur_char = app_bits_get(
                entry, PRV_BAR_MAP_CHAR_POS, PRV_BAR_MAP_CHAR_BITS);

            if (cur_char == char_index)
            {
                g_bar_map[idx] = new_entry;
                b_was_found    = true;
            }
            else if ((BAR_MAP_EMPTY_CHAR == cur_char) &&
                     (BAR_MAP_ENTRIES == empty_slot))
            {
                empty_slot = idx;
            }
            else
            {
                /* Slot holds another character. */
            }
        }

        if ((!b_was_found) && (empty_slot < BAR_MAP_ENTRIES))
        {
            g_bar_map[empty_slot] = new_entry;
        }
        else
        {
            /* Updated in place or map table is full. */
        }
    }
    else
    {
        /* Out-of-range character index or command: ignore. */
    }
}

/**
 * barcode_branch_seq
 * @brief   Returns the monotonic branch counter (+1 per long single bar seen).
 * @return  8-bit wrapping sequence value as uint32_t.
 * @context line_task
 */
uint32_t barcode_branch_seq(void)
{
    /* cast: widen atomic 8-bit sequence counter */
    return (uint32_t) g_bar_branch_seq;
}

/**
 * barcode_edge_count
 * @brief   Bring-up diagnosis: DO edges accepted by the ISR since boot.
 * @return  Wrapping edge count.
 * @context line_task (console line while the motors are off)
 */
uint32_t barcode_edge_count(void)
{
    return g_bar_edge_cnt;
}

/**
 * barcode_do_level
 * @brief   Bring-up diagnosis: the barcode DO level now.
 * @return  1 = black under the sensor, 0 = white.
 * @context line_task (console line while the motors are off)
 */
uint32_t barcode_do_level(void)
{
    bool const b_pin_high = (0u != (in_w(GPIO_IN) & BAR_DO_PIN_MASK));

    return (BAR_DO_BLACK_IS_HIGH == b_pin_high) ? 1u : 0u;
}

/* ---- private functions ------------------------------------------------ */

/**
 * barcode_init_state
 * @brief   Resets the edge ring, packed decoder state, and default map.
 * @return  none
 * @context usermain / host test reset
 */
static void barcode_init_state(void)
{
    uint32_t idx = 0u;

    g_bar_head       = 0u;
    g_bar_tail       = 0u;
    g_bar_overflow   = 0u;
    g_bar_last_us    = 0u;
    g_bar_branch_seq = 0u;
    g_bar_prv_a      = 0u;
    g_bar_prv_b      = 0u;
    g_bar_fix_cnt    = 0u;
    g_bar_carry_16us = 0u;

    for (idx = 0u; idx < BAR_MAP_ENTRIES; idx++)
    {
        g_bar_map[idx] = g_bar_default_map[idx];
    }

    for (idx = 0u; idx < BAR_CAPTURE_MAX; idx++)
    {
        g_bar_widths[idx] = 0u;
    }
}

/**
 * barcode_mark_set
 * @brief   Packs path into PRV_BAR_B_MARK (17-bit RNG) and flags clamp fault.
 * @param   path  Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_mark_set(uint32_t path)
{
    bool           b_clamped = false;
    uint32_t const mark_enc  = app_bits_clamp_u(
        path, (uint32_t) PRV_BAR_B_MARK_MAX, &b_clamped);

    if (b_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
    else
    {
        /* Path fits 17-bit mark field. */
    }

    g_bar_prv_b = app_bits_put(g_bar_prv_b, PRV_BAR_B_MARK_POS,
                               PRV_BAR_B_MARK_BITS, mark_enc);
}

/**
 * barcode_mark_get
 * @brief   Unpacks the 17-bit path mark from PRV_BAR_B_MARK.
 * @return  Stored path mark in counts.
 * @context barcode_task
 */
static uint32_t barcode_mark_get(void)
{
    return app_bits_get(g_bar_prv_b, PRV_BAR_B_MARK_POS, PRV_BAR_B_MARK_BITS);
}

/**
 * barcode_state_set
 * @brief   Updates STATE and N_ELEM inside g_bar_prv_a.
 * @param   state   Decoder state (BAR_STATE_*).
 * @param   n_elem  Captured element count (0..63).
 * @return  none
 * @context barcode_task
 */
static void barcode_state_set(uint32_t state, uint32_t n_elem)
{
    uint32_t word = g_bar_prv_a;

    word = app_bits_put(word, PRV_BAR_A_STATE_POS,
                        PRV_BAR_A_STATE_BITS, state);
    word = app_bits_put(word, PRV_BAR_A_N_ELEM_POS,
                        PRV_BAR_A_N_ELEM_BITS, n_elem);
    g_bar_prv_a = word;
}

/**
 * barcode_edge_isr
 * @brief   R1 GPIO interrupt callback for PIN_BAR_DO (both edges).
 *          Packs MSG_EDGE (15-bit gap + 1-bit level) into one uint16_t write.
 * @param   events  GPIO interrupt edge mask (unused).
 * @param   now_us  1 MHz timestamp captured at R1 entry.
 * @return  none
 * @context ISR (R1 gpio_bank0_isr)
 */
static void barcode_edge_isr(uint32_t events, uint32_t now_us)
{
    uint32_t const dt_us = now_us - g_bar_last_us;

    (void) events; /* both edges enabled; level is read from the pin */

    if (dt_us >= BAR_EDGE_REJECT_US)
    {
        uint32_t       gap        = dt_us >> BAR_GAP_SHIFT;
        bool const     b_pin_high = (0u != (in_w(GPIO_IN) & BAR_DO_PIN_MASK));
        uint32_t const level =
            (BAR_DO_BLACK_IS_HIGH == b_pin_high) ? 1u : 0u;
        uint8_t const  head = g_bar_head;
        /* cast: masked to 0..63 so fits in uint8_t */
        uint8_t const  next =
            (uint8_t) (((uint32_t) head + 1u) & (BAR_RING_ENTRIES - 1u));

        if (gap > (uint32_t) MSG_EDGE_GAP_MAX)
        {
            gap = (uint32_t) MSG_EDGE_GAP_MAX;
        }
        else
        {
            /* Fits in 15-bit MSG_EDGE_GAP domain. */
        }
        g_bar_last_us = now_us;
        g_bar_edge_cnt++;   /* aligned 32-bit, ISR is the only writer */

        if (next == g_bar_tail)
        {
            g_bar_overflow =
                (g_bar_overflow < (uint8_t) SH_BAR_OVF_OVERFLOW_MAX)
                    ? ((uint8_t) (g_bar_overflow + 1u)) /* cast: <= 255 */
                    : (uint8_t) SH_BAR_OVF_OVERFLOW_MAX;
        }
        else
        {
            uint32_t entry = app_bits_put(0u, MSG_EDGE_GAP_POS,
                                          MSG_EDGE_GAP_BITS, gap);

            entry = app_bits_put(entry, MSG_EDGE_LEVEL_POS,
                                 MSG_EDGE_LEVEL_BITS, level);
            /* cast: MSG_EDGE_USED_BITS <= 16 by layout check */
            g_bar_ring[head] = (uint16_t) entry;
            g_bar_head       = next;
            /* E_QOVR when semaphore is already at maxsem is benign. */
            (void) tk_sig_sem(gh_bar_sem, 1);
        }
    }
    else
    {
        /* Chatter filter rejected edge (< 1000 us). */
    }
}

/**
 * barcode_task
 * @brief   Drains the K2 edge ring on SEM_BAR wake or 10 ms timeout and
 *          drives the Code 39 decoder state machine.
 * @param   stacd    Start code from tk_sta_tsk (unused).
 * @param   p_exinf  Extended information (unused).
 * @return  never returns
 * @context barcode_task
 */
static void barcode_task(INT stacd, void * p_exinf)
{
    (void) stacd;   /* unused: kernel start code */
    (void) p_exinf; /* unused: no extended info  */

    for (;;)
    {
        (void) tk_wai_sem(gh_bar_sem, 1, BAR_SEM_TIMEOUT_MS);
        barcode_cycle();
    }
}

/**
 * barcode_cycle
 * @brief   Executes one wake pass of barcode_task (ring drain + timeouts).
 * @return  none
 * @context barcode_task
 */
static void barcode_cycle(void)
{
    status_board_t snap    = {0};
    uint32_t       path    = 0u;
    uint32_t       cpw     = 0u;
    bool const     b_reset = mission_mode_is_set(APP_MODE_RUN_START);
    bool const     b_inhib = mission_mode_is_set(APP_MODE_BARCODE_INHIBIT);

    status_board_read(&snap);
    path = status_board_path(&snap);
    cpw  = barcode_avg_cpw(status_board_speed_l(&snap),
                           status_board_speed_r(&snap));

    if (b_reset || b_inhib)
    {
        g_bar_tail       = g_bar_head;
        g_bar_fix_cnt    = 0u;
        g_bar_carry_16us = 0u;
        barcode_state_set(BAR_STATE_IDLE, 0u);
        barcode_mark_set(path);
    }
    else
    {
        while (g_bar_tail != g_bar_head)
        {
            uint8_t const  tail     = g_bar_tail;
            /* cast: widen 16-bit packed MSG_EDGE word */
            uint32_t const word     = (uint32_t) g_bar_ring[tail];
            uint32_t const gap_16us = app_bits_get(word, MSG_EDGE_GAP_POS,
                                                   MSG_EDGE_GAP_BITS);
            uint32_t const level    = app_bits_get(word, MSG_EDGE_LEVEL_POS,
                                                   MSG_EDGE_LEVEL_BITS);

            /* cast: masked ring index fits in uint8_t */
            g_bar_tail =
                (uint8_t) (((uint32_t) tail + 1u) & (BAR_RING_ENTRIES - 1u));
            barcode_process_edge(gap_16us, level, cpw, path);
        }

        barcode_check_timeouts(path);
    }
}

/**
 * barcode_avg_cpw
 * @brief   Computes unsigned average wheel speed (|speed_l| + |speed_r|) / 2.
 * @param   speed_l  Left wheel speed in counts per 50 ms window.
 * @param   speed_r  Right wheel speed in counts per 50 ms window.
 * @return  Average magnitude in cpw (0..127).
 * @context barcode_task
 */
static uint32_t barcode_avg_cpw(int32_t speed_l, int32_t speed_r)
{
    /* cast: negated negative speed_l/r is positive and <= 127 */
    uint32_t const abs_l =
        (speed_l < 0) ? ((uint32_t) (-speed_l)) : ((uint32_t) speed_l);
    uint32_t const abs_r =
        (speed_r < 0) ? ((uint32_t) (-speed_r)) : ((uint32_t) speed_r);

    return ((abs_l + abs_r) >> 1u);
}

/**
 * barcode_process_edge
 * @brief   Dispatches one popped edge to the active decoder state handler.
 * @param   gap_16us  Duration of the element ending at this edge (16 us LSB).
 * @param   level     DO level after the edge (1 = black, 0 = white).
 * @param   cpw       Current average speed in counts per 50 ms window.
 * @param   path      Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_process_edge(uint32_t gap_16us, uint32_t level,
                                 uint32_t cpw, uint32_t path)
{
    uint32_t const state = app_bits_get(g_bar_prv_a, PRV_BAR_A_STATE_POS,
                                        PRV_BAR_A_STATE_BITS);

    switch (state)
    {
        case BAR_STATE_IDLE:
        {
            barcode_edge_idle(level, path);
            break;
        }
        case BAR_STATE_CAPTURING:
        {
            barcode_edge_capturing(gap_16us, level, cpw, path);
            break;
        }
        case BAR_STATE_DECODING:
        case BAR_STATE_COOLDOWN:
        {
            /* Ignore edges while in DECODING or COOLDOWN. */
            break;
        }
        default:
        {
            barcode_state_set(BAR_STATE_IDLE, 0u);
            break;
        }
    }
}

/**
 * barcode_edge_idle
 * @brief   IDLE state handler: drops the pre-barcode quiet zone on the first
 *          rising transition to black (level == 1) and enters CAPTURING.
 * @param   level  DO level after the edge (1 = black, 0 = white).
 * @param   path   Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_edge_idle(uint32_t level, uint32_t path)
{
    if (1u == level)
    {
        uint32_t word = 0u;

        g_bar_fix_cnt    = 0u;
        g_bar_carry_16us = 0u;
        word = app_bits_put(word, PRV_BAR_A_STATE_POS,
                            PRV_BAR_A_STATE_BITS, BAR_STATE_CAPTURING);
        word = app_bits_put(word, PRV_BAR_A_N_ELEM_POS,
                            PRV_BAR_A_N_ELEM_BITS, 0u);
        word = app_bits_put(word, PRV_BAR_A_LAST_LEVEL_POS,
                            PRV_BAR_A_LAST_LEVEL_BITS, 1u);
        g_bar_prv_a = word;
        barcode_mark_set(path);
    }
    else
    {
        /* Still white; wait for the leading edge of the first black bar. */
    }
}

/**
 * barcode_store_width
 * @brief   Stores width into g_bar_widths, sliding the 29-element window
 *          left by one when n_elem >= BAR_ELEMENTS so a barcode preceded
 *          by 1-3 stray edges still decodes on its 29th bar/space.
 * @param   n_elem  Number of elements captured before this edge.
 * @param   width   Element width in 0.1 mm units (0..255).
 * @return  none
 * @context barcode_task
 */
static void barcode_store_width(uint32_t n_elem, uint8_t width)
{
    if (n_elem < BAR_ELEMENTS)
    {
        g_bar_widths[n_elem] = width;
    }
    else
    {
        uint32_t idx = 0u;

        for (idx = 0u; idx < (BAR_ELEMENTS - 1u); idx++)
        {
            g_bar_widths[idx] = g_bar_widths[idx + 1u];
        }
        g_bar_widths[BAR_ELEMENTS - 1u] = width;
    }
}

/**
 * barcode_edge_capturing
 * @brief   CAPTURING state handler: converts gap_16us to 0.1 mm width, checks
 *          for LM393 chatter/dropped edges, 14 mm branch lines (F-03), and
 *          29-element sliding-window immediate decode (R08).
 * @param   gap_16us  Duration of the completed element (16 us units).
 * @param   level     DO level after the edge (1 = black, 0 = white).
 * @param   cpw       Current average wheel speed in cpw.
 * @param   path      Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_edge_capturing(uint32_t gap_16us, uint32_t level,
                                   uint32_t cpw, uint32_t path)
{
    uint32_t const last_lvl = app_bits_get(
        g_bar_prv_a, PRV_BAR_A_LAST_LEVEL_POS, PRV_BAR_A_LAST_LEVEL_BITS);
    uint32_t n_elem = app_bits_get(
        g_bar_prv_a, PRV_BAR_A_N_ELEM_POS, PRV_BAR_A_N_ELEM_BITS);
    uint32_t lvl = level; /* DO level after the edge, maybe fixed */
    uint32_t const w_own =
        ((gap_16us * cpw) * APP_UM_PER_COUNT) / BAR_WIDTH_DIVISOR;
    uint32_t gap_all = gap_16us + g_bar_carry_16us;

    if (gap_all > (uint32_t) MSG_EDGE_GAP_MAX)
    {
        gap_all = (uint32_t) MSG_EDGE_GAP_MAX;
    }
    else
    {
        /* Fits the 15-bit MSG_EDGE_GAP domain. */
    }

    if (w_own < BAR_GLITCH_01MM)
    {
        /* Flicker at a bar edge: no element; its time joins the next. */
        g_bar_carry_16us = gap_all;
        barcode_mark_set(path);
    }
    else
    {
        g_bar_carry_16us = 0u;
        if ((lvl == last_lvl) && (g_bar_fix_cnt < BAR_LEVEL_FIX_MAX))
        {
            g_bar_fix_cnt++;
            lvl = (0u == last_lvl) ? 1u : 0u;
        }
        else
        {
            /* Level alternates as expected (or fix limit reached). */
        }

        if (lvl == last_lvl)
        {
            if (n_elem >= BAR_LOG_MIN_ELEM)
            {
                /* Bring-up diagnosis. cast: count <= 63 */
                app_log("BAR capture dropped: edge missed after %d elements\n",
                        (int32_t) n_elem, 0, 0);
            }
            else
            {
                /* Short noise capture dropped silently. */
            }
            barcode_state_set(BAR_STATE_IDLE, 0u);
        }
        else
        {
            uint32_t width_01mm =
                ((gap_all * cpw) * APP_UM_PER_COUNT) / BAR_WIDTH_DIVISOR;

            if (width_01mm > (uint32_t) PRV_BAR_W_WIDTH_MAX)
            {
                width_01mm = (uint32_t) PRV_BAR_W_WIDTH_MAX;
            }
            else
            {
                /* Width fits 8-bit 0.1 mm slot. */
            }

            /* cast: clamped to <= 255 above */
            barcode_store_width(n_elem, (uint8_t) width_01mm);
            if (n_elem < (uint32_t) PRV_BAR_A_N_ELEM_MAX)
            {
                n_elem++;
            }
            else
            {
                /* Saturate at 63 (6-bit N_ELEM). */
            }

            g_bar_prv_a = app_bits_put(g_bar_prv_a, PRV_BAR_A_LAST_LEVEL_POS,
                                       PRV_BAR_A_LAST_LEVEL_BITS, lvl);
            barcode_mark_set(path);

            if ((1u == n_elem) && (width_01mm >= BAR_BRANCH_MIN_01MM))
            {
                g_bar_branch_seq++;
                barcode_state_set(BAR_STATE_IDLE, 0u);
            }
            else if (n_elem >= BAR_ELEMENTS)
            {
                barcode_state_set(BAR_STATE_DECODING, n_elem);
                barcode_step_decoding(path);
            }
            else
            {
                barcode_state_set(BAR_STATE_CAPTURING, n_elem);
            }
        }
    }
}

/**
 * barcode_step_decoding
 * @brief   DECODING state handler: runs code39_decode on the 29 widths and
 *          posts APP_EVT_BARCODE or APP_EVT_UNMAPPED on success.
 * @param   path  Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_step_decoding(uint32_t path)
{
    uint32_t char_idx = 0u;
    uint32_t n_elem   = app_bits_get(g_bar_prv_a, PRV_BAR_A_N_ELEM_POS,
                                     PRV_BAR_A_N_ELEM_BITS);

    if (CODE39_OK == code39_decode(g_bar_widths, BAR_ELEMENTS, &char_idx))
    {
        uint32_t cmd = 0u;

        if (barcode_lookup_cmd(char_idx, &cmd))
        {
            uint32_t const val = mission_evt_barcode_value(cmd, char_idx);

            (void) mission_post_event(APP_EVT_BARCODE, val);
        }
        else
        {
            (void) mission_post_event(APP_EVT_UNMAPPED, char_idx);
        }

        barcode_mark_set(path);
        barcode_state_set(BAR_STATE_COOLDOWN, n_elem);
    }
    else
    {
        /* Not valid yet: keep capturing until quiet zone. */
        barcode_state_set(BAR_STATE_CAPTURING, n_elem);
    }
}

/**
 * barcode_check_timeouts
 * @brief   Checks the 30 mm quiet-zone timeout in CAPTURING and the 150 mm
 *          cooldown distance in COOLDOWN.
 * @param   path  Current odometry path in counts.
 * @return  none
 * @context barcode_task
 */
static void barcode_check_timeouts(uint32_t path)
{
    uint32_t const state  = app_bits_get(g_bar_prv_a, PRV_BAR_A_STATE_POS,
                                         PRV_BAR_A_STATE_BITS);
    uint32_t const n_elem = app_bits_get(g_bar_prv_a, PRV_BAR_A_N_ELEM_POS,
                                         PRV_BAR_A_N_ELEM_BITS);
    uint32_t const mark   = barcode_mark_get();

    if ((BAR_STATE_CAPTURING == state) &&
        ((path - mark) >= odometry_mm_to_counts(BAR_QUIET_MM)))
    {
        if (n_elem < BAR_ELEMENTS)
        {
            if (n_elem >= BAR_LOG_MIN_ELEM)
            {
                /* Bring-up diagnosis: a barcode needs 29. cast: <= 63 */
                app_log("BAR capture dropped: %d elements (a barcode has"
                        " 29)\n", (int32_t) n_elem, 0, 0);
            }
            else
            {
                /* Fewer than 4 elements: line/branch crossing. */
            }
            barcode_state_set(BAR_STATE_IDLE, 0u);
        }
        else
        {
            (void) mission_post_event(APP_EVT_INVALID, 0u);
            barcode_mark_set(path);
            barcode_state_set(BAR_STATE_COOLDOWN, n_elem);
        }
    }
    else if ((BAR_STATE_COOLDOWN == state) &&
             ((path - mark) >= odometry_mm_to_counts(BAR_COOLDOWN_MM)))
    {
        barcode_state_set(BAR_STATE_IDLE, 0u);
    }
    else
    {
        /* No distance timeout expired on this cycle. */
    }
}

/**
 * barcode_lookup_cmd
 * @brief   Looks up a decoded character index in g_bar_map.
 * @param   char_index  Code 39 character index (0..42).
 * @param   p_cmd       Output command (APP_BAR_CMD_*) if mapped.
 * @return  true if a mapping exists for char_index, else false.
 * @context barcode_task
 */
static bool barcode_lookup_cmd(uint32_t char_index, uint32_t * p_cmd)
{
    bool     b_is_mapped = false;
    uint32_t idx         = 0u;

    for (idx = 0u; idx < BAR_MAP_ENTRIES; idx++)
    {
        uint32_t const entry    = (uint32_t) g_bar_map[idx]; /* cast: widen */
        uint32_t const map_char = app_bits_get(
            entry, PRV_BAR_MAP_CHAR_POS, PRV_BAR_MAP_CHAR_BITS);

        if (map_char == char_index)
        {
            *p_cmd = app_bits_get(entry, PRV_BAR_MAP_CMD_POS,
                                  PRV_BAR_MAP_CMD_BITS);
            b_is_mapped = true;
        }
        else
        {
            /* Entry does not match char_index. */
        }
    }

    return b_is_mapped;
}

/*** end of file ***/
