/**
 * @file    barcode.c
 * @brief   Barcode edge ISR, K2 ring buffer, decoder state machine, and map.
 * @owner   Buddy 3 (line, barcode)
 * @kind    task
 * @reqs    FR2, FR3, NFR5
 */

#include "barcode.h"           /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../board_pins.h"
#include "../subsystem2/odometry.h"
#include "../subsystem5/drv_gpio_irq.h"
#include "../subsystem5/mission.h"
#include "../subsystem5/status_board.h"
#include "cfg_barcode.h"
#include "code39.h"
#include "hardware/gpio.h"

/* ---- private constants and types ------------------------------------ */

#define BARCODE_STACK_WORDS  (APP_STACK_BARCODE_BYTES / 8u)
#define BAR_SEM_TIMEOUT_MS   (10)
#define BAR_GAP_SHIFT        (4u)
#define BAR_GAP_MAX          (32767u)
#define BAR_OVF_MAX          (255u)
#define BAR_WIDTH_DIVISOR    (312500u)
#define BAR_WIDTH_MAX_01MM   (255u)
#define BAR_ELEM_CNT_MAX     (63u)
#define BAR_BRANCH_MIN_01MM  (BAR_BRANCH_MIN_MM * 10u)
#define BAR_MAP_CHAR_MASK    (0x3Fu)
#define BAR_MAP_CMD_SHIFT    (6u)
#define BAR_MAP_CMD_MASK     (0x03u)
#define BAR_MAP_EMPTY        (0xFFu)
#define BAR_MAP_EMPTY_CHAR   (63u)

#define BAR_STATE_IDLE       (0u)
#define BAR_STATE_CAPTURING  (1u)
#define BAR_STATE_DECODING   (2u)
#define BAR_STATE_COOLDOWN   (3u)

/* Baseline ring entry (8 B per entry, 512 B total). */
typedef struct
{
    uint32_t gap_16us;
    uint32_t level;
} bar_edge_t;

/* ---- private data ---------------------------------------------------- */

static uint64_t            g_barcode_stack[BARCODE_STACK_WORDS];
static ID                  gh_bar_sem                   = 0;
static volatile bar_edge_t g_bar_ring[BAR_RING_ENTRIES];
static volatile uint8_t    g_bar_head                   = 0u;
static volatile uint8_t    g_bar_tail                   = 0u;
static volatile uint8_t    g_bar_overflow               = 0u;
static uint32_t            g_bar_last_us                = 0u; /* E5 */
static volatile uint8_t    g_bar_branch_seq             = 0u;
static uint32_t            g_bar_state                  = BAR_STATE_IDLE;
static uint32_t            g_bar_n_elem                 = 0u;
static uint32_t            g_bar_last_level             = 0u;
static uint32_t            g_bar_mark                   = 0u;
static uint8_t             g_bar_widths[BAR_CAPTURE_MAX];
static uint32_t            g_quiet_counts               = 0u;
static uint32_t            g_cooldown_counts            = 0u;

/* Default character-to-command map: A->LEFT, B->RIGHT, C->STRAIGHT, D->UTURN */
static uint8_t const g_bar_default_map[BAR_MAP_ENTRIES] =
{
    (uint8_t) (10u | (APP_BAR_CMD_LEFT << BAR_MAP_CMD_SHIFT)),
    (uint8_t) (11u | (APP_BAR_CMD_RIGHT << BAR_MAP_CMD_SHIFT)),
    (uint8_t) (12u | (APP_BAR_CMD_STRAIGHT << BAR_MAP_CMD_SHIFT)),
    (uint8_t) (13u | (APP_BAR_CMD_UTURN << BAR_MAP_CMD_SHIFT)),
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
static void     barcode_step_decoding(uint32_t path);
static void     barcode_check_timeouts(uint32_t path);
static bool     barcode_lookup_cmd(uint32_t char_index, uint32_t * p_cmd);

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
        /* cast: packed 6-bit char and 2-bit cmd fit in uint8_t */
        uint8_t const new_entry = (uint8_t) (
            (char_index & BAR_MAP_CHAR_MASK) |
            ((cmd & BAR_MAP_CMD_MASK) << BAR_MAP_CMD_SHIFT));
        uint32_t idx         = 0u;
        uint32_t empty_slot  = BAR_MAP_ENTRIES;
        bool     b_was_found = false;

        for (idx = 0u; idx < BAR_MAP_ENTRIES; idx++)
        {
            uint32_t const cur_char =
                ((uint32_t) g_bar_map[idx]) & BAR_MAP_CHAR_MASK; /* cast */

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

/* ---- private functions ------------------------------------------------ */

/**
 * barcode_init_state
 * @brief   Resets the edge ring, decoder state, and default command map.
 * @return  none
 * @context usermain / host test reset
 */
static void barcode_init_state(void)
{
    uint32_t idx = 0u;

    g_bar_head        = 0u;
    g_bar_tail        = 0u;
    g_bar_overflow    = 0u;
    g_bar_last_us     = 0u;
    g_bar_branch_seq  = 0u;
    g_bar_state       = BAR_STATE_IDLE;
    g_bar_n_elem      = 0u;
    g_bar_last_level  = 0u;
    g_bar_mark        = 0u;
    g_quiet_counts    = odometry_mm_to_counts(BAR_QUIET_MM);
    g_cooldown_counts = odometry_mm_to_counts(BAR_COOLDOWN_MM);

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
 * barcode_edge_isr
 * @brief   R1 GPIO interrupt callback for PIN_BAR_DO (both edges).
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
        bool const     b_pin_high = gpio_get(PIN_BAR_DO);
        uint32_t const level =
            (BAR_DO_BLACK_IS_HIGH == b_pin_high) ? 1u : 0u;
        uint8_t const  head = g_bar_head;
        /* cast: masked to 0..63 so fits in uint8_t */
        uint8_t const  next =
            (uint8_t) (((uint32_t) head + 1u) & (BAR_RING_ENTRIES - 1u));

        if (gap > BAR_GAP_MAX)
        {
            gap = BAR_GAP_MAX;
        }
        else
        {
            /* Fits in 15-bit gap domain. */
        }
        g_bar_last_us = now_us;

        if (next == g_bar_tail)
        {
            g_bar_overflow = (g_bar_overflow < BAR_OVF_MAX)
                                 ? ((uint8_t) (g_bar_overflow + 1u)) /* cast */
                                 : BAR_OVF_MAX;
        }
        else
        {
            g_bar_ring[head].gap_16us = gap;
            g_bar_ring[head].level    = level;
            g_bar_head                = next;
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

    g_quiet_counts    = odometry_mm_to_counts(BAR_QUIET_MM);
    g_cooldown_counts = odometry_mm_to_counts(BAR_COOLDOWN_MM);

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
        g_bar_tail   = g_bar_head;
        g_bar_state  = BAR_STATE_IDLE;
        g_bar_n_elem = 0u;
        g_bar_mark   = path;
    }
    else
    {
        while (g_bar_tail != g_bar_head)
        {
            uint8_t const  tail     = g_bar_tail;
            uint32_t const gap_16us = g_bar_ring[tail].gap_16us;
            uint32_t const level    = g_bar_ring[tail].level;

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
    switch (g_bar_state)
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
            g_bar_state  = BAR_STATE_IDLE;
            g_bar_n_elem = 0u;
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
        g_bar_n_elem     = 0u;
        g_bar_mark       = path;
        g_bar_last_level = 1u;
        g_bar_state      = BAR_STATE_CAPTURING;
    }
    else
    {
        /* Still white; wait for the leading edge of the first black bar. */
    }
}

/**
 * barcode_edge_capturing
 * @brief   CAPTURING state handler: converts gap_16us to 0.1 mm width, checks
 *          for dropped edges, 14 mm branch lines (F-03), and 29th-element
 *          immediate decode (R08).
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
    if (level == g_bar_last_level)
    {
        /* Two consecutive identical levels: an edge was missed -> discard. */
        g_bar_state  = BAR_STATE_IDLE;
        g_bar_n_elem = 0u;
    }
    else
    {
        /* width_um = gap16 * 16 us * (cpw * um_per_count / 50000 us), and
           / 100 for 0.1 mm => divisor = 50000 * 100 / 16 = 312500.
           Product < 2^32 because gap16 <= 32767, cpw <= 127, and
           APP_UM_PER_COUNT ~= 349 (max product ~= 1.45e9). */
        uint32_t width_01mm =
            ((gap_16us * cpw) * APP_UM_PER_COUNT) / BAR_WIDTH_DIVISOR;

        if (width_01mm > BAR_WIDTH_MAX_01MM)
        {
            width_01mm = BAR_WIDTH_MAX_01MM;
        }
        else
        {
            /* Width fits 8-bit 0.1 mm slot. */
        }

        if (g_bar_n_elem < BAR_CAPTURE_MAX)
        {
            /* cast: clamped to <= 255 above */
            g_bar_widths[g_bar_n_elem] = (uint8_t) width_01mm;
        }
        else
        {
            /* Beyond capture buffer capacity; only count elements. */
        }

        if (g_bar_n_elem < BAR_ELEM_CNT_MAX)
        {
            g_bar_n_elem++;
        }
        else
        {
            /* Saturate at 63 (6-bit N_ELEM). */
        }

        g_bar_last_level = level;
        g_bar_mark       = path;

        if ((1u == g_bar_n_elem) && (width_01mm >= BAR_BRANCH_MIN_01MM))
        {
            g_bar_branch_seq++;
            g_bar_state  = BAR_STATE_IDLE;
            g_bar_n_elem = 0u;
        }
        else if (BAR_ELEMENTS == g_bar_n_elem)
        {
            g_bar_state = BAR_STATE_DECODING;
            barcode_step_decoding(path);
        }
        else
        {
            /* Continue capturing elements. */
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

        g_bar_mark  = path;
        g_bar_state = BAR_STATE_COOLDOWN;
    }
    else
    {
        /* Not valid at 29 elements: keep capturing until quiet zone. */
        g_bar_state = BAR_STATE_CAPTURING;
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
    if ((BAR_STATE_CAPTURING == g_bar_state) &&
        ((path - g_bar_mark) >= g_quiet_counts))
    {
        if (g_bar_n_elem < BAR_ELEMENTS)
        {
            g_bar_state  = BAR_STATE_IDLE;
            g_bar_n_elem = 0u;
        }
        else
        {
            (void) mission_post_event(APP_EVT_INVALID, 0u);
            g_bar_mark  = path;
            g_bar_state = BAR_STATE_COOLDOWN;
        }
    }
    else if ((BAR_STATE_COOLDOWN == g_bar_state) &&
             ((path - g_bar_mark) >= g_cooldown_counts))
    {
        g_bar_state  = BAR_STATE_IDLE;
        g_bar_n_elem = 0u;
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
        uint32_t const map_char = entry & BAR_MAP_CHAR_MASK;

        if (map_char == char_index)
        {
            *p_cmd      = (entry >> BAR_MAP_CMD_SHIFT) & BAR_MAP_CMD_MASK;
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
