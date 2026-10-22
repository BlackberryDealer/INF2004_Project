/**
 * @file    telem.c
 * @brief   Telemetry task: turns mission events (K7) and status-board
 *          snapshots into MQTT payloads queued for the radio task (K11).
 *          Packed representation (B1.4): K7 carries the 16-bit app event
 *          word plus K7_PROF / K7_SUMM detail bytes; K11 carries the
 *          binary payloads built by telem_bin.c.
 * @owner   Buddy 1 (communications)
 * @kind    task
 * @reqs    FR10, FR12, NFR8, NFR17 (A16 evidence)
 */

#include "telem.h"             /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "../app_layout.h"
#include "cfg_telem.h"
#include "../app_bits.h"
#include "mqtt_bridge.h"
#include "../subsystem2/fault.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define TELEM_STACK_WORDS     (APP_STACK_TELEM_BYTES / 8u)

/* Kernel message-buffer cost: 4-byte header + length rounded up to 4. */
#define TELEM_MBF_HDR_BYTES   (4u)

/* K7 packed message: 2-byte event word (+ 5 B profile or 6 B summary).
   telem_task tells them apart by length.                             */
#define TELEM_K7_WORD_BYTES   (2u)
#define TELEM_K7_PROF_LEN     (TELEM_K7_WORD_BYTES + K7_PROF_BYTES)
#define TELEM_K7_SUMM_LEN     (TELEM_K7_WORD_BYTES + K7_SUMM_BYTES)
#define TELEM_K7_MSG_BYTES    (8u)
#define TELEM_K7_MBF_BYTES    (TELEM_K7_ENTRIES \
                               * (TELEM_MBF_HDR_BYTES + TELEM_K7_MSG_BYTES))

/* K11 packed message: the binary payload itself (byte 0 = header). */
#define TELEM_K11_MSG_BYTES   (APP_MQ_PAYLOAD_MAX)
#define TELEM_K11_MBF_BYTES   (TELEM_K11_ENTRIES \
                               * (TELEM_MBF_HDR_BYTES + TELEM_K11_MSG_BYTES))

#define TELEM_US_PER_MS       (1000u)
#define TELEM_STATE_PERIOD_US (TELEM_STATE_PERIOD_MS * TELEM_US_PER_MS)
#define TELEM_WAIT_MIN_MS     (1u)
#define TELEM_TIME_HALF_RANGE (0x80000000u)  /* wrap-safe "has reached" */
#define TELEM_MM_STEP_SHIFT   (2u)           /* 4 mm fields             */

#if ((TELEM_K7_MBF_BYTES % 4u) != 0u) || ((TELEM_K11_MBF_BYTES % 4u) != 0u)
#error "Telemetry message buffers must be a multiple of 4 bytes"
#endif
#if (TELEM_K7_PROF_LEN != 7u) || (TELEM_K7_SUMM_LEN != 8u) \
    || (TELEM_K7_SUMM_LEN > TELEM_K7_MSG_BYTES)
#error "K7 detail sizes changed: update the length dispatch in telem_task"
#endif
#if (TELEM_HB_EVERY_N_STATE < 1u) || \
    ((TELEM_HB_EVERY_N_STATE - 1u) > PRV_TEL_HB_DIV_MAX)
#error "TELEM_HB_EVERY_N_STATE no longer fits PRV_TEL.HB_DIV: widen it"
#endif
#if ((MQ_STATE_BYTES + 1u) > APP_MQ_PAYLOAD_MAX)
#error "The state payload no longer fits one K11 message"
#endif

/* ---- private data ---------------------------------------------------- */

static uint64_t g_telem_stack[TELEM_STACK_WORDS];
static uint32_t g_telem_k7_mbf_buf[TELEM_K7_MBF_BYTES / 4u];
static uint32_t g_telem_k11_mbf_buf[TELEM_K11_MBF_BYTES / 4u];

static ID gh_telem_k7  = 0;
static ID gh_telem_k11 = 0;

/* Rule 1: one writer each. PRV_TEL_K7: mission_task (telem_post_*);
   PRV_TEL (K11 drops, heartbeat divider, uptime): telem_task. Readers
   load the whole word (one aligned access, atomic).                  */
static volatile uint8_t  g_tel_k7_word = 0u;
static volatile uint32_t g_tel_word    = 0u;

/* ---- private function prototypes ------------------------------------- */

static void     telem_task(INT stacd, void * p_exinf);
static bool     telem_k7_send(uint8_t const * p_msg, uint32_t len);
static void     telem_k11_put(uint8_t const * p_msg, uint32_t len);
static TMO      telem_wait_ms(uint32_t next_us, uint32_t now_us);
static void     telem_forward(uint8_t const * p_msg, uint32_t len);
static void     telem_send_state(void);
static void     telem_send_heartbeat(void);
static uint32_t telem_evt_word(uint32_t type, uint32_t value);
static uint32_t telem_evt_value_max(uint32_t type);
static void     telem_k7_put_u(app_bits_writer_t * p_wr, uint32_t value,
                               int32_t max, uint32_t bits, bool * pb_any);
static uint32_t telem_tel_get(uint32_t pos, uint32_t bits);
static void     telem_tel_put(uint32_t pos, uint32_t bits, uint32_t value);

/* ---- public functions ------------------------------------------------- */

/**
 * telem_init
 * @brief   Creates K7 MBF_TELEM_EVT and K11 MBF_TELEM on static buffers
 *          and zeroes the drop counters.
 * @return  E_OK, or the first kernel error
 * @context usermain (init phase)
 */
ER telem_init(void)
{
    T_CMBF cmbf = {0};
    ER     ercd = E_OK;

    g_tel_k7_word = 0u;
    g_tel_word    = 0u;

    cmbf.mbfatr = TA_TFIFO | TA_USERBUF;
    cmbf.bufsz  = (SZ) TELEM_K7_MBF_BYTES;   /* cast: kernel size type */
    cmbf.maxmsz = (INT) TELEM_K7_MSG_BYTES;  /* cast: kernel size type */
    cmbf.bufptr = g_telem_k7_mbf_buf;
    gh_telem_k7 = tk_cre_mbf(&cmbf);

    if (gh_telem_k7 < E_OK)
    {
        ercd = (ER) gh_telem_k7;  /* cast: negative ID is an error code */
    }
    else
    {
        cmbf.bufsz   = (SZ) TELEM_K11_MBF_BYTES;  /* cast: kernel size */
        cmbf.maxmsz  = (INT) TELEM_K11_MSG_BYTES; /* cast: kernel size */
        cmbf.bufptr  = g_telem_k11_mbf_buf;
        gh_telem_k11 = tk_cre_mbf(&cmbf);
        if (gh_telem_k11 < E_OK)
        {
            ercd = (ER) gh_telem_k11;  /* cast: negative ID is an error */
        }
    }

    return ercd;
}

/**
 * telem_start
 * @brief   Creates and starts telem_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER telem_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) telem_task, /* cast: kernel entry type */
                             APP_PRI_TELEM, g_telem_stack,
                             APP_STACK_TELEM_BYTES, "telem_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * telem_post_event
 * @brief   Queues an event for telemetry (K7, TMO_POL): the 16-bit app
 *          event word. The value is clamped to the type's own width
 *          (layout EVT_* groups; types without a group carry 0), as
 *          mission_post_event does.
 * @param   type   APP_EVT_*
 * @param   value  event value (D.13)
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_event(uint32_t type, uint32_t value)
{
    uint8_t           msg[TELEM_K7_MSG_BYTES] = {0u};
    app_bits_writer_t wr = {NULL, 0u, 0u, false};

    app_bits_writer_init(&wr, msg, sizeof(msg));
    app_bits_write(&wr, telem_evt_word(type, value), MSG_EVT_USED_BITS);

    return telem_k7_send(msg, app_bits_writer_bytes(&wr));
}

/**
 * telem_post_profile
 * @brief   Queues an obstacle profile (K7, TMO_POL): an OBSTACLE event
 *          word, then the K7_PROF fields. Distances are stored in 4 mm
 *          steps (SAT, silent); an angle step above 24 (RNG) sets
 *          APP_FAULT_RANGE_CLAMP.
 * @param   p_profile  obstacle profile
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_profile(telem_bin_profile_t const * p_profile)
{
    uint8_t           msg[TELEM_K7_MSG_BYTES] = {0u};
    app_bits_writer_t wr        = {NULL, 0u, 0u, false};
    bool              b_clamped = false;

    app_bits_writer_init(&wr, msg, sizeof(msg));
    app_bits_write(&wr, telem_evt_word(APP_EVT_OBSTACLE, 0u),
                   MSG_EVT_USED_BITS);
    telem_k7_put_u(&wr, p_profile->angle_step, K7_PROF_ANGLE_STEP_MAX,
                   K7_PROF_ANGLE_STEP_BITS, &b_clamped);
    telem_k7_put_u(&wr, p_profile->width_mm >> TELEM_MM_STEP_SHIFT,
                   K7_PROF_WIDTH_MAX, K7_PROF_WIDTH_BITS, NULL);
    telem_k7_put_u(&wr, p_profile->nearest_mm >> TELEM_MM_STEP_SHIFT,
                   K7_PROF_NEAREST_MAX, K7_PROF_NEAREST_BITS, NULL);
    telem_k7_put_u(&wr, p_profile->clear_left_mm >> TELEM_MM_STEP_SHIFT,
                   K7_PROF_CLEAR_L_MAX, K7_PROF_CLEAR_L_BITS, NULL);
    telem_k7_put_u(&wr, p_profile->clear_right_mm >> TELEM_MM_STEP_SHIFT,
                   K7_PROF_CLEAR_R_MAX, K7_PROF_CLEAR_R_BITS, NULL);
    telem_k7_put_u(&wr, p_profile->side, K7_PROF_SIDE_MAX,
                   K7_PROF_SIDE_BITS, NULL);
    if (b_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    return telem_k7_send(msg, app_bits_writer_bytes(&wr));
}

/**
 * telem_post_summary
 * @brief   Queues the run summary (K7, TMO_POL): a SUMMARY event word,
 *          then the K7_SUMM fields. An RNG clamp sets
 *          APP_FAULT_RANGE_CLAMP.
 * @param   p_summary  end-of-run data
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_summary(telem_bin_summary_t const * p_summary)
{
    uint8_t           msg[TELEM_K7_MSG_BYTES] = {0u};
    app_bits_writer_t wr    = {NULL, 0u, 0u, false};
    bool              b_any = false;

    app_bits_writer_init(&wr, msg, sizeof(msg));
    app_bits_write(&wr, telem_evt_word(APP_EVT_SUMMARY, 0u),
                   MSG_EVT_USED_BITS);
    telem_k7_put_u(&wr, p_summary->end_reason, K7_SUMM_END_REASON_MAX,
                   K7_SUMM_END_REASON_BITS, NULL);
    telem_k7_put_u(&wr, p_summary->hump_peak_mm, K7_SUMM_HUMP_PEAK_MAX,
                   K7_SUMM_HUMP_PEAK_BITS, &b_any);
    telem_k7_put_u(&wr, p_summary->odometry_mm, K7_SUMM_ODO_MM_MAX,
                   K7_SUMM_ODO_MM_BITS, &b_any);
    telem_k7_put_u(&wr, p_summary->faults, K7_SUMM_FAULTS_MAX,
                   K7_SUMM_FAULTS_BITS, NULL);
    telem_k7_put_u(&wr, p_summary->run_time_s, K7_SUMM_RUN_S_MAX,
                   K7_SUMM_RUN_S_BITS, &b_any);
    if (b_any)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    return telem_k7_send(msg, app_bits_writer_bytes(&wr));
}

/**
 * telem_drops
 * @brief   K7 + K11 + publish drops, each from one atomic word read.
 * @return  0..255 (saturates)
 * @context any task
 */
uint32_t telem_drops(void)
{
    uint32_t const k7    = app_bits_get(g_tel_k7_word, PRV_TEL_K7_DROPS_POS,
                                        PRV_TEL_K7_DROPS_BITS);
    uint32_t const k11   = app_bits_get(g_tel_word, PRV_TEL_DROPS_POS,
                                        PRV_TEL_DROPS_BITS);
    uint32_t const total = k7 + k11 + mqtt_bridge_publish_drops();

    /* cast: positive layout constant 255 */
    return app_bits_clamp_u(total, (uint32_t) MQ_HB_DROPS_MAX, NULL);
}

/**
 * telem_k11_id
 * @brief   ID of K11 for mqtt_bridge.c.
 * @return  K11 ID (0 before telem_init)
 * @context radio task
 */
ID telem_k11_id(void)
{
    return gh_telem_k11;
}

/* ---- private functions ------------------------------------------------ */

/**
 * telem_task
 * @brief   Waits for a K7 message until the next 200 ms state slot, then
 *          queues the state payload, and every 5th slot the heartbeat.
 * @param   stacd    start code from tk_sta_tsk (unused)
 * @param   p_exinf  extended information (unused)
 * @return  never returns
 * @context telem_task
 */
static void telem_task(INT stacd, void * p_exinf)
{
    uint8_t  msg[TELEM_K7_MSG_BYTES] = {0u};
    uint32_t next_us = sys_tick_now_us() + TELEM_STATE_PERIOD_US;
    uint32_t now_us  = 0u;
    uint32_t hb_div  = 0u;
    INT      rcv     = 0;

    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    for (;;)
    {
        now_us = sys_tick_now_us();
        rcv    = tk_rcv_mbf(gh_telem_k7, msg, telem_wait_ms(next_us,
                                                            now_us));
        if (rcv > 0)
        {
            telem_forward(msg, (uint32_t) rcv);   /* cast: rcv > 0 */
        }

        now_us = sys_tick_now_us();
        if ((now_us - next_us) < TELEM_TIME_HALF_RANGE) /* reached: wraps */
        {
            next_us += TELEM_STATE_PERIOD_US;
            telem_send_state();
            hb_div = telem_tel_get(PRV_TEL_HB_DIV_POS, PRV_TEL_HB_DIV_BITS)
                     + 1u;
            if (hb_div >= TELEM_HB_EVERY_N_STATE)
            {
                hb_div = 0u;
                telem_send_heartbeat();
            }
            telem_tel_put(PRV_TEL_HB_DIV_POS, PRV_TEL_HB_DIV_BITS, hb_div);
        }
    }
}

/**
 * telem_k7_send
 * @brief   Sends one K7 message without waiting; counts a full K7.
 * @param   p_msg  message bytes
 * @param   len    2, 7 or 8 bytes
 * @return  true if queued
 * @context mission_task
 */
static bool telem_k7_send(uint8_t const * p_msg, uint32_t len)
{
    bool     b_is_sent = false;
    uint32_t drops     = 0u;

    if (E_OK == tk_snd_mbf(gh_telem_k7, p_msg,
                           (INT) len, /* cast: <= 8 */
                           TMO_POL))
    {
        b_is_sent = true;
    }
    else
    {
        drops = app_bits_get(g_tel_k7_word, PRV_TEL_K7_DROPS_POS,
                             PRV_TEL_K7_DROPS_BITS);
        if (drops < (uint32_t) PRV_TEL_K7_DROPS_MAX) /* cast: 255 */
        {
            /* cast: the 8-bit group fits its uint8_t word */
            g_tel_k7_word = (uint8_t) app_bits_put(g_tel_k7_word,
                                                   PRV_TEL_K7_DROPS_POS,
                                                   PRV_TEL_K7_DROPS_BITS,
                                                   drops + 1u);
        }
    }

    return b_is_sent;
}

/**
 * telem_k11_put
 * @brief   Queues one payload for the radio task without waiting. A full
 *          K11 or an empty payload (did not fit) is counted as a drop.
 * @param   p_msg  binary payload (byte 0 = header)
 * @param   len    payload length (0 = did not fit)
 * @context telem_task
 */
static void telem_k11_put(uint8_t const * p_msg, uint32_t len)
{
    bool     b_is_sent = false;
    uint32_t drops     = 0u;

    if (len > 0u)
    {
        b_is_sent = (E_OK == tk_snd_mbf(gh_telem_k11, p_msg,
                                        (INT) len, /* cast: <= 20 */
                                        TMO_POL));
    }
    if (!b_is_sent)
    {
        drops = telem_tel_get(PRV_TEL_DROPS_POS, PRV_TEL_DROPS_BITS);
        if (drops < (uint32_t) PRV_TEL_DROPS_MAX)   /* cast: 255 */
        {
            telem_tel_put(PRV_TEL_DROPS_POS, PRV_TEL_DROPS_BITS,
                          drops + 1u);
        }
    }
}

/**
 * telem_wait_ms
 * @brief   Time left until the next state slot, for tk_rcv_mbf.
 * @param   next_us  next state slot (sys_tick time)
 * @param   now_us   current sys_tick time
 * @return  1..200 ms (1 when the slot has already been reached)
 * @context telem_task (division allowed: 5 Hz path)
 */
static TMO telem_wait_ms(uint32_t next_us, uint32_t now_us)
{
    uint32_t const left_us = next_us - now_us;    /* wraps by design */
    uint32_t       wait_ms = TELEM_WAIT_MIN_MS;

    if (left_us < TELEM_TIME_HALF_RANGE)
    {
        wait_ms = (left_us + (TELEM_US_PER_MS - 1u)) / TELEM_US_PER_MS;
    }
    if (wait_ms < TELEM_WAIT_MIN_MS)
    {
        wait_ms = TELEM_WAIT_MIN_MS;
    }
    else if (wait_ms > TELEM_STATE_PERIOD_MS)
    {
        wait_ms = TELEM_STATE_PERIOD_MS;
    }
    else
    {
        /* in range */
    }

    return (TMO) wait_ms;   /* cast: 1..200 */
}

/**
 * telem_forward
 * @brief   Builds the payload for one K7 message (told apart by length:
 *          2 = event, 7 = profile, 8 = summary) and queues it in K11.
 * @param   p_msg  K7 message bytes
 * @param   len    message length
 * @context telem_task
 */
static void telem_forward(uint8_t const * p_msg, uint32_t len)
{
    uint8_t             out[TELEM_K11_MSG_BYTES] = {0u};
    uint32_t            out_len = 0u;
    uint32_t            word    = 0u;
    app_bits_reader_t   rd      = {NULL, 0u, 0u, false};
    telem_bin_profile_t prof    = {0u, 0u, 0u, 0u, 0u, 0u};
    telem_bin_summary_t summ    = {0u, 0u, 0u, 0u, 0u};

    app_bits_reader_init(&rd, p_msg, len);
    word = app_bits_read(&rd, MSG_EVT_USED_BITS);

    if (TELEM_K7_PROF_LEN == len)
    {
        prof.angle_step     = app_bits_read(&rd, K7_PROF_ANGLE_STEP_BITS);
        prof.width_mm       = app_bits_read(&rd, K7_PROF_WIDTH_BITS)
                              << TELEM_MM_STEP_SHIFT;
        prof.nearest_mm     = app_bits_read(&rd, K7_PROF_NEAREST_BITS)
                              << TELEM_MM_STEP_SHIFT;
        prof.clear_left_mm  = app_bits_read(&rd, K7_PROF_CLEAR_L_BITS)
                              << TELEM_MM_STEP_SHIFT;
        prof.clear_right_mm = app_bits_read(&rd, K7_PROF_CLEAR_R_BITS)
                              << TELEM_MM_STEP_SHIFT;
        prof.side           = app_bits_read(&rd, K7_PROF_SIDE_BITS);
        out_len             = telem_bin_profile(&prof, out, sizeof(out));
    }
    else if (TELEM_K7_SUMM_LEN == len)
    {
        summ.end_reason   = app_bits_read(&rd, K7_SUMM_END_REASON_BITS);
        summ.hump_peak_mm = app_bits_read(&rd, K7_SUMM_HUMP_PEAK_BITS);
        summ.odometry_mm  = app_bits_read(&rd, K7_SUMM_ODO_MM_BITS);
        summ.faults       = app_bits_read(&rd, K7_SUMM_FAULTS_BITS);
        summ.run_time_s   = app_bits_read(&rd, K7_SUMM_RUN_S_BITS);
        out_len           = telem_bin_summary(&summ, out, sizeof(out));
    }
    else
    {
        out_len = telem_bin_event(word, out, sizeof(out));
    }

    telem_k11_put(out, out_len);
}

/**
 * telem_send_state
 * @brief   Copies a status-board snapshot and the fault mask into a state
 *          sample and queues the state payload. An RNG clamp sets
 *          APP_FAULT_RANGE_CLAMP.
 * @context telem_task
 */
static void telem_send_state(void)
{
    status_board_t    snap      = {0};
    telem_bin_state_t state     = {0};
    uint8_t           out[TELEM_K11_MSG_BYTES] = {0u};
    uint32_t          len       = 0u;
    bool              b_clamped = false;

    status_board_read(&snap);
    state.sys_state     = status_board_sys_state(&snap);
    state.run_state     = status_board_run_state(&snap);
    state.motion_state  = status_board_motion_state(&snap);
    state.speed_l_cpw   = status_board_speed_l(&snap);
    state.speed_r_cpw   = status_board_speed_r(&snap);
    state.pos_l_counts  = status_board_pos_l(&snap);
    state.pos_r_counts  = status_board_pos_r(&snap);
    state.line_status   = status_board_line_status(&snap);
    state.steering      = status_board_steering(&snap);
    state.path_counts   = status_board_path(&snap);
    state.pitch_ddeg    = status_board_pitch(&snap);
    state.hump_peak_mm  = status_board_hump_peak(&snap);
    state.turn_rate_dps = status_board_turn_rate(&snap);
    state.motion_class  = status_board_motion_class(&snap);
    state.faults        = fault_get();

    len = telem_bin_state(&state, out, sizeof(out), &b_clamped);
    if (b_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }
    telem_k11_put(out, len);
}

/**
 * telem_send_heartbeat
 * @brief   Counts one more second of uptime and queues the heartbeat.
 *          Called on every 5th 200 ms slot of the drift-free schedule,
 *          so each call is one second of sys_tick time.
 * @context telem_task
 */
static void telem_send_heartbeat(void)
{
    uint8_t        out[TELEM_K11_MSG_BYTES] = {0u};
    uint32_t       len    = 0u;
    uint32_t const lat_us = sys_tick_latency_max_us();
    uint32_t       up_s   = telem_tel_get(PRV_TEL_UPTIME_HI_POS,
                                          PRV_TEL_UPTIME_HI_BITS);

    if (up_s < (uint32_t) PRV_TEL_UPTIME_HI_MAX)   /* cast: 65535 */
    {
        up_s++;
        telem_tel_put(PRV_TEL_UPTIME_HI_POS, PRV_TEL_UPTIME_HI_BITS, up_s);
    }

    len = telem_bin_heartbeat(up_s, telem_drops(), lat_us, out,
                              sizeof(out));
    telem_k11_put(out, len);

#if APP_TEST && APP_PROBE
    /* A16 harness (B1.1 method A): worst release latency of all slots */
    app_log("[A16] up %d s lat_max %d us drops %d",
            (int32_t) up_s,           /* cast: <= 65535 */
            (int32_t) lat_us,         /* cast: <= 4095  */
            (int32_t) telem_drops()); /* cast: <= 255   */
#endif
}

/**
 * telem_evt_word
 * @brief   The 16-bit app event word (MSG_EVT layout): value clamped to
 *          the type's width in bits 0-10, type in bits 11-15.
 * @param   type   APP_EVT_* (clamped to 0..23)
 * @param   value  event value
 * @return  the event word
 * @context any task
 */
static uint32_t telem_evt_word(uint32_t type, uint32_t value)
{
    /* casts: positive layout constants (23) */
    uint32_t const evt_type = app_bits_clamp_u(type,
                                               (uint32_t) MSG_EVT_TYPE_MAX,
                                               NULL);
    uint32_t const evt_val  = app_bits_clamp_u(value,
                                               telem_evt_value_max(evt_type),
                                               NULL);
    uint32_t       word     = 0u;

    word = app_bits_put(word, MSG_EVT_VALUE_POS, MSG_EVT_VALUE_BITS,
                        evt_val);
    word = app_bits_put(word, MSG_EVT_TYPE_POS, MSG_EVT_TYPE_BITS,
                        evt_type);

    return word;
}

/**
 * telem_evt_value_max
 * @brief   Largest value an event type carries: the width of its EVT_*
 *          layout group. Types without a group carry 0.
 * @param   type  APP_EVT_*
 * @return  2^bits - 1, or 0
 * @context any
 */
static uint32_t telem_evt_value_max(uint32_t type)
{
    uint32_t bits = 0u;

    switch (type)
    {
        case APP_EVT_JUNCTION:
            bits = EVT_JUNCTION_USED_BITS;
            break;

        case APP_EVT_LINE_FOUND:
            bits = EVT_LINE_FOUND_USED_BITS;
            break;

        case APP_EVT_BARCODE:
            bits = EVT_BARCODE_USED_BITS;
            break;

        case APP_EVT_UNMAPPED:
            bits = EVT_UNMAPPED_USED_BITS;
            break;

        case APP_EVT_PLAN:
            bits = EVT_PLAN_USED_BITS;
            break;

        case APP_EVT_MOVE_DONE:
            bits = EVT_MOVE_DONE_USED_BITS;
            break;

        case APP_EVT_STALL:
            bits = EVT_STALL_USED_BITS;
            break;

        case APP_EVT_HUMP:
            bits = EVT_HUMP_USED_BITS;
            break;

        case APP_EVT_MOTION_CLASS:
            bits = EVT_MCLASS_USED_BITS;
            break;

        case APP_EVT_CAL_DONE:
            bits = EVT_CAL_DONE_USED_BITS;
            break;

        case APP_EVT_SENSOR_FAULT:
            bits = EVT_SFAULT_USED_BITS;
            break;

        case APP_EVT_STATE_CHANGE:
            bits = EVT_STATE_USED_BITS;
            break;

        case APP_EVT_MAP:
            bits = EVT_MAP_USED_BITS;
            break;

        default:
            bits = 0u;   /* no value carried */
            break;
    }

    return ((1u << bits) - 1u);
}

/**
 * telem_k7_put_u
 * @brief   Clamps an unsigned value to [0, max] and writes it to a K7
 *          detail stream.
 * @param   p_wr    writer
 * @param   value   value in stored LSB units
 * @param   max     layout _MAX (non-negative)
 * @param   bits    layout _BITS
 * @param   pb_any  set true if this value was clamped (never cleared;
 *                  may be NULL)
 * @context mission_task
 */
static void telem_k7_put_u(app_bits_writer_t * p_wr, uint32_t value,
                           int32_t max, uint32_t bits, bool * pb_any)
{
    bool           b_clamped = false;
    /* cast: layout maxima of unsigned fields are >= 0 */
    uint32_t const raw       = app_bits_clamp_u(value, (uint32_t) max,
                                                &b_clamped);

    if (b_clamped && (NULL != pb_any))
    {
        *pb_any = true;
    }
    app_bits_write(p_wr, raw, bits);
}

/**
 * telem_tel_get
 * @brief   Reads one field of the PRV_TEL word.
 * @param   pos   layout _POS
 * @param   bits  layout _BITS
 * @return  the field
 * @context telem_task
 */
static uint32_t telem_tel_get(uint32_t pos, uint32_t bits)
{
    return app_bits_get(g_tel_word, pos, bits);
}

/**
 * telem_tel_put
 * @brief   Writes one field of the PRV_TEL word (telem_task is its only
 *          writer, so the read-modify-write needs no lock).
 * @param   pos    layout _POS
 * @param   bits   layout _BITS
 * @param   value  new field value (already within the field's range)
 * @context telem_task
 */
static void telem_tel_put(uint32_t pos, uint32_t bits, uint32_t value)
{
    g_tel_word = app_bits_put(g_tel_word, pos, bits, value);
}

/*** end of file ***/
