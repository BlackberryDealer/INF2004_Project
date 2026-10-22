/**
 * @file    telem.c
 * @brief   Telemetry task: turns mission events (K7) and status-board
 *          snapshots into MQTT payloads queued for the radio task (K11).
 *          Baseline representation: natural-type K7 messages, JSON
 *          payloads.
 * @owner   Buddy 1 (communications)
 * @kind    task
 * @reqs    FR10, FR12, NFR8, NFR17 (A16 evidence)
 */

#include "telem.h"             /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "../app_layout.h"
#include "cfg_telem.h"
#include "telem_json.h"
#include "mqtt_bridge.h"
#include "../subsystem2/fault.h"
#include "../subsystem2/odometry.h"
#include "../subsystem5/status_board.h"
#include "../subsystem5/sys_tick.h"

/* ---- private constants and types ------------------------------------ */

#define TELEM_STACK_WORDS     (APP_STACK_TELEM_BYTES / 8u)

/* Kernel message-buffer cost: 4-byte header + length rounded up to 4. */
#define TELEM_MBF_HDR_BYTES   (4u)

/* K7 baseline message: kind, type, six values = 32 bytes. */
#define TELEM_K7_VALUES       (6u)
#define TELEM_K7_MSG_BYTES    (32u)
#define TELEM_K7_MBF_BYTES    (TELEM_K7_ENTRIES \
                               * (TELEM_MBF_HDR_BYTES + TELEM_K7_MSG_BYTES))

#define TELEM_K7_KIND_EVENT   (0u)
#define TELEM_K7_KIND_PROFILE (1u)
#define TELEM_K7_KIND_SUMMARY (2u)

/* K11 baseline message: byte 0 = APP_MQ_TYPE_*, then JSON text. */
#define TELEM_K11_MSG_BYTES   (MQTT_BRIDGE_TX_MAX)
#define TELEM_K11_MBF_BYTES   (TELEM_K11_ENTRIES \
                               * (TELEM_MBF_HDR_BYTES + TELEM_K11_MSG_BYTES))

#define TELEM_DROPS_MAX       (255u)
#define TELEM_UPTIME_MAX_S    (65535u)
#define TELEM_US_PER_MS       (1000u)
#define TELEM_STATE_PERIOD_US (TELEM_STATE_PERIOD_MS * TELEM_US_PER_MS)
#define TELEM_WAIT_MIN_MS     (1u)
#define TELEM_TIME_HALF_RANGE (0x80000000u)  /* wrap-safe "has reached" */

/* Profile and summary fields are 4 mm steps up to 255 (K7_PROF). */
#define TELEM_MM_PER_STEP     (4u)
#define TELEM_PROF_MM_MAX     (TELEM_MM_PER_STEP \
                               * (uint32_t) K7_PROF_WIDTH_MAX)

typedef struct
{
    uint32_t kind;                      /* TELEM_K7_KIND_* */
    uint32_t type;                      /* APP_EVT_* */
    int32_t  value[TELEM_K7_VALUES];
} telem_k7_msg_t;

#if ((TELEM_K7_MBF_BYTES % 4u) != 0u) || ((TELEM_K11_MBF_BYTES % 4u) != 0u)
#error "Telemetry message buffers must be a multiple of 4 bytes"
#endif
#if (TELEM_HB_EVERY_N_STATE < 1u)
#error "TELEM_HB_EVERY_N_STATE must be at least 1"
#endif

/* ---- private data ---------------------------------------------------- */

static uint64_t g_telem_stack[TELEM_STACK_WORDS];
static uint32_t g_telem_k7_mbf_buf[TELEM_K7_MBF_BYTES / 4u];
static uint32_t g_telem_k11_mbf_buf[TELEM_K11_MBF_BYTES / 4u];

static ID gh_telem_k7  = 0;
static ID gh_telem_k11 = 0;

/* Rule 1: one writer each. K7 drops: mission_task (telem_post_*);
   K11 drops, heartbeat divider and uptime: telem_task.             */
static volatile uint8_t g_tel_k7_drops  = 0u;
static volatile uint8_t g_tel_k11_drops = 0u;
static uint32_t         g_tel_hb_div    = 0u;
static uint32_t         g_tel_uptime_s  = 0u;

/* ---- private function prototypes ------------------------------------- */

static void     telem_task(INT stacd, void * p_exinf);
static bool     telem_k7_send(telem_k7_msg_t const * p_msg);
static void     telem_k11_put(uint8_t const * p_msg, uint32_t len);
static TMO      telem_wait_ms(uint32_t next_us, uint32_t now_us);
static void     telem_forward(telem_k7_msg_t const * p_msg);
static void     telem_send_state(void);
static void     telem_send_heartbeat(void);
static bool     telem_state_clamp(telem_bin_state_t * p_state);
static int32_t  telem_clamp_s(int32_t value, int32_t min, int32_t max,
                              bool * pb_was_clamped);
static uint32_t telem_clamp_u(uint32_t value, uint32_t max,
                              bool * pb_was_clamped);
static uint32_t telem_evt_value_max(uint32_t type);

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

    g_tel_k7_drops  = 0u;
    g_tel_k11_drops = 0u;

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
 * @brief   Queues an event for telemetry (K7, TMO_POL). The value is
 *          clamped to the type's own width (layout EVT_* groups; types
 *          without a group carry 0), as mission_post_event does.
 * @param   type   APP_EVT_*
 * @param   value  event value (D.13)
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_event(uint32_t type, uint32_t value)
{
    telem_k7_msg_t msg     = {0u, 0u, {0, 0, 0, 0, 0, 0}};
    uint32_t const val_max = telem_evt_value_max(type);

    msg.kind     = TELEM_K7_KIND_EVENT;
    msg.type     = type;
    /* cast: <= 2047 after the clamp */
    msg.value[0] = (int32_t) telem_clamp_u(value, val_max, NULL);

    return telem_k7_send(&msg);
}

/**
 * telem_post_profile
 * @brief   Queues an obstacle profile (K7, TMO_POL). Distances are
 *          clamped to the 4 mm x 255 range (SAT, silent); the angle step
 *          to 0..24 (RNG, sets APP_FAULT_RANGE_CLAMP).
 * @param   p_profile  obstacle profile
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_profile(telem_bin_profile_t const * p_profile)
{
    telem_k7_msg_t msg       = {0u, 0u, {0, 0, 0, 0, 0, 0}};
    bool           b_clamped = false;
    /* cast: positive layout constant */
    uint32_t const step_max  = (uint32_t) K7_PROF_ANGLE_STEP_MAX;

    msg.kind = TELEM_K7_KIND_PROFILE;
    msg.type = APP_EVT_OBSTACLE;
    /* casts below: every value is clamped to <= 1020 first */
    msg.value[0] = (int32_t) telem_clamp_u(p_profile->angle_step, step_max,
                                           &b_clamped);
    msg.value[1] = (int32_t) telem_clamp_u(p_profile->width_mm,
                                           TELEM_PROF_MM_MAX, NULL);
    msg.value[2] = (int32_t) telem_clamp_u(p_profile->nearest_mm,
                                           TELEM_PROF_MM_MAX, NULL);
    msg.value[3] = (int32_t) telem_clamp_u(p_profile->clear_left_mm,
                                           TELEM_PROF_MM_MAX, NULL);
    msg.value[4] = (int32_t) telem_clamp_u(p_profile->clear_right_mm,
                                           TELEM_PROF_MM_MAX, NULL);
    msg.value[5] = (int32_t) telem_clamp_u(p_profile->side,
                                           (uint32_t) K7_PROF_SIDE_MAX,
                                           NULL); /* cast: max > 0 */
    if (b_clamped)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    return telem_k7_send(&msg);
}

/**
 * telem_post_summary
 * @brief   Queues the run summary (K7, TMO_POL). Each field is clamped to
 *          its K7_SUMM range; an RNG clamp sets APP_FAULT_RANGE_CLAMP.
 * @param   p_summary  end-of-run data
 * @return  true if queued; false if K7 is full (counted)
 * @context mission_task
 */
bool telem_post_summary(telem_bin_summary_t const * p_summary)
{
    telem_k7_msg_t msg       = {0u, 0u, {0, 0, 0, 0, 0, 0}};
    bool           b_clamped = false;
    bool           b_any     = false;

    msg.kind = TELEM_K7_KIND_SUMMARY;
    msg.type = APP_EVT_SUMMARY;
    /* casts below: every max is a positive layout constant <= 32767 */
    msg.value[0] = (int32_t) telem_clamp_u(p_summary->end_reason,
                                           (uint32_t) K7_SUMM_END_REASON_MAX,
                                           NULL);
    msg.value[1] = (int32_t) telem_clamp_u(p_summary->hump_peak_mm,
                                           (uint32_t) K7_SUMM_HUMP_PEAK_MAX,
                                           &b_clamped);
    b_any        = b_clamped;
    msg.value[2] = (int32_t) telem_clamp_u(p_summary->odometry_mm,
                                           (uint32_t) K7_SUMM_ODO_MM_MAX,
                                           &b_clamped);
    b_any        = b_any || b_clamped;
    msg.value[3] = (int32_t) telem_clamp_u(p_summary->faults,
                                           (uint32_t) K7_SUMM_FAULTS_MAX,
                                           NULL);
    msg.value[4] = (int32_t) telem_clamp_u(p_summary->run_time_s,
                                           (uint32_t) K7_SUMM_RUN_S_MAX,
                                           &b_clamped);
    b_any        = b_any || b_clamped;
    if (b_any)
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    return telem_k7_send(&msg);
}

/**
 * telem_drops
 * @brief   K7 + K11 + publish drops, from single-byte reads (atomic).
 * @return  0..255 (saturates)
 * @context any task
 */
uint32_t telem_drops(void)
{
    uint32_t total = (uint32_t) g_tel_k7_drops   /* cast: widening */
                     + (uint32_t) g_tel_k11_drops /* cast: widening */
                     + mqtt_bridge_publish_drops();

    if (total > TELEM_DROPS_MAX)
    {
        total = TELEM_DROPS_MAX;
    }

    return total;
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
    telem_k7_msg_t msg     = {0u, 0u, {0, 0, 0, 0, 0, 0}};
    uint32_t       next_us = sys_tick_now_us() + TELEM_STATE_PERIOD_US;
    uint32_t       now_us  = 0u;
    INT            rcv     = 0;

    (void) stacd;     /* unused: kernel start code */
    (void) p_exinf;   /* unused: no extended info  */

    g_tel_hb_div   = 0u;
    g_tel_uptime_s = 0u;

    for (;;)
    {
        now_us = sys_tick_now_us();
        rcv    = tk_rcv_mbf(gh_telem_k7, &msg, telem_wait_ms(next_us,
                                                             now_us));
        if (rcv > 0)
        {
            telem_forward(&msg);
        }

        now_us = sys_tick_now_us();
        if ((now_us - next_us) < TELEM_TIME_HALF_RANGE) /* reached: wraps */
        {
            next_us += TELEM_STATE_PERIOD_US;
            telem_send_state();
            g_tel_hb_div++;
            if (g_tel_hb_div >= TELEM_HB_EVERY_N_STATE)
            {
                g_tel_hb_div = 0u;
                telem_send_heartbeat();
            }
        }
    }
}

/**
 * telem_k7_send
 * @brief   Sends one K7 message without waiting; counts a full K7.
 * @param   p_msg  message to send
 * @return  true if queued
 * @context mission_task
 */
static bool telem_k7_send(telem_k7_msg_t const * p_msg)
{
    bool b_is_sent = false;

    if (E_OK == tk_snd_mbf(gh_telem_k7, p_msg,
                           (INT) TELEM_K7_MSG_BYTES, /* cast: kernel size */
                           TMO_POL))
    {
        b_is_sent = true;
    }
    else if (g_tel_k7_drops < TELEM_DROPS_MAX)
    {
        g_tel_k7_drops++;
    }
    else
    {
        /* saturated at 255 */
    }

    return b_is_sent;
}

/**
 * telem_k11_put
 * @brief   Queues one payload for the radio task without waiting. A full
 *          K11 or an empty payload (did not fit) is counted as a drop.
 * @param   p_msg  K11 message (route byte + payload)
 * @param   len    message length in bytes (1 = payload did not fit)
 * @context telem_task
 */
static void telem_k11_put(uint8_t const * p_msg, uint32_t len)
{
    bool b_is_sent = false;

    if (len > 1u)
    {
        b_is_sent = (E_OK == tk_snd_mbf(gh_telem_k11, p_msg,
                                        (INT) len, /* cast: <= 160 */
                                        TMO_POL));
    }
    if ((!b_is_sent) && (g_tel_k11_drops < TELEM_DROPS_MAX))
    {
        g_tel_k11_drops++;
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
 * @brief   Builds the payload for one K7 message and queues it in K11.
 * @param   p_msg  K7 message
 * @context telem_task
 */
static void telem_forward(telem_k7_msg_t const * p_msg)
{
    uint8_t             out[TELEM_K11_MSG_BYTES] = {0u};
    char *              p_text = (char *) &out[1]; /* cast: JSON bytes */
    uint32_t const      room   = TELEM_K11_MSG_BYTES - 1u;
    uint32_t            len    = 0u;
    telem_bin_profile_t prof   = {0u, 0u, 0u, 0u, 0u, 0u};
    telem_bin_summary_t summ   = {0u, 0u, 0u, 0u, 0u};

    /* casts below: K7 values were clamped to 0..32767 when posted */
    switch (p_msg->kind)
    {
        case TELEM_K7_KIND_PROFILE:
            prof.angle_step     = (uint32_t) p_msg->value[0];
            prof.width_mm       = (uint32_t) p_msg->value[1];
            prof.nearest_mm     = (uint32_t) p_msg->value[2];
            prof.clear_left_mm  = (uint32_t) p_msg->value[3];
            prof.clear_right_mm = (uint32_t) p_msg->value[4];
            prof.side           = (uint32_t) p_msg->value[5];
            out[0]              = (uint8_t) APP_MQ_TYPE_PROFILE; /* < 32 */
            len                 = telem_json_profile(&prof, p_text, room);
            break;

        case TELEM_K7_KIND_SUMMARY:
            summ.end_reason   = (uint32_t) p_msg->value[0];
            summ.hump_peak_mm = (uint32_t) p_msg->value[1];
            summ.odometry_mm  = (uint32_t) p_msg->value[2];
            summ.faults       = (uint32_t) p_msg->value[3];
            summ.run_time_s   = (uint32_t) p_msg->value[4];
            out[0]            = (uint8_t) APP_MQ_TYPE_SUMMARY; /* < 32 */
            len               = telem_json_summary(&summ, p_text, room);
            break;

        default:   /* TELEM_K7_KIND_EVENT */
            out[0] = (uint8_t) APP_MQ_TYPE_EVENT;   /* cast: < 32 */
            len    = telem_json_event(p_msg->type,
                                      (uint32_t) p_msg->value[0],
                                      p_text, room);
            break;
    }

    telem_k11_put(out, len + 1u);
}

/**
 * telem_send_state
 * @brief   Copies a status-board snapshot and the fault mask into a state
 *          sample, applies the MQ_STATE range checks and queues the state
 *          payload.
 * @context telem_task
 */
static void telem_send_state(void)
{
    status_board_t    snap  = {0};
    telem_bin_state_t state = {0};
    uint8_t           out[TELEM_K11_MSG_BYTES] = {0u};
    uint32_t          len   = 0u;

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

    if (telem_state_clamp(&state))
    {
        fault_set(APP_FAULT_RANGE_CLAMP);
    }

    out[0] = (uint8_t) APP_MQ_TYPE_STATE;   /* cast: < 32 */
    len    = telem_json_state(&state, (char *) &out[1], /* cast: text */
                              TELEM_K11_MSG_BYTES - 1u);
    telem_k11_put(out, len + 1u);
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
    uint32_t       len     = 0u;
    uint32_t const lat_us  = sys_tick_latency_max_us();

    if (g_tel_uptime_s < TELEM_UPTIME_MAX_S)
    {
        g_tel_uptime_s++;
    }

    out[0] = (uint8_t) APP_MQ_TYPE_HEARTBEAT;   /* cast: < 32 */
    len    = telem_json_heartbeat(g_tel_uptime_s, telem_drops(), lat_us,
                                  (char *) &out[1], /* cast: text */
                                  TELEM_K11_MSG_BYTES - 1u);
    telem_k11_put(out, len + 1u);

#if APP_TEST && APP_PROBE
    /* A16 harness (B1.1 method A): worst release latency of all slots */
    app_log("[A16] up %d s lat_max %d us drops %d",
            (int32_t) g_tel_uptime_s, /* cast: <= 65535 */
            (int32_t) lat_us,         /* cast: <= 4095  */
            (int32_t) telem_drops()); /* cast: <= 255   */
#endif
}

/**
 * telem_state_clamp
 * @brief   Clamps every state field to its MQ_STATE range in place, so
 *          the JSON carries exactly what the packed payload would.
 * @param   p_state  state sample (modified)
 * @return  true if an RNG field was clamped (range assumption broken)
 * @context telem_task
 */
static bool telem_state_clamp(telem_bin_state_t * p_state)
{
    bool b_any = false;

    /* ENUM and mask fields: clamped silently (casts: maxima > 0) */
    p_state->sys_state    = telem_clamp_u(p_state->sys_state,
                                          (uint32_t) MQ_STATE_SYS_MAX, NULL);
    p_state->run_state    = telem_clamp_u(p_state->run_state,
                                          (uint32_t) MQ_STATE_RUN_MAX, NULL);
    p_state->motion_state = telem_clamp_u(p_state->motion_state,
                                          (uint32_t) MQ_STATE_MOT_MAX, NULL);
    p_state->line_status  = telem_clamp_u(p_state->line_status,
                                          (uint32_t) MQ_STATE_LINE_MAX, NULL);
    p_state->motion_class = telem_clamp_u(p_state->motion_class,
                                          (uint32_t) MQ_STATE_MCLASS_MAX,
                                          NULL);
    p_state->faults       = telem_clamp_u(p_state->faults,
                                          (uint32_t) MQ_STATE_FAULTS_MAX,
                                          NULL);

    /* RNG fields: a clamp means a range assumption was broken */
    p_state->speed_l_cpw   = telem_clamp_s(p_state->speed_l_cpw,
                                           MQ_STATE_SPEED_L_MIN,
                                           MQ_STATE_SPEED_L_MAX, &b_any);
    p_state->speed_r_cpw   = telem_clamp_s(p_state->speed_r_cpw,
                                           MQ_STATE_SPEED_R_MIN,
                                           MQ_STATE_SPEED_R_MAX, &b_any);
    p_state->pos_l_counts  = telem_clamp_s(p_state->pos_l_counts,
                                           MQ_STATE_POS_L_MIN,
                                           MQ_STATE_POS_L_MAX, &b_any);
    p_state->pos_r_counts  = telem_clamp_s(p_state->pos_r_counts,
                                           MQ_STATE_POS_R_MIN,
                                           MQ_STATE_POS_R_MAX, &b_any);
    p_state->steering      = telem_clamp_s(p_state->steering,
                                           MQ_STATE_STEER_MIN,
                                           MQ_STATE_STEER_MAX, &b_any);
    p_state->pitch_ddeg    = telem_clamp_s(p_state->pitch_ddeg,
                                           MQ_STATE_PITCH_MIN,
                                           MQ_STATE_PITCH_MAX, &b_any);
    p_state->turn_rate_dps = telem_clamp_s(p_state->turn_rate_dps,
                                           MQ_STATE_TURN_MIN,
                                           MQ_STATE_TURN_MAX, &b_any);
    p_state->hump_peak_mm  = telem_clamp_u(p_state->hump_peak_mm,
                                           (uint32_t) MQ_STATE_HUMP_PEAK_MAX,
                                           &b_any); /* cast: max > 0 */
    (void) telem_clamp_u(odometry_counts_to_mm(p_state->path_counts),
                         (uint32_t) MQ_STATE_ODO_MM_MAX, /* cast: > 0 */
                         &b_any);  /* telem_json_state clamps the text */

    return b_any;
}

/**
 * telem_clamp_s
 * @brief   Signed clamp to [min, max].
 * @param   value           value to clamp
 * @param   min             lowest valid value
 * @param   max             highest valid value
 * @param   pb_was_clamped  set true if a clamp happened (never cleared;
 *                          may be NULL)
 * @return  the clamped value
 * @context any
 */
static int32_t telem_clamp_s(int32_t value, int32_t min, int32_t max,
                             bool * pb_was_clamped)
{
    int32_t result    = value;
    bool    b_clamped = false;

    if (value < min)
    {
        result    = min;
        b_clamped = true;
    }
    else if (value > max)
    {
        result    = max;
        b_clamped = true;
    }
    else
    {
        /* in range */
    }
    if (b_clamped && (NULL != pb_was_clamped))
    {
        *pb_was_clamped = true;
    }

    return result;
}

/**
 * telem_clamp_u
 * @brief   Unsigned clamp to [0, max].
 * @param   value           value to clamp
 * @param   max             highest valid value
 * @param   pb_was_clamped  set true if a clamp happened (never cleared;
 *                          may be NULL)
 * @return  the clamped value
 * @context any
 */
static uint32_t telem_clamp_u(uint32_t value, uint32_t max,
                              bool * pb_was_clamped)
{
    uint32_t result = value;

    if (value > max)
    {
        result = max;
        if (NULL != pb_was_clamped)
        {
            *pb_was_clamped = true;
        }
    }

    return result;
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

/*** end of file ***/
