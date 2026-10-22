/**
 * @file    telem_bin.c
 * @brief   Binary MQTT payload builders. Each payload is the MQ_HDR byte
 *          then the fields of its MQ_* layout group in CSV order, written
 *          LSB-first with the app_bits stream writer; every value is
 *          clamped (never wrapped) and offset-encoded with the generated
 *          layout macros.
 * @owner   Buddy 1 (communications)
 * @kind    algo
 * @reqs    FR10, FR12, NFR8, minimum-bit items 7-8
 */

#include "telem_bin.h"         /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../app_layout.h"
#include "../app_bits.h"
#include "../subsystem2/odometry.h"

#include <stddef.h>            /* NULL */

/* ---- private constants and types ------------------------------------ */

#define TELEM_BIN_MM_STEP_SHIFT  (2u)   /* 4 mm fields: mm >> 2 (unsigned) */

/* ---- private function prototypes ------------------------------------- */

static void     telem_bin_begin(app_bits_writer_t * p_wr, uint8_t * p_buf,
                                uint32_t size, uint32_t mq_type);
static uint32_t telem_bin_end(app_bits_writer_t const * p_wr);
static void     telem_bin_put_s(app_bits_writer_t * p_wr, int32_t value,
                                int32_t min, int32_t max, int32_t offset,
                                uint32_t bits, bool * pb_any);
static void     telem_bin_put_u(app_bits_writer_t * p_wr, uint32_t value,
                                int32_t max, uint32_t bits, bool * pb_any);

/* ---- public functions ------------------------------------------------- */

/**
 * telem_bin_state
 * @brief   MQ_STATE payload. The path is converted to mm with
 *          odometry_counts_to_mm() (same value the baseline JSON sent).
 * @param   p_state         state sample (natural units)
 * @param   p_buf           output bytes
 * @param   size            bytes available at p_buf
 * @param   pb_was_clamped  set true if an RNG field was clamped, else
 *                          false (may be NULL)
 * @return  payload length (MQ_STATE_BYTES + 1), or 0 if it did not fit
 * @context telem_task (5 Hz: division allowed)
 */
uint32_t telem_bin_state(telem_bin_state_t const * p_state, uint8_t * p_buf,
                         uint32_t size, bool * pb_was_clamped)
{
    app_bits_writer_t wr     = {NULL, 0u, 0u, false};
    bool              b_any  = false;
    uint32_t const    odo_mm = odometry_counts_to_mm(p_state->path_counts);

    telem_bin_begin(&wr, p_buf, size, APP_MQ_TYPE_STATE);
    telem_bin_put_u(&wr, p_state->sys_state, MQ_STATE_SYS_MAX,
                    MQ_STATE_SYS_BITS, NULL);
    telem_bin_put_u(&wr, p_state->run_state, MQ_STATE_RUN_MAX,
                    MQ_STATE_RUN_BITS, NULL);
    telem_bin_put_u(&wr, p_state->motion_state, MQ_STATE_MOT_MAX,
                    MQ_STATE_MOT_BITS, NULL);
    telem_bin_put_s(&wr, p_state->speed_l_cpw, MQ_STATE_SPEED_L_MIN,
                    MQ_STATE_SPEED_L_MAX, MQ_STATE_SPEED_L_OFFSET,
                    MQ_STATE_SPEED_L_BITS, &b_any);
    telem_bin_put_s(&wr, p_state->speed_r_cpw, MQ_STATE_SPEED_R_MIN,
                    MQ_STATE_SPEED_R_MAX, MQ_STATE_SPEED_R_OFFSET,
                    MQ_STATE_SPEED_R_BITS, &b_any);
    telem_bin_put_s(&wr, p_state->pos_l_counts, MQ_STATE_POS_L_MIN,
                    MQ_STATE_POS_L_MAX, MQ_STATE_POS_L_OFFSET,
                    MQ_STATE_POS_L_BITS, &b_any);
    telem_bin_put_s(&wr, p_state->pos_r_counts, MQ_STATE_POS_R_MIN,
                    MQ_STATE_POS_R_MAX, MQ_STATE_POS_R_OFFSET,
                    MQ_STATE_POS_R_BITS, &b_any);
    telem_bin_put_u(&wr, p_state->line_status, MQ_STATE_LINE_MAX,
                    MQ_STATE_LINE_BITS, NULL);
    telem_bin_put_s(&wr, p_state->steering, MQ_STATE_STEER_MIN,
                    MQ_STATE_STEER_MAX, MQ_STATE_STEER_OFFSET,
                    MQ_STATE_STEER_BITS, &b_any);
    telem_bin_put_u(&wr, odo_mm, MQ_STATE_ODO_MM_MAX, MQ_STATE_ODO_MM_BITS,
                    &b_any);
    telem_bin_put_s(&wr, p_state->pitch_ddeg, MQ_STATE_PITCH_MIN,
                    MQ_STATE_PITCH_MAX, MQ_STATE_PITCH_OFFSET,
                    MQ_STATE_PITCH_BITS, &b_any);
    telem_bin_put_u(&wr, p_state->hump_peak_mm, MQ_STATE_HUMP_PEAK_MAX,
                    MQ_STATE_HUMP_PEAK_BITS, &b_any);
    telem_bin_put_s(&wr, p_state->turn_rate_dps, MQ_STATE_TURN_MIN,
                    MQ_STATE_TURN_MAX, MQ_STATE_TURN_OFFSET,
                    MQ_STATE_TURN_BITS, &b_any);
    telem_bin_put_u(&wr, p_state->motion_class, MQ_STATE_MCLASS_MAX,
                    MQ_STATE_MCLASS_BITS, NULL);
    telem_bin_put_u(&wr, p_state->faults, MQ_STATE_FAULTS_MAX,
                    MQ_STATE_FAULTS_BITS, NULL);
    app_bits_write(&wr, 0u, MQ_STATE_PAD_BITS);

    if (NULL != pb_was_clamped)
    {
        *pb_was_clamped = b_any;
    }

    return telem_bin_end(&wr);
}

/**
 * telem_bin_event
 * @brief   MQ_EVT payload: the 16-bit app event word.
 * @param   evt_word  MSG_EVT word (value bits 0-10, type 11-15)
 * @param   p_buf     output bytes
 * @param   size      bytes available at p_buf
 * @return  payload length (3), or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_bin_event(uint32_t evt_word, uint8_t * p_buf, uint32_t size)
{
    app_bits_writer_t wr = {NULL, 0u, 0u, false};

    telem_bin_begin(&wr, p_buf, size, APP_MQ_TYPE_EVENT);
    telem_bin_put_u(&wr, evt_word, MQ_EVT_WORD_MAX, MQ_EVT_WORD_BITS, NULL);

    return telem_bin_end(&wr);
}

/**
 * telem_bin_profile
 * @brief   MQ_PROF payload; distances stored in 4 mm steps (SAT 255).
 * @param   p_prof  profile (angle step 0..24, distances in mm)
 * @param   p_buf   output bytes
 * @param   size    bytes available at p_buf
 * @return  payload length (6), or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_bin_profile(telem_bin_profile_t const * p_prof,
                           uint8_t * p_buf, uint32_t size)
{
    app_bits_writer_t wr = {NULL, 0u, 0u, false};

    telem_bin_begin(&wr, p_buf, size, APP_MQ_TYPE_PROFILE);
    telem_bin_put_u(&wr, p_prof->angle_step, MQ_PROF_ANGLE_STEP_MAX,
                    MQ_PROF_ANGLE_STEP_BITS, NULL);
    telem_bin_put_u(&wr, p_prof->width_mm >> TELEM_BIN_MM_STEP_SHIFT,
                    MQ_PROF_WIDTH_MAX, MQ_PROF_WIDTH_BITS, NULL);
    telem_bin_put_u(&wr, p_prof->nearest_mm >> TELEM_BIN_MM_STEP_SHIFT,
                    MQ_PROF_NEAREST_MAX, MQ_PROF_NEAREST_BITS, NULL);
    telem_bin_put_u(&wr, p_prof->clear_left_mm >> TELEM_BIN_MM_STEP_SHIFT,
                    MQ_PROF_CLEAR_L_MAX, MQ_PROF_CLEAR_L_BITS, NULL);
    telem_bin_put_u(&wr, p_prof->clear_right_mm >> TELEM_BIN_MM_STEP_SHIFT,
                    MQ_PROF_CLEAR_R_MAX, MQ_PROF_CLEAR_R_BITS, NULL);
    telem_bin_put_u(&wr, p_prof->side, MQ_PROF_SIDE_MAX, MQ_PROF_SIDE_BITS,
                    NULL);
    app_bits_write(&wr, 0u, MQ_PROF_PAD_BITS);

    return telem_bin_end(&wr);
}

/**
 * telem_bin_summary
 * @brief   MQ_SUMM payload.
 * @param   p_summ  summary
 * @param   p_buf   output bytes
 * @param   size    bytes available at p_buf
 * @return  payload length (7), or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_bin_summary(telem_bin_summary_t const * p_summ,
                           uint8_t * p_buf, uint32_t size)
{
    app_bits_writer_t wr = {NULL, 0u, 0u, false};

    telem_bin_begin(&wr, p_buf, size, APP_MQ_TYPE_SUMMARY);
    telem_bin_put_u(&wr, p_summ->end_reason, MQ_SUMM_END_REASON_MAX,
                    MQ_SUMM_END_REASON_BITS, NULL);
    telem_bin_put_u(&wr, p_summ->hump_peak_mm, MQ_SUMM_HUMP_PEAK_MAX,
                    MQ_SUMM_HUMP_PEAK_BITS, NULL);
    telem_bin_put_u(&wr, p_summ->odometry_mm, MQ_SUMM_ODO_MM_MAX,
                    MQ_SUMM_ODO_MM_BITS, NULL);
    telem_bin_put_u(&wr, p_summ->faults, MQ_SUMM_FAULTS_MAX,
                    MQ_SUMM_FAULTS_BITS, NULL);
    telem_bin_put_u(&wr, p_summ->run_time_s, MQ_SUMM_RUN_S_MAX,
                    MQ_SUMM_RUN_S_BITS, NULL);
    app_bits_write(&wr, 0u, MQ_SUMM_PAD_BITS);

    return telem_bin_end(&wr);
}

/**
 * telem_bin_heartbeat
 * @brief   MQ_HB payload (every field saturates).
 * @param   uptime_s    seconds since boot
 * @param   drops       telem_drops()
 * @param   lat_max_us  sys_tick_latency_max_us()
 * @param   p_buf       output bytes
 * @param   size        bytes available at p_buf
 * @return  payload length (6), or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_bin_heartbeat(uint32_t uptime_s, uint32_t drops,
                             uint32_t lat_max_us, uint8_t * p_buf,
                             uint32_t size)
{
    app_bits_writer_t wr = {NULL, 0u, 0u, false};

    telem_bin_begin(&wr, p_buf, size, APP_MQ_TYPE_HEARTBEAT);
    telem_bin_put_u(&wr, uptime_s, MQ_HB_UPTIME_S_MAX, MQ_HB_UPTIME_S_BITS,
                    NULL);
    telem_bin_put_u(&wr, drops, MQ_HB_DROPS_MAX, MQ_HB_DROPS_BITS, NULL);
    telem_bin_put_u(&wr, lat_max_us, MQ_HB_LAT_MAX_US_MAX,
                    MQ_HB_LAT_MAX_US_BITS, NULL);
    app_bits_write(&wr, 0u, MQ_HB_PAD_BITS);

    return telem_bin_end(&wr);
}

/**
 * telem_bin_header
 * @brief   Byte 0 of every payload (MQ_HDR): version and type.
 * @param   mq_type  APP_MQ_TYPE_* (0..31)
 * @return  APP_MQ_VERSION | (mq_type << 3)
 * @context any
 */
uint32_t telem_bin_header(uint32_t mq_type)
{
    /* cast: MQ_HDR_TYPE_MAX is the positive constant 31 */
    uint32_t const type = app_bits_clamp_u(mq_type,
                                           (uint32_t) MQ_HDR_TYPE_MAX, NULL);
    uint32_t       word = 0u;

    word = app_bits_put(word, MQ_HDR_VER_POS, MQ_HDR_VER_BITS,
                        APP_MQ_VERSION);
    word = app_bits_put(word, MQ_HDR_TYPE_POS, MQ_HDR_TYPE_BITS, type);

    return word;
}

/* ---- private functions ------------------------------------------------ */

/**
 * telem_bin_begin
 * @brief   Starts a payload: zero-filled buffer, then the MQ_HDR fields.
 * @param   p_wr     writer
 * @param   p_buf    output bytes
 * @param   size     bytes available at p_buf
 * @param   mq_type  APP_MQ_TYPE_*
 * @context any
 */
static void telem_bin_begin(app_bits_writer_t * p_wr, uint8_t * p_buf,
                            uint32_t size, uint32_t mq_type)
{
    app_bits_writer_init(p_wr, p_buf, size);
    app_bits_write(p_wr, APP_MQ_VERSION, MQ_HDR_VER_BITS);
    telem_bin_put_u(p_wr, mq_type, MQ_HDR_TYPE_MAX, MQ_HDR_TYPE_BITS, NULL);
}

/**
 * telem_bin_end
 * @brief   Result of a builder.
 * @param   p_wr  writer
 * @return  bytes written, or 0 after an overflow
 * @context any
 */
static uint32_t telem_bin_end(app_bits_writer_t const * p_wr)
{
    uint32_t len = 0u;

    if (!p_wr->b_is_overflow)
    {
        len = app_bits_writer_bytes(p_wr);
    }

    return len;
}

/**
 * telem_bin_put_s
 * @brief   Clamps a signed value, offset-encodes it and writes it.
 * @param   p_wr    writer
 * @param   value   value in stored LSB units
 * @param   min     layout _MIN
 * @param   max     layout _MAX
 * @param   offset  layout _OFFSET
 * @param   bits    layout _BITS
 * @param   pb_any  set true if this value was clamped (never cleared;
 *                  may be NULL)
 * @context any
 */
static void telem_bin_put_s(app_bits_writer_t * p_wr, int32_t value,
                            int32_t min, int32_t max, int32_t offset,
                            uint32_t bits, bool * pb_any)
{
    bool           b_clamped = false;
    uint32_t const raw       = app_bits_encode(value, min, max, offset,
                                               &b_clamped);

    if (b_clamped && (NULL != pb_any))
    {
        *pb_any = true;
    }
    app_bits_write(p_wr, raw, bits);
}

/**
 * telem_bin_put_u
 * @brief   Clamps an unsigned value to [0, max] and writes it.
 * @param   p_wr    writer
 * @param   value   value in stored LSB units
 * @param   max     layout _MAX (non-negative)
 * @param   bits    layout _BITS
 * @param   pb_any  set true if this value was clamped (never cleared;
 *                  may be NULL)
 * @context any
 */
static void telem_bin_put_u(app_bits_writer_t * p_wr, uint32_t value,
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

/*** end of file ***/
