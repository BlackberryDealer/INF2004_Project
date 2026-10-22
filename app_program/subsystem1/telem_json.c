/**
 * @file    telem_json.c
 * @brief   Baseline JSON payload builders: integers only, no spaces, no
 *          printf. Digits are produced right-to-left with / 10 (allowed:
 *          5 Hz path, never a 10 ms hot path).
 * @owner   Buddy 1 (communications)
 * @kind    algo
 * @reqs    FR10, FR12, NFR8
 */

#include "telem_json.h"        /* own header first (BARR-C 4.3.c) */
#include "../app_types.h"
#include "../app_layout.h"
#include "../subsystem2/odometry.h"

/* ---- private constants and types ------------------------------------ */

#define TELEM_JSON_SCRATCH_BYTES  (12u)   /* "-2147483648" + spare      */
#define TELEM_JSON_RADIX          (10u)
#define TELEM_JSON_ANGLE_BASE_DEG (30u)   /* angle = 30 + 5 x step      */
#define TELEM_JSON_ANGLE_STEP_DEG (5u)

/* Output under construction: text is appended until it no longer fits. */
typedef struct
{
    char *   p_out;
    uint32_t size;
    uint32_t len;
    bool     b_is_overflow;
} telem_json_buf_t;

/* ---- private data ---------------------------------------------------- */

static char const g_json_digits[] = "0123456789";

/* ---- private function prototypes ------------------------------------- */

static void     telem_json_begin(telem_json_buf_t * p_jb, char * p_out,
                                 uint32_t size);
static uint32_t telem_json_end(telem_json_buf_t const * p_jb);
static void     telem_json_put_char(telem_json_buf_t * p_jb, char letter);
static void     telem_json_put_str(telem_json_buf_t * p_jb,
                                   char const * p_text);
static void     telem_json_put_uint(telem_json_buf_t * p_jb, uint32_t value);
static void     telem_json_put_int(telem_json_buf_t * p_jb, int32_t value);

/* ---- public functions ------------------------------------------------- */

/**
 * telem_json_state
 * @brief   State message. Speeds are converted to mm/s and the path to mm
 *          (clamped silently to the MQ_STATE odometry range; the caller
 *          reports range clamps).
 * @param   p_state  state sample (natural units)
 * @param   p_out    output text (no NUL)
 * @param   size     bytes available at p_out
 * @return  text length, or 0 if it did not fit
 * @context telem_task (5 Hz)
 */
uint32_t telem_json_state(telem_bin_state_t const * p_state, char * p_out,
                          uint32_t size)
{
    telem_json_buf_t jb     = {0};
    uint32_t         odo_mm = odometry_counts_to_mm(p_state->path_counts);

    if (odo_mm > (uint32_t) MQ_STATE_ODO_MM_MAX) /* cast: max > 0 */
    {
        odo_mm = (uint32_t) MQ_STATE_ODO_MM_MAX; /* cast: max > 0 */
    }

    telem_json_begin(&jb, p_out, size);
    telem_json_put_str(&jb, "{\"st\":");
    telem_json_put_uint(&jb, p_state->sys_state);
    telem_json_put_str(&jb, ",\"run\":");
    telem_json_put_uint(&jb, p_state->run_state);
    telem_json_put_str(&jb, ",\"mot\":");
    telem_json_put_uint(&jb, p_state->motion_state);
    telem_json_put_str(&jb, ",\"spd\":[");
    telem_json_put_int(&jb, odometry_cpw_to_mm_s(p_state->speed_l_cpw));
    telem_json_put_char(&jb, ',');
    telem_json_put_int(&jb, odometry_cpw_to_mm_s(p_state->speed_r_cpw));
    telem_json_put_str(&jb, "],\"enc\":[");
    telem_json_put_int(&jb, p_state->pos_l_counts);
    telem_json_put_char(&jb, ',');
    telem_json_put_int(&jb, p_state->pos_r_counts);
    telem_json_put_str(&jb, "],\"line\":");
    telem_json_put_uint(&jb, p_state->line_status);
    telem_json_put_str(&jb, ",\"steer\":");
    telem_json_put_int(&jb, p_state->steering);
    telem_json_put_str(&jb, ",\"odo\":");
    telem_json_put_uint(&jb, odo_mm);
    telem_json_put_str(&jb, ",\"pitch\":");
    telem_json_put_int(&jb, p_state->pitch_ddeg);
    telem_json_put_str(&jb, ",\"hump_pk\":");
    telem_json_put_uint(&jb, p_state->hump_peak_mm);
    telem_json_put_str(&jb, ",\"turn\":");
    telem_json_put_int(&jb, p_state->turn_rate_dps);
    telem_json_put_str(&jb, ",\"cls\":");
    telem_json_put_uint(&jb, p_state->motion_class);
    telem_json_put_str(&jb, ",\"flt\":");
    telem_json_put_uint(&jb, p_state->faults);
    telem_json_put_char(&jb, '}');

    return telem_json_end(&jb);
}

/**
 * telem_json_event
 * @brief   Event message.
 * @param   type   APP_EVT_*
 * @param   value  event value (D.13)
 * @param   p_out  output text (no NUL)
 * @param   size   bytes available at p_out
 * @return  text length, or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_json_event(uint32_t type, uint32_t value, char * p_out,
                          uint32_t size)
{
    telem_json_buf_t jb = {0};

    telem_json_begin(&jb, p_out, size);
    telem_json_put_str(&jb, "{\"ev\":");
    telem_json_put_uint(&jb, type);
    telem_json_put_str(&jb, ",\"v\":");
    telem_json_put_uint(&jb, value);
    telem_json_put_char(&jb, '}');

    return telem_json_end(&jb);
}

/**
 * telem_json_profile
 * @brief   Obstacle profile message ("ev" is APP_EVT_OBSTACLE).
 * @param   p_prof  profile (angle step 0..24, distances in mm)
 * @param   p_out   output text (no NUL)
 * @param   size    bytes available at p_out
 * @return  text length, or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_json_profile(telem_bin_profile_t const * p_prof, char * p_out,
                            uint32_t size)
{
    telem_json_buf_t jb = {0};

    telem_json_begin(&jb, p_out, size);
    telem_json_put_str(&jb, "{\"ev\":");
    telem_json_put_uint(&jb, APP_EVT_OBSTACLE);
    telem_json_put_str(&jb, ",\"ang\":");
    telem_json_put_uint(&jb, TELEM_JSON_ANGLE_BASE_DEG
                                 + (TELEM_JSON_ANGLE_STEP_DEG
                                    * p_prof->angle_step));
    telem_json_put_str(&jb, ",\"w\":");
    telem_json_put_uint(&jb, p_prof->width_mm);
    telem_json_put_str(&jb, ",\"near\":");
    telem_json_put_uint(&jb, p_prof->nearest_mm);
    telem_json_put_str(&jb, ",\"cl_l\":");
    telem_json_put_uint(&jb, p_prof->clear_left_mm);
    telem_json_put_str(&jb, ",\"cl_r\":");
    telem_json_put_uint(&jb, p_prof->clear_right_mm);
    telem_json_put_str(&jb, ",\"side\":");
    telem_json_put_uint(&jb, p_prof->side);
    telem_json_put_char(&jb, '}');

    return telem_json_end(&jb);
}

/**
 * telem_json_summary
 * @brief   End-of-run summary message ("ev" is APP_EVT_SUMMARY).
 * @param   p_summ  summary
 * @param   p_out   output text (no NUL)
 * @param   size    bytes available at p_out
 * @return  text length, or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_json_summary(telem_bin_summary_t const * p_summ, char * p_out,
                            uint32_t size)
{
    telem_json_buf_t jb = {0};

    telem_json_begin(&jb, p_out, size);
    telem_json_put_str(&jb, "{\"ev\":");
    telem_json_put_uint(&jb, APP_EVT_SUMMARY);
    telem_json_put_str(&jb, ",\"reason\":");
    telem_json_put_uint(&jb, p_summ->end_reason);
    telem_json_put_str(&jb, ",\"hump_pk\":");
    telem_json_put_uint(&jb, p_summ->hump_peak_mm);
    telem_json_put_str(&jb, ",\"odo\":");
    telem_json_put_uint(&jb, p_summ->odometry_mm);
    telem_json_put_str(&jb, ",\"flt\":");
    telem_json_put_uint(&jb, p_summ->faults);
    telem_json_put_str(&jb, ",\"t\":");
    telem_json_put_uint(&jb, p_summ->run_time_s);
    telem_json_put_char(&jb, '}');

    return telem_json_end(&jb);
}

/**
 * telem_json_heartbeat
 * @brief   Heartbeat message.
 * @param   uptime_s    seconds since boot
 * @param   drops       telem_drops()
 * @param   lat_max_us  sys_tick_latency_max_us()
 * @param   p_out       output text (no NUL)
 * @param   size        bytes available at p_out
 * @return  text length, or 0 if it did not fit
 * @context telem_task
 */
uint32_t telem_json_heartbeat(uint32_t uptime_s, uint32_t drops,
                              uint32_t lat_max_us, char * p_out,
                              uint32_t size)
{
    telem_json_buf_t jb = {0};

    telem_json_begin(&jb, p_out, size);
    telem_json_put_str(&jb, "{\"up\":");
    telem_json_put_uint(&jb, uptime_s);
    telem_json_put_str(&jb, ",\"drops\":");
    telem_json_put_uint(&jb, drops);
    telem_json_put_str(&jb, ",\"lat\":");
    telem_json_put_uint(&jb, lat_max_us);
    telem_json_put_char(&jb, '}');

    return telem_json_end(&jb);
}

/* ---- private functions ------------------------------------------------ */

/**
 * telem_json_begin
 * @brief   Starts an empty output.
 * @param   p_jb   output state
 * @param   p_out  output text
 * @param   size   bytes available at p_out
 * @context any
 */
static void telem_json_begin(telem_json_buf_t * p_jb, char * p_out,
                             uint32_t size)
{
    p_jb->p_out         = p_out;
    p_jb->size          = size;
    p_jb->len           = 0u;
    p_jb->b_is_overflow = false;
}

/**
 * telem_json_end
 * @brief   Result of a builder.
 * @param   p_jb  output state
 * @return  the text length, or 0 if anything did not fit
 * @context any
 */
static uint32_t telem_json_end(telem_json_buf_t const * p_jb)
{
    uint32_t len = 0u;

    if (!p_jb->b_is_overflow)
    {
        len = p_jb->len;
    }

    return len;
}

/**
 * telem_json_put_char
 * @brief   Appends one character, or marks the output as overflowed.
 * @param   p_jb    output state
 * @param   letter  character to append
 * @context any
 */
static void telem_json_put_char(telem_json_buf_t * p_jb, char letter)
{
    if (p_jb->len < p_jb->size)
    {
        p_jb->p_out[p_jb->len] = letter;
        p_jb->len++;
    }
    else
    {
        p_jb->b_is_overflow = true;
    }
}

/**
 * telem_json_put_str
 * @brief   Appends a NUL-terminated constant string (without the NUL).
 * @param   p_jb    output state
 * @param   p_text  text to append
 * @context any
 */
static void telem_json_put_str(telem_json_buf_t * p_jb, char const * p_text)
{
    uint32_t idx = 0u;

    while ('\0' != p_text[idx])
    {
        telem_json_put_char(p_jb, p_text[idx]);
        idx++;
    }
}

/**
 * telem_json_put_uint
 * @brief   Appends an unsigned decimal number. Digits are formed
 *          right-to-left in a scratch buffer (division allowed: 5 Hz).
 * @param   p_jb   output state
 * @param   value  number to append
 * @context any
 */
static void telem_json_put_uint(telem_json_buf_t * p_jb, uint32_t value)
{
    char     scratch[TELEM_JSON_SCRATCH_BYTES] = {0};
    uint32_t rest                              = value;
    uint32_t count                             = 0u;

    do
    {
        scratch[count] = g_json_digits[rest % TELEM_JSON_RADIX];
        rest           = rest / TELEM_JSON_RADIX;
        count++;
    } while ((0u != rest) && (count < TELEM_JSON_SCRATCH_BYTES));

    while (count > 0u)
    {
        count--;
        telem_json_put_char(p_jb, scratch[count]);
    }
}

/**
 * telem_json_put_int
 * @brief   Appends a signed decimal number.
 * @param   p_jb   output state
 * @param   value  number to append
 * @context any
 */
static void telem_json_put_int(telem_json_buf_t * p_jb, int32_t value)
{
    uint32_t magnitude = 0u;

    if (value < 0)
    {
        telem_json_put_char(p_jb, '-');
        /* -(value + 1) cannot overflow; adding 1 back is done unsigned */
        magnitude = ((uint32_t) (-(value + 1))) + 1u; /* cast: >= 0 */
    }
    else
    {
        magnitude = (uint32_t) value;   /* cast: value >= 0 */
    }

    telem_json_put_uint(p_jb, magnitude);
}

/*** end of file ***/
