/**
 * @file    motion.c
 * @brief   The motion library: K5 MBF_MOTION_CMD, the command functions
 *          mission_task calls, and the start of motion_task.
 * @owner   Buddy 2 (motion)
 * @kind    task
 * @reqs    FR15 (motion API), NFR16 (bypass and reverse caps), FR17
 *
 * Phase 2 (baseline) representation: one K5 message is a natural-type
 * struct of 12 bytes (16 bytes of buffer each). Phase 4 replaces it with
 * the 16-bit MSG_CMD word without changing any function below.
 *
 * Tags: every accepted command carries a 2-bit tag, echoed by motion_task
 * in MOVE_DONE so mission can ignore stale completions (review R06). The
 * issuer tag is written only here, i.e. only in mission_task context.
 */

#include "motion.h"            /* own header first (BARR-C 4.3.c) */
#include "../app.h"
#include "../app_types.h"
#include "cfg_motion.h"
#include "motion_task.h"

/* ---- private constants and types ------------------------------------ */

#define MOTION_STACK_WORDS     (APP_STACK_MOTION_BYTES / 8u)
#define MOTION_TAG_MASK        (0x3u)   /* MSG_CMD TAG: 2 bits */

/* K5 message, Phase 2 baseline (Phase 4: one MSG_CMD 16-bit word). */
typedef struct
{
    uint32_t type;    /* APP_CMD_* */
    int32_t  param;   /* mm, deg or mm/s; validated before sending */
    uint32_t tag;     /* 0..3 */
} motion_cmd_t;

/* Kernel cost of one message: 4-byte header + length rounded up to 4. */
#define MOTION_CMD_MSG_BYTES   (sizeof(motion_cmd_t))           /* 12 B */
#define MOTION_MBF_ENTRY_BYTES (4u + ((MOTION_CMD_MSG_BYTES + 3u) & ~3u))
#define MOTION_MBF_BYTES       (MOTION_K5_ENTRIES * MOTION_MBF_ENTRY_BYTES)

/* ---- private data ---------------------------------------------------- */

static uint64_t g_motion_stack[MOTION_STACK_WORDS];
static uint32_t g_motion_mbf_buf[MOTION_MBF_BYTES / 4u];  /* K5 storage */
static ID       gh_motion_mbf = 0;     /* K5; 0 until motion_init() */

/* Issuer tag (layout row PRV_MOT_ISSUE). Written only by the command
   functions, i.e. only in mission_task context, never by motion_task. */
static uint8_t  g_motion_tag  = 0u;

/* ---- private function prototypes ------------------------------------- */

static bool motion_send(uint32_t type, int32_t param);
static bool motion_in_range(int32_t value, int32_t min, int32_t max);

/* ---- public functions ------------------------------------------------- */

/**
 * motion_init
 * @brief   Creates K5 MBF_MOTION_CMD on its static buffer.
 * @return  E_OK, or the kernel error from tk_cre_mbf()
 * @context usermain
 */
ER motion_init(void)
{
    T_CMBF cmbf = {0};
    ER     ercd = E_OK;
    ID     mbf  = 0;

    cmbf.mbfatr = TA_TFIFO | TA_USERBUF;
    cmbf.bufsz  = (SZ) MOTION_MBF_BYTES;      /* cast: 64 B, kernel size */
    cmbf.maxmsz = (INT) MOTION_CMD_MSG_BYTES; /* cast: 12 B, kernel size */
    cmbf.bufptr = (void *) g_motion_mbf_buf;  /* cast: kernel buffer type */

    mbf = tk_cre_mbf(&cmbf);
    if (E_OK < mbf)
    {
        gh_motion_mbf = mbf;
    }
    else
    {
        ercd = mbf;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * motion_start
 * @brief   Creates and starts motion_task on its static stack.
 * @return  E_OK or the kernel error from app_start_task()
 * @context usermain
 */
ER motion_start(void)
{
    ER ercd = E_OK;
    ID tid  = app_start_task((FP) motion_task, /* cast: kernel entry type */
                             APP_PRI_MOTION, g_motion_stack,
                             APP_STACK_MOTION_BYTES, "motion_task");

    if (E_OK > tid)
    {
        ercd = tid;   /* negative ID = kernel error code */
    }

    return ercd;
}

/**
 * motion_move_forward
 * @brief   Drive straight forward a distance at MOTION_MOVE_SPEED_MM_S.
 * @param   distance_mm  1..MOTION_MOVE_MAX_MM (2047)
 * @return  false if out of range or K5 is full (nothing sent)
 * @context mission_task
 */
bool motion_move_forward(int32_t distance_mm)
{
    bool b_is_sent = false;

    if (motion_in_range(distance_mm, 1, MOTION_MOVE_MAX_MM))
    {
        b_is_sent = motion_send(APP_CMD_FWD, distance_mm);
    }

    return b_is_sent;
}

/**
 * motion_move_backward
 * @brief   Reverse straight a distance at MOTION_REVERSE_SPEED_MM_S.
 * @param   distance_mm  1..MOTION_REVERSE_MAX_MM (150, NFR16)
 * @return  false if out of range or K5 is full (nothing sent)
 * @context mission_task
 */
bool motion_move_backward(int32_t distance_mm)
{
    bool b_is_sent = false;

    if (motion_in_range(distance_mm, 1, MOTION_REVERSE_MAX_MM))
    {
        b_is_sent = motion_send(APP_CMD_BACK, distance_mm);
    }

    return b_is_sent;
}

/**
 * motion_turn_left
 * @brief   Turn left (anticlockwise) on the spot.
 * @param   angle_deg  1..MOTION_TURN_MAX_DEG (360)
 * @return  false if out of range or K5 is full (nothing sent)
 * @context mission_task
 */
bool motion_turn_left(int32_t angle_deg)
{
    bool b_is_sent = false;

    if (motion_in_range(angle_deg, 1, MOTION_TURN_MAX_DEG))
    {
        b_is_sent = motion_send(APP_CMD_LEFT, angle_deg);
    }

    return b_is_sent;
}

/**
 * motion_turn_right
 * @brief   Turn right (clockwise) on the spot.
 * @param   angle_deg  1..MOTION_TURN_MAX_DEG (360)
 * @return  false if out of range or K5 is full (nothing sent)
 * @context mission_task
 */
bool motion_turn_right(int32_t angle_deg)
{
    bool b_is_sent = false;

    if (motion_in_range(angle_deg, 1, MOTION_TURN_MAX_DEG))
    {
        b_is_sent = motion_send(APP_CMD_RIGHT, angle_deg);
    }

    return b_is_sent;
}

/**
 * motion_follow
 * @brief   Follow the line at a speed; in FOLLOW it only changes the speed.
 * @param   speed_mm_s  0..MOTION_FOLLOW_MAX_MM_S (300)
 * @return  false if out of range or K5 is full (nothing sent)
 * @context mission_task
 */
bool motion_follow(int32_t speed_mm_s)
{
    bool b_is_sent = false;

    if (motion_in_range(speed_mm_s, 0, MOTION_FOLLOW_MAX_MM_S))
    {
        b_is_sent = motion_send(APP_CMD_FOLLOW, speed_mm_s);
    }

    return b_is_sent;
}

/**
 * motion_stop
 * @brief   Brake now, cancel the active command and the latch.
 * @return  false only if K5 is full
 * @context mission_task
 */
bool motion_stop(void)
{
    return motion_send(APP_CMD_STOP, 0);
}

/**
 * motion_reset_odometry
 * @brief   Rebase positions and path to 0 for a new run (APP_CMD_ZERO).
 *          Acted on in BRAKE; acknowledged by MOVE_DONE with its tag.
 * @return  false only if K5 is full
 * @context mission_task
 */
bool motion_reset_odometry(void)
{
    return motion_send(APP_CMD_ZERO, 0);
}

/**
 * motion_last_tag
 * @brief   Tag of the last command accepted by the functions above.
 * @return  0..3
 * @context mission_task
 */
uint32_t motion_last_tag(void)
{
    return (uint32_t) g_motion_tag;   /* cast: widening, 0..3 */
}

/**
 * motion_cmd_receive
 * @brief   Takes the next K5 command without waiting. The only way
 *          motion_task.c reads K5 (its ID stays private to this file).
 * @param   p_type   receives APP_CMD_*
 * @param   p_param  receives the parameter (mm, deg or mm/s, >= 0)
 * @param   p_tag    receives the command tag 0..3
 * @return  false when K5 is empty (outputs untouched)
 * @context motion_task
 */
bool motion_cmd_receive(uint32_t * p_type, uint32_t * p_param,
                        uint32_t * p_tag)
{
    motion_cmd_t msg        = {0u, 0, 0u};
    bool         b_has_cmd  = false;

    if ((NULL != p_type) && (NULL != p_param) && (NULL != p_tag))
    {
        /* TMO_POL: returns at once; E_TMOUT (< 0) when K5 is empty */
        b_has_cmd = (tk_rcv_mbf(gh_motion_mbf, &msg, TMO_POL)
                     == (INT) MOTION_CMD_MSG_BYTES); /* cast: 12 B */
    }

    if (b_has_cmd)
    {
        *p_type  = msg.type;
        *p_param = (uint32_t) msg.param; /* cast: validated >= 0 at send */
        *p_tag   = msg.tag;
    }

    return b_has_cmd;
}

/* ---- private functions ------------------------------------------------ */

/**
 * motion_send
 * @brief   Sends one command with the next tag (TMO_POL). The issuer tag
 *          is advanced only if the send succeeded.
 * @param   type   APP_CMD_*
 * @param   param  validated parameter
 * @return  true if K5 accepted the message
 * @context mission_task
 */
static bool motion_send(uint32_t type, int32_t param)
{
    /* cast: (tag + 1) & 3 is 0..3, fits uint8_t */
    uint8_t const next_tag = (uint8_t) ((g_motion_tag + 1u)
                                        & MOTION_TAG_MASK);
    motion_cmd_t  msg      = {type, param, (uint32_t) next_tag};
    ER            ercd     = E_OK;

    /* Never blocks: a full K5 is reported to the caller (Part C.2). */
    ercd = tk_snd_mbf(gh_motion_mbf, &msg,
                      (INT) MOTION_CMD_MSG_BYTES, /* cast: 12 B */
                      TMO_POL);
    if (E_OK == ercd)
    {
        g_motion_tag = next_tag;
    }

    return (E_OK == ercd);
}

/**
 * motion_in_range
 * @brief   min <= value <= max.
 * @param   value  argument to check
 * @param   min    lowest allowed
 * @param   max    highest allowed
 * @return  true if in range
 * @context mission_task
 */
static bool motion_in_range(int32_t value, int32_t min, int32_t max)
{
    return ((value >= min) && (value <= max));
}

/*** end of file ***/
