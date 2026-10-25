/**
 * @file    test_motion_task_host.c
 * @brief   Host simulation of motion_task (the Motion Executor): the cycle
 *          tests S2-U03, S2-U12, S2-U15..U18 and the encoder-glitch
 *          regression S2-U19 (proposed), run on the laptop.
 * @owner   Buddy 2 (motion)
 * @kind    test
 * @reqs    FR15, FR17, NFR10, NFR16, reviews R03 and R06
 *
 * motion_task.c is included directly so single cycles can be driven and
 * its private state inspected. Every other module it calls is replaced by
 * a mock below: K5 is a small FIFO, the encoders follow a simple wheel
 * model (applied duty / 70 = counts per 10 ms tick, about 7 counts at 500
 * permille), time advances 10 ms per cycle. odometry.c and speed_pid.c
 * are the real modules. This complements, and does not replace, the
 * on-target runs of the same IDs in Phase 3.
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* The unit under test, with all of its private state. */
#include "motion_task.c"

/* ---- simulation state ------------------------------------------------ */

#define SIM_K5_MAX        (8u)
#define SIM_EVT_MAX       (64u)
#define SIM_TICK_US       (10000u)
#define SIM_PERMILLE_PER_COUNT (70)

typedef struct
{
    uint32_t type;
    uint32_t value;
} sim_evt_t;

static uint32_t  g_sim_k5_type[SIM_K5_MAX];
static uint32_t  g_sim_k5_param[SIM_K5_MAX];
static uint32_t  g_sim_k5_tag[SIM_K5_MAX];
static uint32_t  g_sim_k5_head  = 0u;
static uint32_t  g_sim_k5_count = 0u;

static sim_evt_t g_sim_evt[SIM_EVT_MAX];
static uint32_t  g_sim_evt_count = 0u;

static uint32_t  g_sim_us        = 0u;
static uint32_t  g_sim_raw_l     = 1000000u;  /* arbitrary start images */
static uint32_t  g_sim_raw_r     = 3000000u;
static int32_t   g_sim_acc_l     = 0;
static int32_t   g_sim_acc_r     = 0;
static uint32_t  g_sim_glitch    = 0u;    /* added to the next read only */
static int32_t   g_sim_duty_l    = 0;     /* applied by drv_motor_*      */
static int32_t   g_sim_duty_r    = 0;
static bool      gb_sim_braked   = true;
static bool      gb_sim_motor_en = true;
static bool      gb_sim_safety   = false;
/* S2-U20: MOTOR_EN value the "other core" writes at the moment motion_task
   drains K5, i.e. after step 3 has read the bit. 0 = no change, 1 = set,
   2 = clear. Models mission_task changing FLG_MODE on the other core.   */
static uint32_t  g_sim_en_on_rcv = 0u;
static uint32_t  g_sim_faults    = 0u;
static status_board_motion_t g_sim_board;

/* ---- mocks of the modules motion_task.c calls ------------------------ */

ER tk_slp_tsk(TMO tmout) { (void) tmout; return E_OK; }
ID tk_get_tid(void) { return 7; }
void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
             int32_t arg_c)
{
    (void) p_format; (void) arg_a; (void) arg_b; (void) arg_c;
}
void app_check(ER ercd, char const * p_what)
{
    if (ercd < E_OK) { (void) printf("ERROR %s: %d\n", p_what, ercd); }
}

void drv_motor_init(void) { gb_sim_braked = true; }
void drv_motor_set(int32_t duty_l_permille, int32_t duty_r_permille)
{
    g_sim_duty_l = duty_l_permille;
    g_sim_duty_r = duty_r_permille;
    gb_sim_braked = false;
}
void drv_motor_brake(void)
{
    g_sim_duty_l = 0; g_sim_duty_r = 0; gb_sim_braked = true;
}
int32_t drv_encoder_init(void) { return 0; }
void drv_encoder_read(uint32_t * p_raw_l, uint32_t * p_raw_r)
{
    *p_raw_l = g_sim_raw_l + g_sim_glitch;
    *p_raw_r = g_sim_raw_r + g_sim_glitch;
    g_sim_glitch = 0u;
}
bool drv_encoder_location(uint32_t * p_pio_index, uint32_t * p_sm_l,
                          uint32_t * p_sm_r)
{
    *p_pio_index = 1u; *p_sm_l = 0u; *p_sm_r = 1u; return true;
}

ER sys_tick_register(uint32_t slot, ID task_id)
{
    (void) slot; (void) task_id; return E_OK;
}
void sys_tick_cycle_start(uint32_t slot) { (void) slot; }
uint32_t sys_tick_now_us(void) { return g_sim_us; }
void sys_health_check_in(uint32_t task_bit) { (void) task_bit; }

void status_board_read(status_board_t * p_snapshot)
{
    (void) memset(p_snapshot, 0, sizeof(*p_snapshot));
}
int32_t status_board_steering(status_board_t const * p_snap)
{
    (void) p_snap; return 0;
}
void status_board_write_motion(status_board_motion_t const * p_part)
{
    g_sim_board = *p_part;
}

bool mission_mode_is_set(uint32_t bit)
{
    return (APP_MODE_MOTOR_EN == bit) ? gb_sim_motor_en : false;
}
bool mission_post_event(uint32_t type, uint32_t value)
{
    if (g_sim_evt_count < SIM_EVT_MAX)
    {
        g_sim_evt[g_sim_evt_count].type  = type;
        g_sim_evt[g_sim_evt_count].value = value;
        g_sim_evt_count++;
    }
    return true;
}

bool safety_is_active(void) { return gb_sim_safety; }
void fault_set(uint32_t bits) { g_sim_faults |= bits; }

bool motion_cmd_receive(uint32_t * p_type, uint32_t * p_param,
                        uint32_t * p_tag)
{
    bool b_has = (g_sim_k5_count > 0u);

    if (1u == g_sim_en_on_rcv)
    {
        gb_sim_motor_en = true;     /* set after step 3 read it */
    }
    else if (2u == g_sim_en_on_rcv)
    {
        gb_sim_motor_en = false;    /* cleared after step 3 read it */
    }
    else
    {
        /* no change */
    }
    g_sim_en_on_rcv = 0u;

    if (b_has)
    {
        *p_type  = g_sim_k5_type[g_sim_k5_head];
        *p_param = g_sim_k5_param[g_sim_k5_head];
        *p_tag   = g_sim_k5_tag[g_sim_k5_head];
        g_sim_k5_head = (g_sim_k5_head + 1u) % SIM_K5_MAX;
        g_sim_k5_count--;
    }
    return b_has;
}

/* ---- simulation helpers ---------------------------------------------- */

static void sim_send(uint32_t type, uint32_t param, uint32_t tag)
{
    uint32_t const slot = (g_sim_k5_head + g_sim_k5_count) % SIM_K5_MAX;

    g_sim_k5_type[slot]  = type;
    g_sim_k5_param[slot] = param;
    g_sim_k5_tag[slot]   = tag;
    g_sim_k5_count++;
}

/* One wheel of the plant: counts this tick from the applied duty. */
static uint32_t sim_wheel(int32_t duty, int32_t * p_acc)
{
    int32_t step = 0;

    *p_acc += duty;
    step    = *p_acc / SIM_PERMILLE_PER_COUNT;
    *p_acc -= step * SIM_PERMILLE_PER_COUNT;
    return (uint32_t) step;   /* modular add of a signed step */
}

/* One 10 ms cycle, then the wheels move with the duty just applied. */
static void sim_cycle(void)
{
    g_sim_us += SIM_TICK_US;
    motion_cycle();
    g_sim_raw_l += sim_wheel(g_sim_duty_l, &g_sim_acc_l);
    g_sim_raw_r += sim_wheel(g_sim_duty_r, &g_sim_acc_r);
}

/* One late cycle: dt_ticks x 10 ms elapse (the wheels keep turning for
   all of them) before motion_task runs once.                          */
static void sim_late_cycle(uint32_t dt_ticks)
{
    uint32_t idx = 0u;

    for (idx = 1u; idx < dt_ticks; idx++)
    {
        g_sim_us    += SIM_TICK_US;
        g_sim_raw_l += sim_wheel(g_sim_duty_l, &g_sim_acc_l);
        g_sim_raw_r += sim_wheel(g_sim_duty_r, &g_sim_acc_r);
    }
    sim_cycle();
}

static void sim_cycles(uint32_t count)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < count; idx++)
    {
        sim_cycle();
    }
}

/* Runs until a MOVE_DONE appears (or max cycles). True if one did. */
static bool sim_run_until_done(uint32_t max_cycles, uint32_t * p_tag)
{
    uint32_t const first = g_sim_evt_count;
    uint32_t       idx   = 0u;
    uint32_t       evt   = 0u;
    bool           b_done = false;

    for (idx = 0u; (idx < max_cycles) && (!b_done); idx++)
    {
        sim_cycle();
        for (evt = first; evt < g_sim_evt_count; evt++)
        {
            if (APP_EVT_MOVE_DONE == g_sim_evt[evt].type)
            {
                b_done = true;
                *p_tag = g_sim_evt[evt].value;
            }
        }
    }
    return b_done;
}

static uint32_t sim_count_events(uint32_t type, uint32_t value)
{
    uint32_t idx   = 0u;
    uint32_t count = 0u;

    for (idx = 0u; idx < g_sim_evt_count; idx++)
    {
        if ((type == g_sim_evt[idx].type) && (value == g_sim_evt[idx].value))
        {
            count++;
        }
    }
    return count;
}

/* Fresh executor and plant: as after power-on, motors allowed. */
static void sim_reset(void)
{
    g_sim_k5_head = 0u; g_sim_k5_count = 0u; g_sim_evt_count = 0u;
    g_sim_acc_l = 0; g_sim_acc_r = 0; g_sim_glitch = 0u;
    gb_sim_motor_en = true; gb_sim_safety = false; g_sim_faults = 0u;
    g_sim_en_on_rcv = 0u;
    gb_is_pend_valid = false; g_path2 = 0u; g_strk_l = 0u; g_strk_r = 0u;
    g_stall_l = 0u; g_stall_r = 0u;
    motion_cancel_active();
    motion_startup();
    sim_cycle();
}

/* ---- tests ------------------------------------------------------------ */

int main(void)
{
    uint32_t tag     = 0u;
    int32_t  pos_l   = 0;
    uint32_t prog2   = 0u;
    bool     b_done  = false;
    uint32_t raw_read = 0u;
    uint32_t raw0_before = 0u;

    HOST_INIT();

    /* S2-U03: a command sent while MOVING waits in the latch; STOP brakes
       at once (in the same cycle it is received). */
    sim_reset();
    sim_send(APP_CMD_FWD, 1000u, 1u);
    sim_cycles(5u);
    HOST_CHECK("S2-U03", APP_MOT_MOVE_FWD == g_state, "MOVE_FWD started");
    sim_send(APP_CMD_LEFT, 90u, 2u);
    sim_cycle();
    HOST_CHECK("S2-U03", (APP_MOT_MOVE_FWD == g_state) && gb_is_pend_valid
               && (APP_CMD_LEFT == g_pend_type) && (2u == g_pend_tag),
               "LEFT while moving waits in the latch");
    sim_send(APP_CMD_STOP, 0u, 3u);
    sim_cycle();
    HOST_CHECK("S2-U03", (APP_MOT_BRAKE == g_state) && gb_sim_braked
               && (!gb_is_pend_valid),
               "STOP brakes in the cycle it arrives and clears the latch");

    /* S2-U12: STOP during FOLLOW and during MOVE: braked within one cycle */
    sim_reset();
    sim_send(APP_CMD_FOLLOW, 250u, 1u);
    sim_cycles(20u);
    HOST_CHECK("S2-U12", (APP_MOT_FOLLOW == g_state) && (!gb_sim_braked)
               && (g_sim_duty_l > 0), "FOLLOW drives the wheels");
    sim_send(APP_CMD_STOP, 0u, 2u);
    sim_cycle();
    HOST_CHECK("S2-U12", (APP_MOT_BRAKE == g_state) && gb_sim_braked,
               "STOP during FOLLOW: braked after one cycle");
    sim_send(APP_CMD_FWD, 500u, 3u);
    sim_cycles(20u);
    sim_send(APP_CMD_STOP, 0u, 0u);
    sim_cycle();
    HOST_CHECK("S2-U12", (APP_MOT_BRAKE == g_state) && gb_sim_braked,
               "STOP during MOVE: braked after one cycle");

    /* S2-U15: progress is per command, never from the saturating path */
    sim_reset();
    g_path2 = MOTION_PATH2_MAX - 1000u;
    sim_send(APP_CMD_FWD, 1000u, 2u);
    b_done = sim_run_until_done(3000u, &tag);
    HOST_CHECK("S2-U15", b_done && (2u == tag),
               "1000 mm MOVE near path saturation completes, MOVE_DONE tag 2");
    HOST_CHECK("S2-U15", (MOTION_PATH2_MAX == g_path2)
               && (0u != (g_sim_faults & APP_FAULT_RANGE_CLAMP)),
               "path saturates at 2 x 131071 and reports RANGE_CLAMP");
    HOST_CHECK("S2-U15", (g_sim_board.pos_l_counts >= 2865)
               && (g_sim_board.pos_l_counts <= 2865 + 30),
               "left wheel travelled the 2865 counts of 1000 mm");

    /* S2-U16: ZERO in BRAKE rebases and acks with its tag; in FOLLOW it
       waits in the latch */
    sim_reset();
    sim_send(APP_CMD_FWD, 300u, 1u);
    (void) sim_run_until_done(1000u, &tag);
    HOST_CHECK("S2-U16", (APP_MOT_BRAKE == g_state) && (0 != g_left.pos)
               && (0u != g_path2), "run 1 moved the wheels");
    sim_send(APP_CMD_ZERO, 0u, 2u);
    sim_cycle();
    HOST_CHECK("S2-U16", (0 == g_sim_board.pos_l_counts)
               && (0 == g_sim_board.pos_r_counts)
               && (0u == g_sim_board.path_counts)
               && (1u == sim_count_events(APP_EVT_MOVE_DONE, 2u)),
               "ZERO in BRAKE: positions and path 0, MOVE_DONE tag 2");
    sim_send(APP_CMD_FOLLOW, 200u, 3u);
    sim_cycles(10u);
    sim_send(APP_CMD_ZERO, 0u, 0u);
    sim_cycle();
    HOST_CHECK("S2-U16", (APP_MOT_FOLLOW == g_state) && gb_is_pend_valid
               && (APP_CMD_ZERO == g_pend_type) && (0 != g_left.pos),
               "ZERO during FOLLOW waits in the latch");

    /* S2-U17: MOTOR_EN clear -> FOLLOW never drives; setting it later
       does not start the dropped command */
    sim_reset();
    gb_sim_motor_en = false;
    sim_send(APP_CMD_FOLLOW, 250u, 1u);
    sim_cycles(20u);
    HOST_CHECK("S2-U17", (APP_MOT_BRAKE == g_state) && gb_sim_braked,
               "FOLLOW with MOTOR_EN clear: never drives");
    gb_sim_motor_en = true;
    sim_cycles(20u);
    HOST_CHECK("S2-U17", (APP_MOT_BRAKE == g_state) && gb_sim_braked,
               "setting MOTOR_EN later does not start the dropped command");
    sim_send(APP_CMD_FOLLOW, 250u, 2u);
    sim_cycles(5u);
    sim_send(APP_CMD_STOP, 0u, 3u);
    gb_sim_motor_en = false;
    sim_cycle();
    HOST_CHECK("S2-U17", gb_sim_braked, "clearing MOTOR_EN brakes");

    /* S2-U18: STOP cancels a MOVE without MOVE_DONE; the next MOVE's
       MOVE_DONE carries the new tag */
    sim_reset();
    sim_send(APP_CMD_FWD, 1000u, 1u);
    sim_cycles(10u);
    sim_send(APP_CMD_STOP, 0u, 2u);
    sim_cycles(5u);
    sim_send(APP_CMD_FWD, 200u, 3u);
    b_done = sim_run_until_done(1000u, &tag);
    HOST_CHECK("S2-U18", b_done && (3u == tag)
               && (0u == sim_count_events(APP_EVT_MOVE_DONE, 1u)),
               "no MOVE_DONE for the cancelled MOVE; next one carries tag 3");

    /* S2-U19 (proposed): one impossible encoder read must not advance
       progress, path or position (review: a 5000-count glitch completed
       a 1 m MOVE). */
    sim_reset();
    sim_send(APP_CMD_FWD, 1000u, 1u);
    sim_cycles(20u);
    pos_l = g_left.pos;
    prog2 = g_prog2;
    g_sim_glitch = 5000u;            /* transient: next read only */
    sim_cycle();
    HOST_CHECK("S2-U19", (APP_MOT_MOVE_FWD == g_state)
               && (0u == sim_count_events(APP_EVT_MOVE_DONE, 1u)),
               "5000-count glitch does not complete the 1 m MOVE");
    HOST_CHECK("S2-U19", (g_prog2 == prog2) && (g_left.pos == pos_l),
               "rejected read adds nothing to progress or position");
    HOST_CHECK("S2-U19", 0u == (g_sim_faults & APP_FAULT_ENCODER),
               "a single glitch raises no ENCODER fault");
    b_done = sim_run_until_done(3000u, &tag);
    HOST_CHECK("S2-U19", b_done && (1u == tag)
               && (g_sim_board.pos_l_counts >= 2865)
               && (g_sim_board.pos_l_counts <= 2865 + 30),
               "MOVE then completes at the real distance (2865 counts)");

    sim_reset();
    sim_send(APP_CMD_FWD, 1000u, 1u);
    sim_cycles(20u);
    pos_l = g_left.pos;
    g_sim_raw_l += 5000u;            /* persistent counter jump */
    g_sim_raw_r += 5000u;
    sim_cycles(2u);
    HOST_CHECK("S2-U19", 0u != (g_sim_faults & APP_FAULT_ENCODER),
               "jump that persists: ENCODER fault on the 2nd read");
    HOST_CHECK("S2-U19", (0u == sim_count_events(APP_EVT_MOVE_DONE, 1u))
               && (g_left.pos < (pos_l + 100)),
               "no premature MOVE_DONE; the jump is left out of position");

    /* Late cycle + one-off glitch + valid recovery (follow-up review):
       at 300 mm/s (~8.6 counts per 10 ms) a 30 ms late read with a glitch
       is rejected; the next correct read 10 ms later holds ~34 counts of
       40 ms of movement and must be judged against 40 ms, not 10 ms.   */
    sim_reset();
    sim_send(APP_CMD_FOLLOW, 300u, 1u);
    sim_cycles(60u);
    raw0_before  = g_raw0_l;
    g_sim_glitch = 5000u;
    sim_late_cycle(3u);
    HOST_CHECK("S2-U19", (1u == g_strk_l) && (1u == g_strk_r),
               "late (30 ms) glitched read is rejected");
    raw_read = g_sim_raw_l;          /* what the recovery read returns */
    sim_cycle();
    HOST_CHECK("S2-U19", 0u == (g_sim_faults & APP_FAULT_ENCODER),
               "valid recovery read after a late tick: no ENCODER fault");
    HOST_CHECK("S2-U19", (0u == g_strk_l) && (0u == g_strk_r)
               && (g_left.delta > 30),
               "recovery read accepted with its 40 ms of movement");
    /* cast: modular difference of the simulated raw images */
    HOST_CHECK("S2-U19", (g_raw0_l == raw0_before)
               && (g_left.pos == (int32_t) (raw_read - raw0_before)),
               "no resync; position equals the counter read: nothing lost");

    /* S2-U20 (review 2026-10-26): dual-core race. mission_task sets
       MOTOR_EN and sends the command on the other core after motion_task
       read the bit in step 3 but before it drains K5 in step 4. The
       command must start, not be dropped as if the motors were off.     */
    sim_reset();
    gb_sim_motor_en = false;
    sim_cycles(3u);
    sim_send(APP_CMD_LEFT, 30u, 1u);    /* first CAL_IR sweep leg */
    g_sim_en_on_rcv = 1u;
    sim_cycle();
    HOST_CHECK("S2-U20", APP_MOT_TURN_L == g_state,
               "first calibration turn starts when MOTOR_EN was set late");
    b_done = sim_run_until_done(200u, &tag);
    HOST_CHECK("S2-U20", b_done && (1u == tag),
               "that turn completes with its MOVE_DONE");

    sim_reset();
    gb_sim_motor_en = false;
    sim_cycles(3u);
    sim_send(APP_CMD_FOLLOW, 250u, 2u); /* first RUN command */
    g_sim_en_on_rcv = 1u;
    sim_cycles(20u);
    HOST_CHECK("S2-U20", (APP_MOT_FOLLOW == g_state) && (!gb_sim_braked),
               "first FOLLOW starts and drives when MOTOR_EN was set late");

    /* The other direction: MOTOR_EN cleared after step 3 read it set.
       The command must be rejected (FR17), never driven, and not started
       later when the bit comes back.                                    */
    sim_reset();
    gb_sim_motor_en = true;
    sim_cycles(3u);
    sim_send(APP_CMD_FWD, 500u, 3u);
    g_sim_en_on_rcv = 2u;
    sim_cycle();
    HOST_CHECK("S2-U20", (APP_MOT_BRAKE == g_state) && gb_sim_braked,
               "command after MOTOR_EN cleared late is rejected, braked");
    gb_sim_motor_en = true;
    sim_cycles(5u);
    HOST_CHECK("S2-U20", (APP_MOT_BRAKE == g_state) && gb_sim_braked
               && (0u == sim_count_events(APP_EVT_MOVE_DONE, 3u)),
               "rejected command is not started when MOTOR_EN returns");

    return (0 == HOST_SUMMARY("motion_task")) ? 0 : 1;
}

/*** end of file ***/
