/**
 * test_motion_cmd_host.c
 * S2-U29 (Tribunal 1 min-bit change, gap 3): every K5 motion command is
 * one 16-bit MSG_CMD word. motion.c is compiled against a mock message
 * buffer that charges the kernel's cost (4-byte header + size rounded up
 * to 4) so the queue capacity is tested, not only the bit layout.
 * Host-only test code, outside BARR-C scope.
 * Run: make -C testing/subsystem2/host
 */

#include "host_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "motion.c"           /* unit under test, with its private data */

/* ---- mock kernel message buffer (one buffer, FIFO) -------------------- */

#define MOCK_MBF_MAX   (256u)

static uint8_t  g_q[MOCK_MBF_MAX];
static uint32_t g_q_len[16];
static uint32_t g_q_n      = 0u;
static uint32_t g_q_used   = 0u;      /* bytes incl. headers */
static int32_t  g_bufsz    = 0;
static int32_t  g_maxmsz   = 0;
static int32_t  g_max_snd  = 0;       /* largest message sent */

static uint32_t mock_cost(uint32_t size)
{
    return 4u + ((size + 3u) & ~3u);
}

ID tk_cre_mbf(T_CMBF const * pk_cmbf)
{
    g_bufsz  = (int32_t) pk_cmbf->bufsz;
    g_maxmsz = (int32_t) pk_cmbf->maxmsz;
    return 1;
}

ER tk_snd_mbf(ID mbfid, void const * msg, INT msgsz, TMO tmout)
{
    uint32_t const size = (uint32_t) msgsz;
    ER             ercd = E_TMOUT;

    (void) mbfid;
    (void) tmout;
    if ((int32_t) msgsz > g_max_snd) { g_max_snd = (int32_t) msgsz; }
    if ((msgsz <= g_maxmsz)
        && ((g_q_used + mock_cost(size)) <= (uint32_t) g_bufsz))
    {
        uint32_t off = 0u;
        for (uint32_t i = 0u; i < g_q_n; i++) { off += g_q_len[i]; }
        memcpy(&g_q[off], msg, size);
        g_q_len[g_q_n] = size;
        g_q_n++;
        g_q_used += mock_cost(size);
        ercd = E_OK;
    }
    return ercd;
}

INT tk_rcv_mbf(ID mbfid, void * msg, TMO tmout)
{
    INT size = E_TMOUT;

    (void) mbfid;
    (void) tmout;
    if (g_q_n > 0u)
    {
        uint32_t const first = g_q_len[0];
        uint32_t       rest  = 0u;
        for (uint32_t i = 1u; i < g_q_n; i++) { rest += g_q_len[i]; }
        memcpy(msg, g_q, first);
        memmove(g_q, &g_q[first], rest);
        for (uint32_t i = 1u; i < g_q_n; i++) { g_q_len[i - 1u] = g_q_len[i]; }
        g_q_n--;
        g_q_used -= mock_cost(first);
        size = (INT) first;
    }
    return size;
}

ID app_start_task(FP p_entry, PRI priority, uint64_t * p_stack,
                  uint32_t stack_bytes, char const * p_name)
{
    (void) p_entry; (void) priority; (void) p_stack; (void) stack_bytes;
    (void) p_name;
    return 1;
}

void motion_task(INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;
}

/* ---- helpers ------------------------------------------------------------ */

static bool rx_is(uint32_t type, uint32_t param, uint32_t tag)
{
    uint32_t t = 99u;
    uint32_t p = 99u;
    uint32_t g = 99u;

    return motion_cmd_receive(&t, &p, &g) && (t == type) && (p == param)
           && (g == tag);
}

int main(void)
{
    HOST_INIT();

    HOST_CHECK("S2-U29", (E_OK == motion_init()) && (32 == g_bufsz)
               && (2 == g_maxmsz),
               "K5 created with 2-byte messages and a 32 B buffer (was 12 B / 64 B)");

    /* every command type at its limits, one at a time */
    HOST_CHECK("S2-U29", motion_follow(0) && rx_is(APP_CMD_FOLLOW, 0u, 1u)
               && motion_follow(MOTION_FOLLOW_MAX_MM_S)
               && rx_is(APP_CMD_FOLLOW, 300u, 2u),
               "FOLLOW 0 and 300 mm/s round trip");
    HOST_CHECK("S2-U29", motion_move_forward(1) && rx_is(APP_CMD_FWD, 1u, 3u)
               && motion_move_forward(MOTION_MOVE_MAX_MM)
               && rx_is(APP_CMD_FWD, 2047u, 0u),
               "FWD 1 and 2047 mm round trip; tag wraps 3 -> 0");
    HOST_CHECK("S2-U29", motion_move_backward(1) && rx_is(APP_CMD_BACK, 1u, 1u)
               && motion_move_backward(MOTION_REVERSE_MAX_MM)
               && rx_is(APP_CMD_BACK, 150u, 2u),
               "BACK 1 and 150 mm round trip");
    HOST_CHECK("S2-U29", motion_turn_left(1) && rx_is(APP_CMD_LEFT, 1u, 3u)
               && motion_turn_left(MOTION_TURN_MAX_DEG)
               && rx_is(APP_CMD_LEFT, 360u, 0u)
               && motion_turn_right(1) && rx_is(APP_CMD_RIGHT, 1u, 1u)
               && motion_turn_right(MOTION_TURN_MAX_DEG)
               && rx_is(APP_CMD_RIGHT, 360u, 2u),
               "LEFT and RIGHT 1 and 360 deg round trip");
    HOST_CHECK("S2-U29", motion_stop() && rx_is(APP_CMD_STOP, 0u, 3u)
               && motion_reset_odometry() && rx_is(APP_CMD_ZERO, 0u, 0u)
               && (0u == motion_last_tag()),
               "STOP and ZERO round trip; last tag reported");

    /* out-of-range arguments are still refused, nothing queued */
    HOST_CHECK("S2-U29", !motion_move_forward(0) && !motion_move_forward(2048)
               && !motion_move_backward(151) && !motion_turn_left(361)
               && !motion_follow(301) && !motion_follow(-1)
               && (0u == g_q_n) && (0u == motion_last_tag()),
               "out-of-range commands refused, tag unchanged");

    /* queue depth unchanged: 4 commands fit, the 5th is refused */
    HOST_CHECK("S2-U29", motion_follow(120) && motion_turn_left(90)
               && motion_move_forward(800) && motion_stop()
               && !motion_move_backward(100) && (0u == motion_last_tag()),
               "four commands fill K5, the fifth is refused (depth 4 as before)");
    HOST_CHECK("S2-U29", rx_is(APP_CMD_FOLLOW, 120u, 1u) && rx_is(APP_CMD_LEFT, 90u, 2u)
               && rx_is(APP_CMD_FWD, 800u, 3u) && rx_is(APP_CMD_STOP, 0u, 0u)
               && !rx_is(APP_CMD_BACK, 100u, 1u),
               "FIFO order kept; the refused command never arrives");

    HOST_CHECK("S2-U29", 2 == g_max_snd,
               "every K5 message sent was one 2-byte MSG_CMD word (was 12 B)");

    return (0 == HOST_SUMMARY("motion_cmd")) ? 0 : 1;
}
