/**
 * @file    app.h
 * @brief   Kernel-facing helpers shared by every task module.
 * @owner   Team (integrator; change only by SHARED pull request)
 * @kind    task
 */

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>        /* micro T-Kernel API */

#ifndef APP_PROBE
#define APP_PROBE   (0)
#endif
#ifndef APP_TEST
#define APP_TEST    (0)
#endif

/* Task priorities (1 = highest). Kernel: USB console 3, radio 4. */
#define APP_PRI_LINE       (12)
#define APP_PRI_MOTION     (13)
#define APP_PRI_IMU        (14)
#define APP_PRI_OBSTACLE   (15)
#define APP_PRI_MISSION    (16)
#define APP_PRI_BARCODE    (17)
#define APP_PRI_TELEM      (20)
#define APP_PRI_TEST       (21)   /* usermain while running TEST=1 suites */

/* Stack sizes in bytes, including the kernel's 128-byte system part
   (with TA_USERBUF the kernel adds nothing). Multiples of 8.        */
#define APP_SYS_STACK_BYTES       (128u)
#define APP_STACK_LINE_BYTES      (1024u + APP_SYS_STACK_BYTES)
#define APP_STACK_MOTION_BYTES    (1024u + APP_SYS_STACK_BYTES)
#define APP_STACK_IMU_BYTES       (1024u + APP_SYS_STACK_BYTES)
#define APP_STACK_OBSTACLE_BYTES  (1024u + APP_SYS_STACK_BYTES)
#define APP_STACK_MISSION_BYTES   (2048u + APP_SYS_STACK_BYTES)
#define APP_STACK_BARCODE_BYTES   (1024u + APP_SYS_STACK_BYTES)
#define APP_STACK_TELEM_BYTES     (2048u + APP_SYS_STACK_BYTES)

/* Create and start a task on a caller-owned static stack (TA_USERBUF).
   Fills the stack with a pattern first so free space can be measured. */
extern ID   app_start_task(FP p_entry, PRI priority, uint64_t * p_stack,
                           uint32_t stack_bytes, char const * p_name);

/* Print "ERROR <what>: <code>" if ercd < E_OK. Any task. */
extern void app_check(ER ercd, char const * p_what);

/* Console output (never from an ISR, a handler or the radio task in RUN).
   p_format may use up to three %d; unused arguments are ignored.      */
extern void app_log(char const * p_format, int32_t arg_a, int32_t arg_b,
                    int32_t arg_c);

/* GP14 timing probe; does nothing unless built with PROBE=1. */
extern void app_probe_set(bool b_is_high);

/* Stack measurement (NFR11c). */
extern void     app_stack_report(void);          /* prints every task */
extern uint32_t app_stack_free_pct_min(void);    /* worst task, 0-100 */

#endif /* APP_H */

/*** end of file ***/
