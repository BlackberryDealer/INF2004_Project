/**
 * @file    run_bar.h
 * @brief   Barcode rewind-and-reread recovery state machine for RUN.
 *          Internal to mission_run.c and run_bar.c.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef RUN_BAR_H
#define RUN_BAR_H

#include <stdint.h>
#include "mission_run.h"

/* Inactive phase value stored in mission_ctx_t.barcode_phase. */
#define RUN_BAR_IDLE           (0u)

/* Outcomes returned by run_bar_step() to mission_run_handle(). */
#define RUN_BAR_OUT_ACTIVE     (0u)  /* recovery still running */
#define RUN_BAR_OUT_RESUME     (1u)  /* recovery ended: follow or armed */
#define RUN_BAR_OUT_ARMED      (2u)  /* barcode decoded: arm, clear, follow */
#define RUN_BAR_OUT_CROSS      (3u)  /* cross reached while already armed */
#define RUN_BAR_OUT_SEARCH     (4u)  /* LINE_LOST: enter SEARCH_LINE */
#define RUN_BAR_OUT_FAULT      (5u)  /* brake timeout or motion refused */

/* Stores a rewind target when a Code 39 start guard is probed. */
extern void     run_bar_begin(mission_ctx_t * p_ctx, uint32_t advance_mm,
                              status_board_t const * p_snap, uint32_t now_us);

/* Starts braking for a pending rewind once line tracking is centred, or
   cancels the pending probe if distance, time or motion limits expire. */
extern void     run_bar_try_rewind(mission_ctx_t * p_ctx,
                                   status_board_t const * p_snap,
                                   uint32_t now_us);

/* Advances an active barcode recovery pass and returns RUN_BAR_OUT_*. */
extern uint32_t run_bar_step(mission_ctx_t * p_ctx,
                             mission_evt_in_t const * p_evt,
                             status_board_t const * p_snap, uint32_t now_us);

/* Releases recovery mode flags and suppresses nearby duplicate probes.
   Called by mission_run.c after arming a decoded barcode on
   RUN_BAR_OUT_ARMED so APP_MODE_BARCODE_REREAD stays set while arming. */
extern void     run_bar_clear(mission_ctx_t * p_ctx, uint32_t path);

#endif /* RUN_BAR_H */

/*** end of file ***/
