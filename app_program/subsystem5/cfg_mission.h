/**
 * @file    cfg_mission.h
 * @brief   Every mission tunable (timers, sweeps, rejoin window, buttons).
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    cfg
 */

#ifndef CFG_MISSION_H
#define CFG_MISSION_H

#include <stdint.h>
#include "../app_types.h"
#include "../subsystem2/cfg_motion.h"

#define MISSION_CAL_TIMEOUT_MS      (10000u)
#define MISSION_WDT_LOG_MS          (2000u)   /* 0 = no feed-gap log    */
#define MISSION_CAL_SWEEP_DEG       (30)      /* left 30, right 60, left 30 */
#define MISSION_CAL_SWEEP_MAX_MS    (4000u)   /* sweep + heading return */
/* After the sweep the car turns back to its start heading (encoders):
   each spin turn overshoots in its own direction, and the three legs
   left the car about 8 deg left of the line (on-car log 2026-11-03,
   ENC L -18 R 37).                                                    */
#define MISSION_CAL_HEADING_TOL_CDEG (200)    /* TUNE: 2 deg */
#define MISSION_CAL_CORR_MAX        (2u)      /* return turns at most */
#define MISSION_CDEG_PER_DEG        (100)
#define MISSION_START_WARN_MS       (1000u)
#define MISSION_SEARCH_TIMEOUT_MS   (5000u)   /* FR14 */
#define MISSION_SEARCH_SWEEP_DEG    (60)
/* The first reported barcode edge can be inside the strip. Back up
   past it and require continuous white travel before reading again. */
#define MISSION_BARCODE_LEAD_MM      (40u)  /* behind first reported edge */
#define MISSION_BARCODE_QUIET_MM     (35u)
/* A failed capture asks for a rewind only after its quiet zone, so the
   whole strip is behind the sensor (on-car 2026-11-10: 286 mm). The
   rewind is sent as moves of at most MOTION_REVERSE_MAX_MM.          */
#define MISSION_BARCODE_REWIND_MM    (450u)
#define MISSION_BARCODE_BACK_EXTRA_MM (70u) /* past the needed distance */
#define MISSION_BARCODE_BACK_PAD_MM  (10u)  /* leave room to brake */
#define MISSION_BARCODE_WAIT_MS      (250u) /* let the line settle */
#define MISSION_BARCODE_BRAKE_MS     (500u)
#define MISSION_BARCODE_SPEED_MM_S   (70)   /* above decoder's 50 mm/s */
#define MISSION_BARCODE_READ_MM      (520u) /* rewind + code + quiet */
#define MISSION_BARCODE_READ_MS      (9000u)
#define MISSION_BARCODE_REARM_MM     (60u)
#define MISSION_JCT_ADVANCE_MM      (60u)     /* MEASURE: sensors to axle */
/* FR9 bypass limit (end of the pass leg -> line following again)
   5 s -> 20 s (Dylan, 2026-11-04): more time to find the line again,
   and the car is slower now. FR9 must be updated to match in the
   requirements and traceability.                                     */
#define MISSION_FR9_LIMIT_MS        (20000u)
#define MISSION_REJOIN_DEG          (45)
/* Width of the rejoin crossing. line.c measures a black run while EITHER
   sensor is black, so at 45 deg it is 18 / sin 45 = 25 mm of line plus
   the sensor spacing x cot 45 (the second sensor reaches the line later):
   49 mm on the car (2026-11-04 14:16; 25 +- 8 rejected it, S5-U41).
   45 +- 30 accepts 15..75 mm: crossings from ~30 to ~60 deg; a single
   barcode bar (< 15 mm at one sensor) or driving along the line
   (> 75 mm) is still rejected.                                        */
#define MISSION_REJOIN_WIDTH_MM     (45u)     /* MEASURE: sensor spacing */
#define MISSION_REJOIN_WIDTH_TOL_MM (30u)
/* Rejoin distance window around e = lateral leg x sqrt 2. In the
   9 November runs an early 116 mm crossing (e = 390) and a late
   511 mm crossing (e = 395) led away from the original line. Keep
   the width bounds and accept crossings between 50 % and 120 % of e.
   The 221 mm crossing at e = 407 (S5-U43) is still in range.          */
#define MISSION_REJOIN_WIN_PCT      (20u)     /* far end: e + 20 %     */
#define MISSION_REJOIN_LO_PCT       (50u)     /* near end: 50 % of e   */
#define MISSION_HEADING_TOL_CDEG    (500)     /* 5 deg */
#define MISSION_INHIBIT_CLEAR_MM    (150u)
#define MISSION_RESUME_PULSE_MS     (150u)    /* > 2 obstacle periods */
#define MISSION_RUN_START_HOLD_MS   (30u)     /* >= 3 cycles of 10 ms tasks */
#define MISSION_RECOVER_MS          (100u)    /* fault re-assert window */
#define MISSION_ACK_TIMEOUT_MS      (200u)    /* run-start ZERO ack */
#define MISSION_MOVE_MARGIN_MS      (1000u)   /* ramp + service margin */
/* um of wheel arc per degree of spin turn: pi x track / 360 */
#define MISSION_TURN_UM_PER_DEG     (((APP_TRACK_UM * 355u) / 113u) / 360u)
#define MISSION_BTN_DEBOUNCE_TICKS  (3u)      /* 30 ms */
#define MISSION_BTN_RESET_HOLD_TICKS (200u)   /* 2 s */

/* C.4b: constants that set a stored field's domain (PRV_MIS_G BTN_RUN
   2 bits, BTN_RESET 8 bits; PRV_MIS_A NO_PATH_CNT counts to 2).      */
#if (MISSION_BTN_DEBOUNCE_TICKS > 3u)
#error "Run-button debounce no longer fits the 2-bit BTN_RUN counter"
#endif
#if (MISSION_BTN_RESET_HOLD_TICKS > 255u)
#error "Reset hold no longer fits the 8-bit BTN_RESET counter"
#endif
#if ((MISSION_BARCODE_QUIET_MM > MISSION_BARCODE_LEAD_MM) ||                  \
     (MISSION_BARCODE_REWIND_MM <= MISSION_BARCODE_LEAD_MM) ||               \
     (MISSION_BARCODE_READ_MM < MISSION_BARCODE_REWIND_MM))
#error "Barcode rewind must leave a lead-in and fit the reverse command"
#endif
#if (MISSION_BARCODE_SPEED_MM_S >= MOTION_CRUISE_MM_S)
#error "Barcode reread must be slower than cruise"
#endif
#if (MISSION_RECOVER_MS < (MISSION_RUN_START_HOLD_MS * 3u))
#error "Recovery must cover several cycles of every 10 ms task"
#endif

#endif /* CFG_MISSION_H */

/*** end of file ***/
