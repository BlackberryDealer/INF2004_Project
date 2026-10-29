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
#define MISSION_CAL_SWEEP_MAX_MS    (3000u)
#define MISSION_START_WARN_MS       (1000u)
#define MISSION_SEARCH_TIMEOUT_MS   (5000u)   /* FR14 */
#define MISSION_SEARCH_SWEEP_DEG    (60)
/* MEASURE: 1.5 x the largest barcode-to-junction spacing */
#define MISSION_ARM_EXPIRE_MM       (1000u)
#define MISSION_JCT_ADVANCE_MM      (60u)     /* MEASURE: sensors to axle */
#define MISSION_FR9_LIMIT_MS        (5000u)
#define MISSION_REJOIN_DEG          (45)
#define MISSION_REJOIN_WIDTH_MM     (25u)     /* 18 mm line at 45 deg */
#define MISSION_REJOIN_WIDTH_TOL_MM (8u)
#define MISSION_REJOIN_WIN_PCT      (30u)
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
#if (MISSION_RECOVER_MS < (MISSION_RUN_START_HOLD_MS * 3u))
#error "Recovery must cover several cycles of every 10 ms task"
#endif

#endif /* CFG_MISSION_H */

/*** end of file ***/
