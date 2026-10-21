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
#define MISSION_CAL_SWEEP_DEG       (30)      /* left 30, right 60, left 30 */
#define MISSION_CAL_SWEEP_MAX_MS    (3000u)
#define MISSION_START_WARN_MS       (1000u)
#define MISSION_SEARCH_TIMEOUT_MS   (5000u)   /* FR14 */
#define MISSION_SEARCH_SWEEP_DEG    (60)
#define MISSION_ARM_EXPIRE_MM       (1000u)   /* MEASURE: 1.5 x largest
                                                 barcode-to-junction gap */
#define MISSION_JCT_ADVANCE_MM      (60u)     /* MEASURE: sensors to axle */
#define MISSION_FR9_LIMIT_MS        (5000u)
#define MISSION_REJOIN_DEG          (45)
#define MISSION_REJOIN_WIDTH_MM     (25u)     /* 18 mm line at 45 deg */
#define MISSION_REJOIN_WIDTH_TOL_MM (8u)
#define MISSION_REJOIN_WIN_PCT      (30u)
#define MISSION_HEADING_TOL_CDEG    (500)     /* 5 deg */
#define MISSION_INHIBIT_CLEAR_MM    (150u)
#define MISSION_RESUME_PULSE_MS     (150u)    /* > 2 obstacle periods */
#define MISSION_BTN_DEBOUNCE_TICKS  (3u)      /* 30 ms */
#define MISSION_BTN_RESET_HOLD_TICKS (200u)   /* 2 s */

#endif /* CFG_MISSION_H */

/*** end of file ***/
