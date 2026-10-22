/**
 * @file    cfg_telem.h
 * @brief   Telemetry configuration: topics, periods, queue sizes and the
 *          remote-command rate limit.
 * @owner   Buddy 1 (communications)
 * @kind    cfg
 */

#ifndef CFG_TELEM_H
#define CFG_TELEM_H

#include <stdint.h>
#include <stdbool.h>

/* ---- periods (NFR8) ---------------------------------------------------- */
#define TELEM_STATE_PERIOD_MS   (200u)     /* NFR8a 5 Hz */
#define TELEM_HB_EVERY_N_STATE  (5u)       /* heartbeat every 5th state: 1 Hz */

/* ---- queue sizes -------------------------------------------------------- */
#define TELEM_K7_ENTRIES        (8u)
#define TELEM_K11_ENTRIES       (8u)

/* ---- remote commands ---------------------------------------------------- */
#define TELEM_CMD_MIN_GAP_MS    (100u)     /* one command per type per 100 ms */

/* ---- topics (B1.4.1: short topics; baseline was picocar/csc28-car/...) */
#define TELEM_TOPIC_STATE   "c28/s"
#define TELEM_TOPIC_EVENT   "c28/e"
#define TELEM_TOPIC_HB      "c28/h"
#define TELEM_TOPIC_AVAIL   "c28/a"
#define TELEM_TOPIC_CMD     "c28/c"

#endif /* CFG_TELEM_H */

/*** end of file ***/
