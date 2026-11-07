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

/* ---- debug telemetry (2026-11-06: track sensors without the USB cable) */
/* 1: every app_log() console line is also published as text on
   TELEM_TOPIC_LOG, and a SENSORS payload follows every STATE. Bring-up
   diagnostic like the console prints themselves: set 0 for the Week 13
   message-size evidence (only the binary payloads are then sent).     */
#define TELEM_DEBUG_MQTT        (1u)
#define TELEM_LOG_TEXT_MAX      (120u)     /* text bytes per LOG payload */

/* ---- queue sizes -------------------------------------------------------- */
#define TELEM_K7_ENTRIES        (8u)
/* 8 entries for STATE, heartbeat and events; 12 with SENSORS */
#define TELEM_K11_ENTRIES       ((TELEM_DEBUG_MQTT != 0u) ? 12u : 8u)
/* K12: console lines mirrored to MQTT (format pointer + 3 arguments,
   16 B each); formatted later by the radio task, so app_log() costs its
   caller one non-blocking tk_snd_mbf(). Lines are dropped while full.  */
#define TELEM_K12_ENTRIES       (48u)

/* ---- remote commands ---------------------------------------------------- */
#define TELEM_CMD_MIN_GAP_MS    (100u)     /* one command per type per 100 ms */

/* ---- topics (B1.4.1: short topics; baseline was picocar/csc28-car/...) */
#define TELEM_TOPIC_STATE   "c28/s"
#define TELEM_TOPIC_EVENT   "c28/e"
#define TELEM_TOPIC_HB      "c28/h"
#define TELEM_TOPIC_AVAIL   "c28/a"
#define TELEM_TOPIC_CMD     "c28/c"
#define TELEM_TOPIC_SENS    "c28/d"   /* SENSORS (debug), QoS 0         */
#define TELEM_TOPIC_LOG     "c28/l"   /* LOG: console lines (debug)     */

#endif /* CFG_TELEM_H */

/*** end of file ***/
