/**
 * @file    telem.h
 * @brief   Telemetry task, K7 and K11 message buffers.
 * @owner   Buddy 1 (communications)
 * @kind    task
 */

#ifndef TELEM_H
#define TELEM_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "telem_bin.h"

extern ER   telem_init(void);   /* creates K7, K11 */
extern ER   telem_start(void);  /* starts telem_task */

/* mission_task only. All TMO_POL; false if K7 is full (counted). */
extern bool telem_post_event(uint32_t type, uint32_t value);
extern bool telem_post_profile(telem_bin_profile_t const * p_profile);
extern bool telem_post_summary(telem_bin_summary_t const * p_summary);
extern bool telem_post_snapshot(telem_bin_snapshot_t const * p_snap);

extern uint32_t telem_drops(void);  /* K7 + K11 + publish drops, 0..255 */
extern ID       telem_k11_id(void);  /* K11 ID, for mqtt_bridge.c only */

/* One mirrored console line as queued in K12: the app_log() format (a
   string literal or a constant table entry, so the pointer stays valid)
   and its three arguments. The radio task formats it (mqtt_bridge.c). */
typedef struct
{
    char const * p_format;
    int32_t      arg_a;
    int32_t      arg_b;
    int32_t      arg_c;
} telem_log_rec_t;

/* app_log() only: queue one console line for MQTT without waiting.
   Does nothing unless the image has MQTT and TELEM_DEBUG_MQTT is 1;
   a full K12 drops the line (counted).                               */
extern void     telem_log_post(char const * p_format, int32_t arg_a,
                               int32_t arg_b, int32_t arg_c);
extern ID       telem_k12_id(void);  /* K12 ID (0 = none), mqtt_bridge.c */
extern uint32_t telem_log_drops(void); /* K12 full drops, 0..255         */

#endif /* TELEM_H */

/*** end of file ***/
