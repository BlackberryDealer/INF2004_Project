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

extern uint32_t telem_drops(void);  /* K7 + K11 + publish drops, 0..255 */
extern ID       telem_k11_id(void);  /* K11 ID, for mqtt_bridge.c only */

#endif /* TELEM_H */

/*** end of file ***/
