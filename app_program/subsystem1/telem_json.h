/**
 * @file    telem_json.h
 * @brief   Baseline JSON payload builders (natural types, integers only).
 *          Pure, host-safe. Exists until w10-baseline, then replaced by
 *          telem_bin.
 * @owner   Buddy 1 (communications)
 * @kind    algo
 */

#ifndef TELEM_JSON_H
#define TELEM_JSON_H

#include <stdint.h>
#include <stdbool.h>
#include "telem_bin.h"

/* Every builder writes its JSON text (no NUL) into p_out and returns the
   length, or 0 if the text did not fit in size bytes.                  */

/* {"st":..,"run":..,"mot":..,"spd":[l,r],"enc":[l,r],"line":..,
   "steer":..,"odo":..,"pitch":..,"hump_pk":..,"turn":..,"cls":..,
   "flt":..}  speeds in mm/s, odo in mm (clamped to 32767).             */
extern uint32_t telem_json_state(telem_bin_state_t const * p_state,
                                 char * p_out, uint32_t size);

/* {"ev":<type>,"v":<value>} */
extern uint32_t telem_json_event(uint32_t type, uint32_t value,
                                 char * p_out, uint32_t size);

/* {"ev":10,"ang":<deg>,"w":..,"near":..,"cl_l":..,"cl_r":..,"side":..} */
extern uint32_t telem_json_profile(telem_bin_profile_t const * p_prof,
                                   char * p_out, uint32_t size);

/* {"ev":22,"reason":..,"hump_pk":..,"odo":..,"flt":..,"t":..} */
extern uint32_t telem_json_summary(telem_bin_summary_t const * p_summ,
                                   char * p_out, uint32_t size);

/* {"up":<s>,"drops":<n>,"lat":<us>} */
extern uint32_t telem_json_heartbeat(uint32_t uptime_s, uint32_t drops,
                                     uint32_t lat_max_us, char * p_out,
                                     uint32_t size);

#endif /* TELEM_JSON_H */

/*** end of file ***/
