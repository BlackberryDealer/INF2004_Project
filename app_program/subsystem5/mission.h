/**
 * @file    mission.h
 * @brief   Mission queue K4, mode flags K10 and the mission task.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   mission_init(void);    /* creates K4 and K10 */
extern ER   mission_start(void);   /* starts mission_task (call last) */

/* Any task or the radio task. Clamps value to the type's own width
   (layout.csv), sends one 16-bit word with TMO_POL. Returns false and
   sets APP_FAULT_QUEUE_OVERFLOW if K4 is full.                       */
extern bool     mission_post_event(uint32_t type, uint32_t value);

/* Builds the BARCODE / MAP event value: char index bits 0-5, cmd 6-7. */
extern uint32_t mission_evt_barcode_value(uint32_t cmd, uint32_t char_index);

extern void     mission_mode_set(uint32_t bits);     /* mission_task only */
extern void     mission_mode_clear(uint32_t bits);   /* mission_task only */
extern bool     mission_mode_is_set(uint32_t bit);   /* any task */
extern uint32_t mission_queue_high_water(void);

#endif /* MISSION_H */

/*** end of file ***/
