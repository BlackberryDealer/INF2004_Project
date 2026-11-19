/**
 * @file    barcode.h
 * @brief   Barcode task entry points (D.15) plus the branch counter line.c
 *          reads (B3).
 * @owner   Buddy 3 (line and barcode)
 * @kind    task
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   barcode_init(void);
extern ER   barcode_start(void);
/* True when the mapping was stored. False for invalid input or a full map. */
extern bool barcode_map_set(uint32_t char_index, uint32_t cmd);
extern uint32_t barcode_branch_seq(void); /* +1 per long single bar seen */
extern uint32_t barcode_edge_count(void); /* accepted DO edges, 16-bit wrap */
extern uint32_t barcode_do_level(void);   /* DO now: 1 = black, 0 = white */
/* BAR_AO=1: R4 sample entry, and the last GP28 code for the console. */
extern void     barcode_ao_sample(uint32_t code, uint32_t t_us);
extern uint32_t barcode_ao_code(void);

#endif /* BARCODE_H */

/*** end of file ***/
