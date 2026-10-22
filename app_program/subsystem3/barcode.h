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
extern void barcode_map_set(uint32_t char_index, uint32_t cmd); /* mission */
extern uint32_t barcode_branch_seq(void); /* +1 per long single bar seen */

#endif /* BARCODE_H */

/*** end of file ***/
