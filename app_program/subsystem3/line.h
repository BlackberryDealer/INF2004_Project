/**
 * @file    line.h
 * @brief   Line task entry points (D.15).
 * @owner   Buddy 3 (line and barcode)
 * @kind    task
 */

#ifndef LINE_H
#define LINE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   line_init(void);
extern ER   line_start(void);
/* Last raw IR ADC pair as one SH_IR_RAW word (L, R; 12 bits each).
   Written by line_task every 10 ms; one aligned load, so atomic.      */
extern uint32_t line_raw_word(void);

#endif /* LINE_H */

/*** end of file ***/
