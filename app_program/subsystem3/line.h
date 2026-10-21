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

#endif /* LINE_H */

/*** end of file ***/
