/**
 * @file    fault.h
 * @brief   FLG_FAULT (K12): the shared fault bitmask (APP_FAULT_*).
 * @owner   Buddy 2 (motion)
 * @kind    task
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include "../app.h"

extern ER       fault_init(void);
extern void     fault_set(uint32_t bits);    /* tasks + radio task; no ISR */
extern void     fault_clear(uint32_t bits);  /* mission RESET; bridge WIFI */
extern uint32_t fault_get(void);             /* current mask, 10 bits */

#endif /* FAULT_H */

/*** end of file ***/
