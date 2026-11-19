/**
 * @file    bar_ao.h
 * @brief   Analog barcode level detector (BAR_AO=1): turns GP28 ADC
 *          samples into black / white levels with a relative threshold.
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 */

#ifndef BAR_AO_H
#define BAR_AO_H

#include <stdint.h>

extern uint32_t bar_ao_init(void);
extern uint32_t bar_ao_step(uint32_t word, uint32_t code);
extern uint32_t bar_ao_level(uint32_t word);   /* 1 = black, 0 = white */

#endif /* BAR_AO_H */

/*** end of file ***/
