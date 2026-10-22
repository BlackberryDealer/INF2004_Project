/**
 * @file    code39.h
 * @brief   Pure Code 39 barcode decoder (forward and reverse, both scales).
 * @owner   Buddy 3 (line, barcode)
 * @kind    algo
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

#define CODE39_OK       (0u)
#define CODE39_INVALID  (1u)
#define CODE39_STAR     (43u)   /* index of '*' in the pattern table */

extern uint32_t code39_decode(uint8_t const * p_widths, uint32_t count,
                              uint32_t * p_char_index);
extern uint32_t code39_char_ascii(uint32_t char_index);  /* for logs/tests */

#endif /* CODE39_H */

/*** end of file ***/
