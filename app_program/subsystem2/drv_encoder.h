/**
 * @file    drv_encoder.h
 * @brief   Two Hall quadrature encoders counted by PIO state machines (K9).
 *          Called only by motion_task.
 * @owner   Buddy 2 (motion)
 * @kind    drv
 *
 * Kernel-free on purpose: drv_encoder.c uses the Pico SDK, whose headers
 * cannot share a file with the kernel headers (docs/BARR_C_DEVIATIONS.md).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* Loads the quadrature program at offset 0 of pio0 or pio1 and starts one
   state machine per wheel. Never resets a PIO block.
   Returns 0 = OK, -1 = no PIO space (motion then raises APP_FAULT_PIO_LOAD
   every cycle and keeps the motors braked).                              */
extern int32_t drv_encoder_init(void);

/* Raw 32-bit count images (E3: use only modular differences). Sign already
   corrected so + = wheel driving forward. Both 0 if init failed.         */
extern void    drv_encoder_read(uint32_t * p_raw_l, uint32_t * p_raw_r);

/* SPEC-QUESTION: B2.1.4 asks drv_encoder_init() to print
   "ENC PIO%d SM %d %d offset 0" with app_log(), but app_log() is declared
   in app.h, which pulls in the kernel headers that cannot be compiled
   together with the SDK headers this driver needs. This getter lets
   motion_task print the same line instead. Returns false if not loaded. */
extern bool    drv_encoder_location(uint32_t * p_pio_index,
                                    uint32_t * p_sm_l, uint32_t * p_sm_r);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
