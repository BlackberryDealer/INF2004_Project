/**
 * @file    drv_button.h
 * @brief   RUN (GP20) and RESET (GP21) push-buttons on the Robo Pico,
 *          active-low inputs with internal pull-ups.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    drv
 */

#ifndef DRV_BUTTON_H
#define DRV_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

extern void drv_button_init(void);           /* GP20/GP21 pull-up inputs */
extern bool drv_button_run_pressed(void);    /* true while RUN held      */
extern bool drv_button_reset_pressed(void);  /* true while RESET held    */

#endif /* DRV_BUTTON_H */

/*** end of file ***/
