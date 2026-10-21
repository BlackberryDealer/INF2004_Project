/**
 * @file    hmi.h
 * @brief   Buttons (RUN, RESET), RGB colours and buzzer patterns.
 * @owner   Buddy 5 (mission, obstacle, shared services)
 * @kind    task
 */

#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stdbool.h>

#define HMI_NONE         (0u)
#define HMI_RUN_PRESS    (1u)   /* stable pressed 3 polls, then released */
#define HMI_RESET_HOLD   (2u)   /* stable pressed 200 polls (2 s)        */

extern void     hmi_init(void);                 /* mission_task, once  */
extern uint32_t hmi_poll(void);                 /* mission, every 10 ms */
extern void     hmi_show(uint32_t sys_state, uint32_t now_us);

/* SPEC-QUESTION Q4: B5.2.7 asks for a continuous buzzer and an LED cue
   during the 1 s start warning, but hmi_show(sys_state, now_us) cannot
   tell the warning apart from READY. This one extra call carries it.  */
extern void     hmi_set_warning(bool b_is_on);  /* mission_task only    */

#endif /* HMI_H */

/*** end of file ***/
