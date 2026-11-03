/**
 * @file    cfg_motion.h
 * @brief   Every tunable and measured constant of the motion subsystem.
 *          TUNE = set by tuning on the robot, MEASURE = from a bench test
 *          logged in testing/subsystem2/TEST_LOG.md.
 * @owner   Buddy 2 (motion)
 * @kind    cfg
 *
 * Other buddies read two names from here: MOTION_BRAKE_DECEL_MM_S2
 * (Buddy 5, obstacle stop budget) and APP_SPEED_PHYS_MAX_MM_S via
 * app_types.h (Buddy 3). Keep those names.
 */

#ifndef CFG_MOTION_H
#define CFG_MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "../app_types.h"

/* ---- Motor driver (PWM slices 4 and 5, 10 kHz) ----------------------- */
#define MOTION_PWM_WRAP               (12499u) /* 125 MHz / 10 kHz - 1 */
#define MOTION_DUTY_MAX_PERMILLE      (1000)
/* A5, measured on the car 2026-10-29: the Robo Pico M1A test button turns
   the left wheel forward, M2A turns the right wheel BACKWARD. Hand-turning
   each wheel forward: the right count rises, the left count FALLS.       */
#define MOTION_L_REVERSED             (false)  /* measured: A5 wiring   */
#define MOTION_R_REVERSED             (true)   /* measured: A5 wiring   */
#define MOTION_ENC_L_REVERSED         (true)   /* measured: count sign  */
#define MOTION_ENC_R_REVERSED         (false)  /* measured: count sign  */

/* ---- Speeds and command limits (NFR15, NFR16) ------------------------ */
/* Cruise 250 -> 180 mm/s (team decision 2026-11-03): at 250 the car ran
   off the line on curves. NFR15 (cruise >= 250) must be updated to match
   in the requirements and traceability.                               */
#define MOTION_CRUISE_MM_S            (180)    /* NFR15 (revised), TUNE */
#define MOTION_SLOW_MM_S              (150)    /* NFR15 turns/search  */
#define MOTION_FOLLOW_MAX_MM_S        (300)
#define MOTION_MOVE_SPEED_MM_S        (150)    /* NFR16 bypass legs   */
#define MOTION_TURN_SPEED_MM_S        (150)
#define MOTION_REVERSE_SPEED_MM_S     (100)    /* NFR16               */
#define MOTION_REVERSE_MAX_MM         (150)    /* NFR16               */
#define MOTION_MOVE_MAX_MM            (2047)
#define MOTION_TURN_MAX_DEG           (360)
/* Creep (on-car logs 2026-11-03): braking from 150 mm/s carried each
   spin turn about 9 mm of wheel travel past its target, and small turns
   about twice as far as asked. The last MOTION_CREEP_ZONE_MM of every
   MOVE and TURN (wheel travel) now run at MOTION_CREEP_MM_S, so the
   brake starts slow. If both wheels stand still for
   MOTION_CREEP_STUCK_TICKS inside the zone after having moved (creep
   duty too low to keep turning), the command ends there instead of
   hanging until mission's deadline.                                   */
#define MOTION_CREEP_MM_S             (80)     /* TUNE */
#define MOTION_CREEP_ZONE_MM          (20u)    /* TUNE */
#define MOTION_CREEP_STUCK_TICKS      (15u)    /* 150 ms */

/* ---- Control gains (Q8 = value x 256) -------------------------------- */
/* 5 -> 3 (on-car 2026-11-03): at 5 a full error turned the car at about
   115 deg/s and it swayed on straights; the damping is in line_track.c. */
#define MOTION_KP_STEER_Q8            (3)    /* TUNE: cpw/steer unit x256 */
#define MOTION_K_HEADING_Q8           (64)   /* TUNE: cpw/count drift x256 */
#define MOTION_KFF_Q8                 (3556) /* TUNE: 1000 / 72 cpw x256 */
#define MOTION_KP_Q8                  (2560)   /* TUNE */
/* TUNE: KI 256 (spec start value) winds the integrator up while the duty
   is ramp-limited: about 30 % overshoot and 8 % error at 0.5 s on the
   S2-U02 model. 16 settles within 2.3 % (gain sweep in testing/
   subsystem2/TEST_LOG.md). Retune on the bench in S2-I01.              */
#define MOTION_KI_Q8                  (16)
#define MOTION_I_CLAMP                (76800)  /* +-300 permille x256 (E1) */
#define MOTION_RAMP_PERMILLE_PER_TICK (50)     /* TUNE: 0 to full in 200 ms */

/* ---- Stall, encoder plausibility, braking, K5 ------------------------ */
#define MOTION_STALL_DUTY_MIN_PERMILLE (300)   /* TUNE */
#define MOTION_STALL_TICKS            (30u)    /* 300 ms */
#define MOTION_ENC_FAULT_STREAK       (2u)     /* impossible reads in a row */
#define MOTION_BRAKE_DECEL_MM_S2      (1500)   /* MEASURE: A20 */
#define MOTION_US_TO_MS_Q20           (1049u)  /* (us * 1049) >> 20 = ms */
#define MOTION_K5_ENTRIES             (4u)

/* ---- Constant-versus-field checks (Part C.4b) ------------------------ */

#if (MOTION_STALL_TICKS > 31u)   /* PRV_MOT_A STALL_L/R are 5 bits */
#error "MOTION_STALL_TICKS no longer fits the stall counter: widen it"
#endif
#if (MOTION_ENC_FAULT_STREAK > 3u)   /* PRV_MOT_A ENC_STRK_L/R: 2 bits */
#error "MOTION_ENC_FAULT_STREAK no longer fits the streak counter"
#endif
#if ((MOTION_MOVE_MAX_MM > 2047) || (MOTION_TURN_MAX_DEG > 2047) \
     || (MOTION_FOLLOW_MAX_MM_S > 2047))   /* MSG_CMD PARAM: 11 bits */
#error "A command limit no longer fits the 11-bit K5 parameter"
#endif
#if ((2u * (((MOTION_MOVE_MAX_MM * 1000u) + (APP_UM_PER_COUNT / 2u)) \
            / APP_UM_PER_COUNT)) > 16383u)   /* PRV_MOT_C TARGET2: 14 b */
#error "2 x the longest move no longer fits TARGET2: widen it"
#endif
#if ((MOTION_DUTY_MAX_PERMILLE * 25) / 2 > 65535)  /* PWM level uint16 */
#error "Full duty no longer fits the 16-bit PWM level"
#endif
#if (MOTION_REVERSE_MAX_MM > MOTION_MOVE_MAX_MM)
#error "Reverse limit must not exceed the forward limit"
#endif

#endif /* CFG_MOTION_H */

/*** end of file ***/
