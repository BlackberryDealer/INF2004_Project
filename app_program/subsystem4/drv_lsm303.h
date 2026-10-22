/**
 * @file    drv_lsm303.h
 * @brief   LSM303D driver: register-level I2C1, accelerometer FIFO,
 *          magnetometer and the impact-interrupt configuration.
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    drv
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include <stdbool.h>

#define DRV_LSM303_FIFO_MAX (4u) /* samples read per call, at most */

/* LSM303D hardware identity. Spec B4.1.1 was written for the LSM303DLHC
   (accelerometer 0x19, magnetometer 0x1E, identity 'H','4','3'); the
   GY-511 board actually held carries an LSM303D, which answers on one
   address with WHO_AM_I = 0x49. See SPEC-QUESTION SQ-B4-2 in
   drv_lsm303.c. Kept in the header so the host test can lock them. */
#define DRV_LSM303_ADDR     (0x1Du)
#define DRV_LSM303_WHO_AM_I (0x49u)

typedef struct
{
    int16_t acc_x; /* raw register image, E4 */
    int16_t acc_y; /* raw register image, E4 */
    int16_t acc_z; /* raw register image, E4 */
} drv_lsm303_acc_t;

/* Pure FIFO-source planner (host-tested by S4-U14). From one
   FIFO_SRC_REG_A image it decides how many samples to take now and
   counts an overrun. No hardware is touched here, so the host test
   compiles only this header.                                              */
static inline uint32_t drv_lsm303_fifo_plan(uint32_t   fifo_src,
                                            uint32_t   max_samples,
                                            uint32_t * p_take,
                                            uint32_t   overrun_count)
{
    uint32_t take = 0u;
    uint32_t ovr  = overrun_count;

    if (0u != (fifo_src & 0x40u)) /* OVRN: discard, use nothing */
    {
        if (ovr < 255u)
        {
            ovr++;
        }
    }
    else
    {
        take = fifo_src & 0x1Fu; /* FSS0..4: unread count 0..31 */
        if (take > max_samples)
        {
            take = max_samples; /* backlog above the cap waits */
        }
    }

    *p_take = take;
    return ovr;
}

extern int32_t drv_lsm303_init(void); /* 0 OK, <0 I2C error */
extern int32_t drv_lsm303_read_acc(drv_lsm303_acc_t * p_samples,
                                   uint32_t max_samples); /* count or <0 */
extern int32_t drv_lsm303_read_mag(int16_t * p_mx, int16_t * p_my,
                                   int16_t * p_mz); /* 0 or <0 */
/* Reads INT1_SRC_A, which also clears the latched interrupt so a later
   impact can raise a new edge (B4.1.1). Returns the byte or <0. */
extern int32_t  drv_lsm303_read_int1_src(void);
extern int32_t  drv_lsm303_identity_ok(void);   /* 1 = WHO_AM_I 0x49 */
extern uint32_t drv_lsm303_fifo_overruns(void); /* saturating, for the
                                                   packed PRV_OFS_B row */
extern void drv_lsm303_bus_reset(void);

#endif /* DRV_LSM303_H */

/*** end of file ***/
