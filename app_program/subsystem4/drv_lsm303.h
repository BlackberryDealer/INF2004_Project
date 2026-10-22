/**
 * @file    drv_lsm303.h
 * @brief   LSM303DLHC driver: register-level I2C1, accelerometer FIFO,
 *          magnetometer and the impact-interrupt configuration.
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    drv
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include <stdbool.h>

#define DRV_LSM303_FIFO_MAX (4u) /* samples read per call, at most */

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

extern int32_t  drv_lsm303_init(void); /* 0 OK, <0 I2C error */
extern int32_t  drv_lsm303_read_acc(drv_lsm303_acc_t * p_samples,
                                    uint32_t max_samples); /* count or <0 */
extern int32_t  drv_lsm303_read_mag(int16_t * p_mx, int16_t * p_my,
                                    int16_t * p_mz); /* 0 or <0 */
extern int32_t  drv_lsm303_identity_ok(void);        /* 1 = 'H','4','3' found */
extern uint32_t drv_lsm303_fifo_overruns(void);      /* saturating, for the
                                                        packed PRV_OFS_B row */
extern void drv_lsm303_bus_reset(void);

#endif /* DRV_LSM303_H */

/*** end of file ***/
