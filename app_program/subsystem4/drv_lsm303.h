/**
 * @file    drv_lsm303.h
 * @brief   LSM303D driver: register-level I2C1, accelerometer FIFO,
 *          magnetometer (INT1 unused since Change Requirement 2).
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
/* The other LSM303D address: SA0 low selects 0x1E, SA0 high 0x1D (ST
   datasheet DocID023312, I2C address). On-car 2026-11-04 the module's SA0
   (no wire on the header) read low and the chip answered only at 0x1E. */
#define DRV_LSM303_ADDR_ALT (0x1Eu)

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

/* Pure address helpers (host-tested by S4-U15). */
static inline uint32_t drv_lsm303_other_addr(uint32_t addr)
{
    return (DRV_LSM303_ADDR == addr) ? DRV_LSM303_ADDR_ALT : DRV_LSM303_ADDR;
}

/* id_now / id_other: drv_lsm303_identity_ok() at the current and the other
   address (1 = WHO_AM_I 0x49, 0 = wrong identity, <0 = I2C error). The
   other address is taken only when it identifies and the current one
   does not; otherwise the current one is kept (fault path as before).    */
static inline uint32_t drv_lsm303_pick_addr(uint32_t addr_now, int32_t id_now,
                                            int32_t id_other)
{
    uint32_t pick = addr_now;

    if ((1 != id_now) && (1 == id_other))
    {
        pick = drv_lsm303_other_addr(addr_now);
    }

    return pick;
}

extern int32_t drv_lsm303_init(void); /* 0 OK, <0 I2C error */
extern uint32_t drv_lsm303_addr(void); /* 7-bit address in use */
extern int32_t drv_lsm303_read_acc(drv_lsm303_acc_t * p_samples,
                                   uint32_t max_samples); /* count or <0 */
extern int32_t drv_lsm303_read_mag(int16_t * p_mx, int16_t * p_my,
                                   int16_t * p_mz); /* 0 or <0 */
extern int32_t  drv_lsm303_identity_ok(void);   /* 1 = WHO_AM_I 0x49 */
extern uint32_t drv_lsm303_fifo_overruns(void); /* saturating, for the
                                                   packed PRV_OFS_B row */
extern void drv_lsm303_bus_reset(void);
extern int32_t drv_lsm303_last_error(void); /* -1 timeout, -2 NACK */

#endif /* DRV_LSM303_H */

/*** end of file ***/
