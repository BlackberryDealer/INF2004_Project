/*
 *  test_drv_lsm303_host.c
 *  Laptop tests of the pure FIFO-source planner in drv_lsm303.h
 *  (S4-U14): overrun handling and backlog splitting.
 *
 *  Host-only, outside BARR-C scope (docs/BARR_C_DEVIATIONS.md, D7).
 */

#include "host_test.h"
#include "drv_lsm303.h"

int main(void)
{
    HOST_INIT();

    {
        uint32_t take = 99u;
        uint32_t ovr  = 0u;

        ovr = drv_lsm303_fifo_plan(0x00u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (0u == ovr),
                   "empty FIFO: take 0, no overrun");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr  = 0u;

        ovr = drv_lsm303_fifo_plan(0x02u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (2u == take) && (0u == ovr),
                   "two unread samples: take 2");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr  = 0u;

        ovr = drv_lsm303_fifo_plan(0x06u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (4u == take) && (0u == ovr),
                   "backlog of 6: take 4 now");

        ovr = drv_lsm303_fifo_plan(0x02u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (2u == take) && (0u == ovr),
                   "backlog of 2 next cycle: take 2");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr  = 7u;

        ovr = drv_lsm303_fifo_plan(0x45u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (8u == ovr),
                   "overrun: no samples used, counter +1");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr  = 255u;

        ovr = drv_lsm303_fifo_plan(0x40u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (255u == ovr),
                   "overrun counter saturates at 255");
    }

    /* SQ-B4-2: the driver talks to an LSM303D, not the LSM303DLHC that
       spec B4.1.1 assumes. Lock the two facts measured on the board so a
       later edit cannot silently restore the DLHC addresses. */
    HOST_CHECK("S4-U14", (0x1Du == DRV_LSM303_ADDR),
               "LSM303D answers on the single address 0x1D");
    HOST_CHECK("S4-U14", (0x49u == DRV_LSM303_WHO_AM_I),
               "LSM303D WHO_AM_I is 0x49, not the DLHC 'H','4','3'");

    /* S4-U15 (on-car 2026-11-04): the LSM303D answered at 0x1E (SA0 read
       low) instead of 0x1D, so every transfer NACKed and the car faulted
       in CAL_IMU. The driver now checks WHO_AM_I and takes the other SA0
       address when only that one identifies.                            */
    HOST_CHECK("S4-U15", (0x1Eu == drv_lsm303_other_addr(0x1Du))
               && (0x1Du == drv_lsm303_other_addr(0x1Eu)),
               "the two SA0 addresses are 0x1D (high) and 0x1E (low)");
    HOST_CHECK("S4-U15", 0x1Du == drv_lsm303_pick_addr(0x1Du, 1, -2),
               "answers at the current address: keep it");
    HOST_CHECK("S4-U15", 0x1Eu == drv_lsm303_pick_addr(0x1Du, -2, 1),
               "NACK at 0x1D, identity at 0x1E: switch to 0x1E");
    HOST_CHECK("S4-U15", 0x1Du == drv_lsm303_pick_addr(0x1Eu, -2, 1),
               "SA0 back high: switch back to 0x1D");
    HOST_CHECK("S4-U15", 0x1Du == drv_lsm303_pick_addr(0x1Du, -2, -2),
               "nothing answers: keep the address (fault path as before)");
    HOST_CHECK("S4-U15", 0x1Du == drv_lsm303_pick_addr(0x1Du, -2, 0),
               "other address answers with a wrong identity: not taken");

    return (0 == HOST_SUMMARY("drv_lsm303")) ? 0 : 1;
}
