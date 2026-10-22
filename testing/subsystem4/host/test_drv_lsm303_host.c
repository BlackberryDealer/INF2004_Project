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
        uint32_t ovr = 0u;

        ovr = drv_lsm303_fifo_plan(0x00u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (0u == ovr),
                   "empty FIFO: take 0, no overrun");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr = 0u;

        ovr = drv_lsm303_fifo_plan(0x02u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (2u == take) && (0u == ovr),
                   "two unread samples: take 2");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr = 0u;

        ovr = drv_lsm303_fifo_plan(0x06u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (4u == take) && (0u == ovr),
                   "backlog of 6: take 4 now");

        ovr = drv_lsm303_fifo_plan(0x02u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (2u == take) && (0u == ovr),
                   "backlog of 2 next cycle: take 2");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr = 7u;

        ovr = drv_lsm303_fifo_plan(0x45u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (8u == ovr),
                   "overrun: no samples used, counter +1");
    }
    {
        uint32_t take = 99u;
        uint32_t ovr = 255u;

        ovr = drv_lsm303_fifo_plan(0x40u, 4u, &take, ovr);
        HOST_CHECK("S4-U14", (0u == take) && (255u == ovr),
                   "overrun counter saturates at 255");
    }

    return (0 == HOST_SUMMARY("drv_lsm303")) ? 0 : 1;
}
