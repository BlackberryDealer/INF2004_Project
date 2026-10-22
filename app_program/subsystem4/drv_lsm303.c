/**
 * @file    drv_lsm303.c
 * @brief   Register-level LSM303DLHC driver on I2C1 (400 kHz): accelerometer
 *          FIFO stream, magnetometer, impact interrupt (INT1), bus reset.
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    drv
 * @reqs    FR4, FR5, FR13, FR18, NFR7
 *
 * The template cannot link the SDK's i2c.c (see build_make/pico_rp2040.mk,
 * CSC28 note C2), so every bus transaction is done here against the
 * hardware/structs/i2c.h register map: IC_CON, IC_TAR, IC_DATA_CMD,
 * IC_RAW_INTR_STAT and friends. Transfers poll with a 1 ms deadline from
 * sys_tick_now_us(); this busy-wait is the one listed exception to Part A
 * rule 7 and is measured on the bench (S4-I09).
 *
 * Timing at 400 kHz (22.5 us per byte): FIFO status read = 4 bytes
 * (90 us), 2 samples = 15 bytes (338 us), 4 samples = 27 bytes (608 us),
 * magnetometer = 9 bytes (203 us). Budgets (B.3): 550 us normal, 750 us
 * magnetometer cycle, 800 us backlog.
 */

#include "drv_lsm303.h" /* own header first (BARR-C 4.3.c) */
#include "../board_pins.h"
#include "cfg_imu.h"
#include "hardware/gpio.h"
#include "hardware/structs/i2c.h"

/* This file stays kernel-free (docs/BARR_C_DEVIATIONS.md build rule): a
   file that includes app.h must not include SDK hardware headers, so the
   timestamp API is declared here instead of including sys_tick.h. The
   definition links in from Buddy 5's sys_tick.c.                          */
extern uint32_t sys_tick_now_us(void);

/* ---- private constants and types ------------------------------------ */

#define LSM303_ADDR_ACC (0x19u)
#define LSM303_ADDR_MAG (0x1Eu)

/* Accelerometer registers (LSM303DLHC datasheet) */
#define LSM303_REG_CTRL1_A     (0x20u)
#define LSM303_REG_CTRL3_A     (0x22u)
#define LSM303_REG_CTRL4_A     (0x23u)
#define LSM303_REG_CTRL5_A     (0x24u)
#define LSM303_REG_OUT_X_L_A   (0x28u)
#define LSM303_REG_FIFO_CTRL_A (0x2Eu)
#define LSM303_REG_FIFO_SRC_A  (0x2Fu)
#define LSM303_REG_INT1_CFG_A  (0x30u)
#define LSM303_REG_INT1_SRC_A  (0x31u)
#define LSM303_REG_INT1_THS_A  (0x32u)
#define LSM303_REG_INT1_DUR_A  (0x33u)

/* Magnetometer registers */
#define LSM303_REG_CRA_M     (0x00u)
#define LSM303_REG_CRB_M     (0x01u)
#define LSM303_REG_MR_M      (0x02u)
#define LSM303_REG_OUT_X_H_M (0x03u)
#define LSM303_REG_IRA_M     (0x0Au)

/* Register values (B4.1.1 table) */
#define LSM303_VAL_CTRL1_A     (0x67u) /* 200 Hz, XYZ on             */
#define LSM303_VAL_CTRL3_A     (0x40u) /* AOI1 interrupt on INT1     */
#define LSM303_VAL_CTRL4_A     (0x98u) /* BDU, +-4 g, high resolution */
#define LSM303_VAL_CTRL5_A     (0x40u) /* FIFO enable                */
#define LSM303_VAL_FIFO_CTRL_A (0x80u) /* stream mode                */
#define LSM303_VAL_INT1_CFG_A  (0x0Au) /* X-high or Y-high event     */
#define LSM303_VAL_INT1_DUR_A  (0x00u)
#define LSM303_VAL_CRA_M       (0x18u) /* 75 Hz                      */
#define LSM303_VAL_CRB_M       (0x20u) /* +-1.3 gauss                */
#define LSM303_VAL_MR_M        (0x00u) /* continuous                 */

/* Identity bytes expected from IRA/IRB/IRC */
#define LSM303_ID_H (0x48u)
#define LSM303_ID_4 (0x34u)
#define LSM303_ID_3 (0x33u)

/* FIFO_SRC_REG_A bits */
#define LSM303_FIFO_SRC_FSS_MASK (0x1Fu) /* unread samples, bits 0-4   */
#define LSM303_FIFO_SRC_OVRN     (0x40u) /* overrun, bit 6             */

/* Bus timings */
#define LSM303_BUS_TIMEOUT_US (1000u)      /* per-transfer deadline      */
#define LSM303_SYS_HZ         (125000000u) /* template fixed clock  */
#define LSM303_BUS_HZ         (400000u)
#define LSM303_RESET_HALF_US  (5u) /* SCL half period, 100 kHz   */

/* Driver error codes */
#define LSM303_ERR_TIMEOUT (-1)
#define LSM303_ERR_ABORT   (-2)

#define LSM303_SAMPLES_BYTES (DRV_LSM303_FIFO_MAX * 6u)

/* ---- private data ---------------------------------------------------- */

static uint32_t g_lsm303_fifo_ovr = 0u; /* saturating, imu_task context */

/* ---- private function prototypes ------------------------------------- */

static int32_t lsm303_hw_write_config(void);
static int32_t lsm303_tx2(uint32_t slave, uint8_t reg, uint8_t val);
static int32_t lsm303_rx(uint32_t slave, uint8_t reg, uint8_t * p_out,
                         uint32_t count);
static int32_t lsm303_wait_bits(uint32_t start_us, uint32_t bits);
static int32_t lsm303_check_abort(void);
static void    lsm303_flush(void);
static void    lsm303_rx_drain(void);
static void    lsm303_delay_us(uint32_t delay_us);
static void    lsm303_hw_setup(void);

/* ---- public functions ------------------------------------------------- */

/**
 * drv_lsm303_init
 * @brief   Puts SDA/SCL on I2C1, configures 400 kHz and writes every
 *          register from the B4.1.1 table.
 * @return  0 on success, <0 on the first I2C error
 * @context imu_task (via imu_init, or usermain before tasks run)
 */
int32_t drv_lsm303_init(void)
{
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    /* No pull-ups configured here: the GY-511 module carries them (A18,
       measured on the bench as S4-I07). */
    lsm303_hw_setup();
    return lsm303_hw_write_config();
}

/**
 * drv_lsm303_read_acc
 * @brief   Reads up to max_samples accelerometer samples from the FIFO in
 *          one auto-increment burst from OUT_X_L_A. An overrun is counted
 *          (up to 4 samples read and discarded) and no sample is used.
 * @param   p_samples    output array, one entry per sample
 * @param   max_samples  0..DRV_LSM303_FIFO_MAX
 * @return  number of samples read (0 if none), or <0 on an I2C error
 * @context imu_task
 */
int32_t drv_lsm303_read_acc(drv_lsm303_acc_t * p_samples, uint32_t max_samples)
{
    uint8_t  fifo_src                    = 0u;
    uint8_t  burst[LSM303_SAMPLES_BYTES] = {0};
    uint32_t take                        = 0u;
    uint32_t idx                         = 0u;
    int32_t  ercd                        = 0;

    if (max_samples > DRV_LSM303_FIFO_MAX)
    {
        max_samples = DRV_LSM303_FIFO_MAX; /* API cap */
    }

    ercd = lsm303_rx(LSM303_ADDR_ACC, LSM303_REG_FIFO_SRC_A, &fifo_src, 1u);
    if (0 != ercd)
    {
        return ercd;
    }

    g_lsm303_fifo_ovr = drv_lsm303_fifo_plan((uint32_t)fifo_src, max_samples,
                                             &take, g_lsm303_fifo_ovr);
    /* cast: byte to word,
       zero-extended */
    if (0u != ((uint32_t)fifo_src & LSM303_FIFO_SRC_OVRN))
    {
        /* Discard up to 4 samples so the stream can continue; no sample
           from this cycle is used (review R14). */
        ercd =
            lsm303_rx(LSM303_ADDR_ACC, (uint8_t)(LSM303_REG_OUT_X_L_A | 0x80u),
                      burst, LSM303_SAMPLES_BYTES);
        if (0 != ercd)
        {
            return ercd;
        }
        return 0;
    }

    if (0u == take)
    {
        return 0; /* no new data this cycle */
    }

    ercd = lsm303_rx(LSM303_ADDR_ACC, (uint8_t)(LSM303_REG_OUT_X_L_A | 0x80u),
                     burst, take * 6u);
    if (0 != ercd)
    {
        return ercd;
    }

    for (idx = 0u; idx < take; idx++)
    {
        uint32_t base  = idx * 6u;
        uint16_t raw_x = (uint16_t)((uint16_t)burst[base] |
                                    ((uint16_t)burst[base + 1u] << 8));
        uint16_t raw_y = (uint16_t)((uint16_t)burst[base + 2u] |
                                    ((uint16_t)burst[base + 3u] << 8));
        uint16_t raw_z = (uint16_t)((uint16_t)burst[base + 4u] |
                                    ((uint16_t)burst[base + 5u] << 8));
        /* casts: 16-bit image, no bits lost */

        p_samples[idx].acc_x = (int16_t)raw_x; /* cast: 16-bit image   */
        p_samples[idx].acc_y = (int16_t)raw_y; /* cast: 16-bit image   */
        p_samples[idx].acc_z = (int16_t)raw_z; /* cast: 16-bit image   */
    }

    return (int32_t)take; /* cast: <= 4 */
}

/**
 * drv_lsm303_read_mag
 * @brief   One magnetometer read (6 bytes from OUT_X_H_M in the datasheet
 *          order X_H, X_L, Z_H, Z_L, Y_H, Y_L).
 * @param   p_mx  out: X field
 * @param   p_my  out: Y field
 * @param   p_mz  out: Z field
 * @return  0 on success, <0 on an I2C error
 * @context imu_task
 */
int32_t drv_lsm303_read_mag(int16_t * p_mx, int16_t * p_my, int16_t * p_mz)
{
    uint8_t buf[6u] = {0};
    int32_t ercd    = 0;

    ercd = lsm303_rx(LSM303_ADDR_MAG, LSM303_REG_OUT_X_H_M, buf, 6u);
    if (0 != ercd)
    {
        return ercd;
    }

    *p_mx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    *p_mz = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    *p_my = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    /* casts: 16-bit register image */

    return 0;
}

/**
 * drv_lsm303_read_int1_src
 * @brief   Reads INT1_SRC_A. The LSM303DLHC latches the interrupt until
 *          this register is read, so calling it after every impact is
 *          what lets a repeated tap generate a new rising edge on INT1
 *          (B4.1.1 register table, S4-I03).
 * @return  the INT1_SRC_A byte, or <0 on an I2C error
 * @context imu_task
 */
int32_t drv_lsm303_read_int1_src(void)
{
    uint8_t src  = 0u;
    int32_t ercd = lsm303_rx(LSM303_ADDR_ACC, LSM303_REG_INT1_SRC_A, &src, 1u);

    if (0 != ercd)
    {
        return ercd;
    }
    return (int32_t)src; /* cast: byte, zero-extended */
}

/**
 * drv_lsm303_identity_ok
 * @brief   Reads the magnetometer identity registers IRA/IRB/IRC.
 * @return  1 when 'H','4','3' are found, 0 on a mismatch, <0 on an
 *          I2C error
 * @context imu_task or a test
 */
int32_t drv_lsm303_identity_ok(void)
{
    uint8_t id_bytes[3u] = {0, 0, 0};
    int32_t ercd = lsm303_rx(LSM303_ADDR_MAG, LSM303_REG_IRA_M, id_bytes, 3u);

    if (0 != ercd)
    {
        return ercd;
    }
    if ((LSM303_ID_H == id_bytes[0]) && (LSM303_ID_4 == id_bytes[1]) &&
        (LSM303_ID_3 == id_bytes[2]))
    {
        return 1;
    }
    return 0;
}

/**
 * drv_lsm303_fifo_overruns
 * @brief   Returns the saturating FIFO-overrun counter (packed build:
 *          PRV_OFS_B FIFO_OVR).
 * @return  overruns 0..255
 * @context imu_task
 */
uint32_t drv_lsm303_fifo_overruns(void)
{
    return g_lsm303_fifo_ovr;
}

/**
 * drv_lsm303_bus_reset
 * @brief   Recovers a stuck bus: disables the controller, clocks SCL nine
 *          times as a GPIO (5 us half period) to release a held slave,
 *          then re-inits and rewrites every register.
 * @return  none
 * @context imu_task
 */
void drv_lsm303_bus_reset(void)
{
    uint32_t idx = 0u;

    i2c1_hw->enable = 0u; /* register-level de-init (the SDK
                             i2c_deinit lives in unlinked i2c.c) */
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_IMU_SCL, GPIO_OUT);
    gpio_put(PIN_IMU_SCL, false);

    for (idx = 0u; idx < 9u; idx++)
    {
        gpio_put(PIN_IMU_SCL, true);
        lsm303_delay_us(LSM303_RESET_HALF_US);
        gpio_put(PIN_IMU_SCL, false);
        lsm303_delay_us(LSM303_RESET_HALF_US);
    }

    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    lsm303_hw_setup();
    (void)lsm303_hw_write_config(); /* ignored: the caller is already
                                       in the I2C fault path */
}

/* ---- private functions ------------------------------------------------ */

/**
 * lsm303_hw_setup
 * @brief   Configures the I2C1 controller for fast mode (400 kHz) at the
 *          template's fixed 125 MHz system clock. The register maths is
 *          the SDK's i2c_set_baudrate, inlined because i2c.c (and with it
 *          clock_get_hz) is not linked.
 * @return  none
 * @context imu_task or usermain
 */
static void lsm303_hw_setup(void)
{
    uint32_t period =
        (LSM303_SYS_HZ + (LSM303_BUS_HZ / 2u)) / LSM303_BUS_HZ;  /* 313 */
    uint32_t lcnt     = (period * 3u) / 5u;                      /* 187 */
    uint32_t hcnt     = period - lcnt;                           /* 126 */
    uint32_t spklen   = lcnt / 16u;                              /* 11 */
    uint32_t sda_hold = ((LSM303_SYS_HZ * 3u) / 10000000u) + 1u; /* 38 */

    i2c1_hw->enable = 0u;
    i2c1_hw->con = I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_SPEED_VALUE_FAST |
                   I2C_IC_CON_IC_RESTART_EN_BITS |
                   I2C_IC_CON_IC_SLAVE_DISABLE_BITS;
    i2c1_hw->fs_scl_hcnt = hcnt;
    i2c1_hw->fs_scl_lcnt = lcnt;
    i2c1_hw->fs_spklen   = spklen;
    i2c1_hw->sda_hold    = sda_hold;
    i2c1_hw->enable      = 1u;
}

/**
 * lsm303_hw_write_config
 * @brief   Writes every register from the B4.1.1 table, accelerometer
 *          first, then magnetometer.
 * @return  0 or the first I2C error
 * @context imu_task or usermain
 */
static int32_t lsm303_hw_write_config(void)
{
    int32_t ercd = 0;

    ercd = lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_CTRL1_A, LSM303_VAL_CTRL1_A);
    if (0 == ercd)
    {
        ercd =
            lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_CTRL4_A, LSM303_VAL_CTRL4_A);
    }
    if (0 == ercd)
    {
        ercd =
            lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_CTRL5_A, LSM303_VAL_CTRL5_A);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_FIFO_CTRL_A,
                          LSM303_VAL_FIFO_CTRL_A);
    }
    if (0 == ercd)
    {
        ercd =
            lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_CTRL3_A, LSM303_VAL_CTRL3_A);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_INT1_CFG_A,
                          LSM303_VAL_INT1_CFG_A);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_INT1_THS_A,
                          (uint8_t)IMU_IMPACT_THS);
        /* cast: threshold is 0..127 */
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_ACC, LSM303_REG_INT1_DUR_A,
                          LSM303_VAL_INT1_DUR_A);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_MAG, LSM303_REG_CRA_M, LSM303_VAL_CRA_M);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_MAG, LSM303_REG_CRB_M, LSM303_VAL_CRB_M);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_ADDR_MAG, LSM303_REG_MR_M, LSM303_VAL_MR_M);
    }

    return ercd;
}

/**
 * lsm303_tx2
 * @brief   Writes one register: byte 0 = address, byte 1 = value with
 *          STOP.
 * @param   slave  7-bit slave address
 * @param   reg    register address
 * @param   val    value to write
 * @return  0 or an LSM303_ERR_* code
 * @context imu_task
 */
static int32_t lsm303_tx2(uint32_t slave, uint8_t reg, uint8_t val)
{
    uint32_t start_us = sys_tick_now_us();
    int32_t  ercd     = 0;

    i2c1_hw->tar = slave;
    lsm303_rx_drain();
    i2c1_hw->data_cmd = I2C_IC_DATA_CMD_RESTART_BITS | reg;
    i2c1_hw->data_cmd = I2C_IC_DATA_CMD_STOP_BITS | val;

    ercd = lsm303_wait_bits(start_us, I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS |
                                          I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS);
    if (0 == ercd)
    {
        ercd = lsm303_check_abort();
    }
    if (0 != ercd)
    {
        lsm303_flush();
    }

    return ercd;
}

/**
 * lsm303_rx
 * @brief   Reads count bytes starting at reg: one address byte, then
 *          count read commands, the last one with STOP.
 * @param   slave   7-bit slave address
 * @param   reg     register address
 * @param   p_out   output buffer, count bytes
 * @param   count   bytes to read
 * @return  0 or an LSM303_ERR_* code
 * @context imu_task
 */
static int32_t lsm303_rx(uint32_t slave, uint8_t reg, uint8_t * p_out,
                         uint32_t count)
{
    uint32_t start_us = sys_tick_now_us();
    uint32_t idx      = 0u;
    int32_t  ercd     = 0;

    i2c1_hw->tar = slave;
    lsm303_rx_drain();
    i2c1_hw->data_cmd = I2C_IC_DATA_CMD_RESTART_BITS | reg;

    for (idx = 0u; idx < count; idx++)
    {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS; /* read */

        if ((count - 1u) == idx)
        {
            cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        }
        i2c1_hw->data_cmd = cmd;
    }

    ercd = lsm303_wait_bits(start_us, I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS |
                                          I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS);
    if (0 == ercd)
    {
        ercd = lsm303_check_abort();
    }
    if (0 != ercd)
    {
        lsm303_flush();
        return ercd;
    }

    for (idx = 0u; idx < count; idx++)
    {
        ercd =
            lsm303_wait_bits(start_us, I2C_IC_RAW_INTR_STAT_RX_FULL_BITS |
                                           I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS);
        if (0 == ercd)
        {
            ercd = lsm303_check_abort();
        }
        if (0 != ercd)
        {
            lsm303_flush();
            return ercd;
        }
        p_out[idx] = (uint8_t)(i2c1_hw->data_cmd & 0xFFu);
        /* cast: DAT field is bits 0-7 */
    }

    ercd = lsm303_wait_bits(start_us, I2C_IC_RAW_INTR_STAT_STOP_DET_BITS);
    if (0 == ercd)
    {
        ercd = lsm303_check_abort();
    }
    if (0 != ercd)
    {
        lsm303_flush();
    }

    return ercd;
}

/**
 * lsm303_wait_bits
 * @brief   Polls raw_intr_stat until any bit of bits is set, or the
 *          1 ms per-transfer deadline passes.
 * @param   start_us  deadline base from sys_tick_now_us()
 * @param   bits      mask to wait for
 * @return  0 or LSM303_ERR_TIMEOUT
 * @context imu_task
 */
static int32_t lsm303_wait_bits(uint32_t start_us, uint32_t bits)
{
    for (;;)
    {
        if (0u != (i2c1_hw->raw_intr_stat & bits))
        {
            return 0;
        }
        if ((sys_tick_now_us() - start_us) > LSM303_BUS_TIMEOUT_US)
        {
            return LSM303_ERR_TIMEOUT;
        }
    }
}

/**
 * lsm303_check_abort
 * @brief   Handles a TX abort: clears the flag, flushes the command
 *          FIFO by toggling IC_ENABLE (configuration is preserved),
 *          returns an error.
 * @return  0 when no abort is pending, else LSM303_ERR_ABORT
 * @context imu_task
 */
static int32_t lsm303_check_abort(void)
{
    if (0u != (i2c1_hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS))
    {
        (void)i2c1_hw->clr_tx_abrt; /* read clears; always reads 0 */
        i2c1_hw->enable = 0u;       /* flush the queued commands    */
        i2c1_hw->enable = 1u;
        return LSM303_ERR_ABORT;
    }

    return 0;
}

/**
 * lsm303_flush
 * @brief   Recovers the controller after a failed transfer: clears the
 *          TX abort latch, drops any queued command and drains the RX
 *          FIFO, so the next transaction starts clean. Toggling
 *          IC_ENABLE discards the FIFOs but preserves IC_CON, the
 *          baud-rate counters, IC_TAR and the other configuration.
 * @return  none
 * @context imu_task
 */
static void lsm303_flush(void)
{
    (void)i2c1_hw->clr_tx_abrt; /* read clears; always reads 0 */
    i2c1_hw->enable = 0u;
    i2c1_hw->enable = 1u;
    lsm303_rx_drain();
}

/**
 * lsm303_rx_drain
 * @brief   Empties the RX FIFO before a transaction starts.
 * @return  none
 * @context imu_task
 */
static void lsm303_rx_drain(void)
{
    while (0u != (i2c1_hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_RX_FULL_BITS))
    {
        (void)i2c1_hw->data_cmd; /* ignored: discarding stale data */
    }
}

/**
 * lsm303_delay_us
 * @brief   Busy-wait of at most a few microseconds (5 us here), which is
 *          inside the 20 us busy-wait limit for tasks.
 * @param   delay_us  delay, us
 * @return  none
 * @context imu_task
 */
static void lsm303_delay_us(uint32_t delay_us)
{
    uint32_t start_us = sys_tick_now_us();

    while ((sys_tick_now_us() - start_us) < delay_us)
    {
        /* bounded wait: LSM303_RESET_HALF_US is 5 us */
    }
}

/*** end of file ***/
