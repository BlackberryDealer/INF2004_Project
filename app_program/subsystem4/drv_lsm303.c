/**
 * @file    drv_lsm303.c
 * @brief   Register-level LSM303D driver on I2C1 (400 kHz): accelerometer
 *          FIFO stream, magnetometer, impact interrupt (INT1), bus reset.
 * @owner   Buddy 4 (IMU, terrain)
 * @kind    drv
 * @reqs    FR4, FR5, FR13, FR18, NFR7
 *
 * SPEC-QUESTION SQ-B4-2: spec B4.1.1 targets the LSM303DLHC (accelerometer
 * at 0x19, magnetometer at 0x1E, identity 'H','4','3'). The GY-511 board
 * the team actually holds carries an LSM303D: single address 0x1D and
 * WHO_AM_I (0x0F) = 0x49. Measured on the board by Buddy 5, so this file
 * was ported to that part instead of guessing. Differences from B4.1.1:
 * one I2C address for both sensors; the LSM303D control bit layouts in
 * CTRL0..CTRL7; 16-bit acceleration at 0.122 mg/LSB for +-4 g (B4.1.1
 * assumed 12-bit left-justified at 2 mg/LSB after /16); the impact
 * interrupt is interrupt generator 1 (IG_CFG1/IG_SRC1/IG_THS1/IG_DUR1);
 * the magnetometer reads 0x08..0x0D as plain little-endian X, Y, Z. The
 * FIFO_SRC bit layout happens to be identical, so the FIFO planner and
 * its host test are unchanged. Pending team decision before merge.
 *
 * The template cannot link the SDK's i2c.c (see build_make/pico_rp2040.mk,
 * CSC28 note C2), so every bus transaction is done here against the
 * hardware/structs/i2c.h register map: IC_CON, IC_TAR, IC_DATA_CMD,
 * IC_RAW_INTR_STAT and friends. The TX and RX FIFOs are only 16 entries
 * deep, so a read queues one command at a time and collects each byte
 * before the next (as the SDK's i2c_read_blocking does); queueing a whole
 * 4-sample burst at once (25 commands) dropped the STOP and timed out.
 * IC_TAR is written once while the block is disabled. Transfers poll with
 * a 1 ms deadline from sys_tick_now_us(); this busy-wait is the one listed
 * exception to Part A rule 7 and is measured on the bench (S4-I09).
 *
 * Timing at 400 kHz (22.5 us per byte): FIFO status read = 4 bytes
 * (90 us), 2 samples = 15 bytes (338 us), 4 samples = 27 bytes (608 us),
 * magnetometer = 9 bytes (203 us). Budgets (B.3): 550 us normal, 750 us
 * magnetometer cycle, 800 us backlog. Re-measured for this part in
 * S4-I09 because the byte counts are unchanged but the register set is
 * not.
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

#define LSM303_ADDR (DRV_LSM303_ADDR) /* one address for accel and mag */

/* Registers (LSM303D datasheet, DocID023312; register names and bit
   positions verified against the ST register map, not assumed) */
#define LSM303_REG_CTRL0     (0x1Fu)
#define LSM303_REG_CTRL1     (0x20u)
#define LSM303_REG_CTRL2     (0x21u)
#define LSM303_REG_CTRL3     (0x22u)
#define LSM303_REG_CTRL5     (0x24u)
#define LSM303_REG_CTRL6     (0x25u)
#define LSM303_REG_CTRL7     (0x26u)
#define LSM303_REG_OUT_X_L_A (0x28u)
#define LSM303_REG_FIFO_CTRL (0x2Eu)
#define LSM303_REG_FIFO_SRC  (0x2Fu)
#define LSM303_REG_IG_CFG1   (0x30u)
#define LSM303_REG_IG_SRC1   (0x31u)
#define LSM303_REG_IG_THS1   (0x32u)
#define LSM303_REG_IG_DUR1   (0x33u)

/* Magnetometer data and identity */
#define LSM303_REG_OUT_X_L_M (0x08u)
#define LSM303_REG_WHO_AM_I  (0x0Fu)

/* Sub-address bit 7: the LSM303D auto-increments the register address in
   a multi-byte read only when it is set, for the magnetometer as well as
   the accelerometer (the DLHC magnetometer incremented on its own). */
#define LSM303_AUTO_INC (0x80u)

/* Register values (LSM303D; each bit position taken from the datasheet
   register tables on pages 34-43 of DocID023312) */
#define LSM303_VAL_CTRL0     (0x40u) /* FIFO_EN, bit 6                  */
#define LSM303_VAL_CTRL1     (0x7Fu) /* AODR 200 Hz, BDU, X/Y/Z on      */
#define LSM303_VAL_CTRL2     (0x08u) /* AFS = +-4 g, ABW 773 Hz         */
#define LSM303_VAL_CTRL3     (0x20u) /* INT1_IG1, bit 5                 */
#define LSM303_VAL_CTRL5     (0x71u) /* M_RES high, M_ODR 100 = 50 Hz,
                                         LIR1 (bits 6-5, 4-2, 0)         */
#define LSM303_VAL_CTRL6     (0x00u) /* MFS = +-2 gauss                 */
#define LSM303_VAL_CTRL7     (0x00u) /* MD = continuous conversion      */
#define LSM303_VAL_FIFO_CTRL (0x40u) /* FM = stream mode                */
#define LSM303_VAL_IG_CFG1   (0x0Au) /* OR, X-high or Y-high event      */
#define LSM303_VAL_IG_DUR1   (0x00u)

/* Identity byte expected from WHO_AM_I (0x0F) */
#define LSM303_WHO_AM_I_D (DRV_LSM303_WHO_AM_I)

/* FIFO_SRC bits (LSM303D Table 60: FTH, OVRN, EMPTY, FSS[4:0]) */
#define LSM303_FIFO_SRC_FSS_MASK (0x1Fu) /* unread samples, bits 0-4   */
#define LSM303_FIFO_SRC_OVRN     (0x40u) /* overrun, bit 6             */

/* Bus timings */
#define LSM303_BUS_TIMEOUT_US (1000u)      /* per-transfer deadline      */
#define LSM303_ENABLE_WAIT_US (100u) /* IC_ENABLE / ABORT settle bound */
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
static int32_t lsm303_tx2(uint8_t reg, uint8_t val);
static int32_t lsm303_rx(uint8_t reg, uint8_t * p_out, uint32_t count);
static void    lsm303_begin(void);
static int32_t lsm303_wait_rx(uint32_t start_us);
static int32_t lsm303_wait_stop(uint32_t start_us);
static void    lsm303_recover(void);
static void    lsm303_set_enable(bool b_on);
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
    /* The GY-511 module carries its own pull-ups (A18, S4-I07). The
       internal ~50 kOhm pull-ups are added as well so the lines still idle
       high if the module's are missing or weak; in parallel they change
       the module's value by only a few percent. */
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);
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

    ercd = lsm303_rx(LSM303_REG_FIFO_SRC, &fifo_src, 1u);
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
        ercd = lsm303_rx((uint8_t)(LSM303_REG_OUT_X_L_A | LSM303_AUTO_INC),
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

    ercd = lsm303_rx((uint8_t)(LSM303_REG_OUT_X_L_A | LSM303_AUTO_INC),
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
 * @brief   One magnetometer read: 6 bytes from OUT_X_L_M, in the LSM303D
 *          order X_L, X_H, Y_L, Y_H, Z_L, Z_H (little-endian, plain XYZ;
 *          the DLHC's X/Z/Y order does not apply here).
 *          The sub-address carries LSM303_AUTO_INC: without it the
 *          LSM303D returns OUT_X_L_M six times.
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

    ercd = lsm303_rx((uint8_t)(LSM303_REG_OUT_X_L_M | LSM303_AUTO_INC), buf,
                     6u);
    if (0 != ercd)
    {
        return ercd;
    }

    *p_mx = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    *p_my = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    *p_mz = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    /* casts: 16-bit register image */

    return 0;
}

/**
 * drv_lsm303_read_int1_src
 * @brief   Reads IG_SRC1. Reading it clears the IA bit and the interrupt
 *          line, and (with LIR1 set in CTRL5) releases the latch, so
 *          calling it after every impact is what lets a repeated tap
 *          generate a new edge on INT1 (LSM303D datasheet 8.31, S4-I03).
 * @return  the IG_SRC1 byte, or <0 on an I2C error
 * @context imu_task
 */
int32_t drv_lsm303_read_int1_src(void)
{
    uint8_t src  = 0u;
    int32_t ercd = lsm303_rx(LSM303_REG_IG_SRC1, &src, 1u);

    if (0 != ercd)
    {
        return ercd;
    }
    return (int32_t)src; /* cast: byte, zero-extended */
}

/**
 * drv_lsm303_identity_ok
 * @brief   Reads WHO_AM_I and compares it with the LSM303D value 0x49.
 * @return  1 when 0x49 is found, 0 on a mismatch, <0 on an I2C error
 * @context imu_task or a test
 */
int32_t drv_lsm303_identity_ok(void)
{
    uint8_t id_byte = 0u;
    int32_t ercd    = lsm303_rx(LSM303_REG_WHO_AM_I, &id_byte, 1u);

    if (0 != ercd)
    {
        return ercd;
    }
    if (LSM303_WHO_AM_I_D == id_byte)
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

    lsm303_set_enable(false); /* register-level de-init (the SDK
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

    /* IC_CON, IC_TAR and the SCL counts are only writable while the block
       is disabled (RP2040 datasheet, I2C chapter), so they are all set
       here, once, between disable and enable. One slave address serves
       both sensors, so IC_TAR never changes afterwards. */
    lsm303_set_enable(false);
    i2c1_hw->con = I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_SPEED_VALUE_FAST |
                   I2C_IC_CON_IC_RESTART_EN_BITS |
                   I2C_IC_CON_IC_SLAVE_DISABLE_BITS;
    i2c1_hw->tar         = LSM303_ADDR;
    i2c1_hw->fs_scl_hcnt = hcnt;
    i2c1_hw->fs_scl_lcnt = lcnt;
    i2c1_hw->fs_spklen   = spklen;
    i2c1_hw->sda_hold    = sda_hold;
    lsm303_set_enable(true);
    lsm303_begin();
}

/**
 * lsm303_hw_write_config
 * @brief   Writes the LSM303D control registers: accelerometer 200 Hz at
 *          +-4 g with the FIFO in stream mode, interrupt generator 1 on
 *          INT1 for the impact threshold, magnetometer at 50 Hz high
 *          resolution. Register and value choices are in the define
 *          block above.
 * @return  0 or the first I2C error
 * @context imu_task or usermain
 */
static int32_t lsm303_hw_write_config(void)
{
    int32_t ercd = 0;

    ercd = lsm303_tx2(LSM303_REG_CTRL0, LSM303_VAL_CTRL0);
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL1, LSM303_VAL_CTRL1);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL2, LSM303_VAL_CTRL2);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL5, LSM303_VAL_CTRL5);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL6, LSM303_VAL_CTRL6);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL7, LSM303_VAL_CTRL7);
    }
    if (0 == ercd)
    {
        ercd =
            lsm303_tx2(LSM303_REG_FIFO_CTRL, LSM303_VAL_FIFO_CTRL);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_CTRL3, LSM303_VAL_CTRL3);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_IG_CFG1, LSM303_VAL_IG_CFG1);
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_IG_THS1,
                          (uint8_t)IMU_IMPACT_THS);
        /* cast: threshold is 0..127 */
    }
    if (0 == ercd)
    {
        ercd = lsm303_tx2(LSM303_REG_IG_DUR1, LSM303_VAL_IG_DUR1);
    }

    return ercd;
}

/**
 * lsm303_tx2
 * @brief   Writes one register: byte 0 = address, byte 1 = value with
 *          STOP, then waits for the STOP condition on the bus (STOP_DET),
 *          not just for an empty TX FIFO, so the write has really ended
 *          before the next transfer starts.
 * @param   reg  register address
 * @param   val  value to write
 * @return  0 or an LSM303_ERR_* code
 * @context imu_task
 */
static int32_t lsm303_tx2(uint8_t reg, uint8_t val)
{
    uint32_t start_us = sys_tick_now_us();
    int32_t  ercd     = 0;

    lsm303_begin();
    i2c1_hw->data_cmd = reg;
    i2c1_hw->data_cmd = I2C_IC_DATA_CMD_STOP_BITS | val;
    /* 2 commands: far below the 16-entry TX FIFO */

    ercd = lsm303_wait_stop(start_us);
    if (0 != ercd)
    {
        lsm303_recover();
    }

    return ercd;
}

/**
 * lsm303_rx
 * @brief   Reads count bytes starting at reg: the address byte, then one
 *          read command at a time. Each byte is collected from the RX FIFO
 *          before the next command is queued (as the SDK's
 *          i2c_read_blocking does), so at most 2 entries are ever in the
 *          16-entry TX FIFO and the RX FIFO cannot overflow, whatever
 *          count is. The last command carries STOP.
 * @param   reg     register address (bit 7 set for auto-increment)
 * @param   p_out   output buffer, count bytes
 * @param   count   bytes to read, >= 1
 * @return  0 or an LSM303_ERR_* code
 * @context imu_task
 */
static int32_t lsm303_rx(uint8_t reg, uint8_t * p_out, uint32_t count)
{
    uint32_t start_us = sys_tick_now_us();
    uint32_t idx      = 0u;
    int32_t  ercd     = 0;

    lsm303_begin();
    i2c1_hw->data_cmd = reg;

    for (idx = 0u; (idx < count) && (0 == ercd); idx++)
    {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS; /* read */

        if (0u == idx)
        {
            cmd |= I2C_IC_DATA_CMD_RESTART_BITS; /* write -> read turn */
        }
        if ((count - 1u) == idx)
        {
            cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        }
        i2c1_hw->data_cmd = cmd;

        ercd = lsm303_wait_rx(start_us);
        if (0 == ercd)
        {
            p_out[idx] = (uint8_t)(i2c1_hw->data_cmd & 0xFFu);
            /* cast: DAT field is bits 0-7 */
        }
    }

    if (0 == ercd)
    {
        ercd = lsm303_wait_stop(start_us);
    }
    if (0 != ercd)
    {
        lsm303_recover();
    }

    return ercd;
}

/**
 * lsm303_begin
 * @brief   Clears what a previous transfer may have left behind before a
 *          new one starts: stale RX bytes and the STOP_DET latch.
 * @return  none
 * @context imu_task
 */
static void lsm303_begin(void)
{
    lsm303_rx_drain();
    (void)i2c1_hw->clr_stop_det; /* read clears; always reads 0 */
}

/**
 * lsm303_wait_rx
 * @brief   Polls until one received byte is in the RX FIFO, a TX abort
 *          (NACK, lost arbitration) is flagged, or the per-transfer
 *          deadline passes.
 * @param   start_us  deadline base from sys_tick_now_us()
 * @return  0, LSM303_ERR_ABORT or LSM303_ERR_TIMEOUT
 * @context imu_task
 */
static int32_t lsm303_wait_rx(uint32_t start_us)
{
    for (;;)
    {
        if (0u != (i2c1_hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS))
        {
            return LSM303_ERR_ABORT;
        }
        if (0u != i2c1_hw->rxflr)
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
 * lsm303_wait_stop
 * @brief   Polls until the STOP condition has been sent (STOP_DET), then
 *          clears it. A TX abort also ends in STOP, so the abort flag is
 *          checked first and wins.
 * @param   start_us  deadline base from sys_tick_now_us()
 * @return  0, LSM303_ERR_ABORT or LSM303_ERR_TIMEOUT
 * @context imu_task
 */
static int32_t lsm303_wait_stop(uint32_t start_us)
{
    for (;;)
    {
        uint32_t const stat = i2c1_hw->raw_intr_stat;

        if (0u != (stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS))
        {
            return LSM303_ERR_ABORT;
        }
        if (0u != (stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS))
        {
            (void)i2c1_hw->clr_stop_det; /* read clears; always reads 0 */
            return 0;
        }
        if ((sys_tick_now_us() - start_us) > LSM303_BUS_TIMEOUT_US)
        {
            return LSM303_ERR_TIMEOUT;
        }
    }
}

/**
 * lsm303_recover
 * @brief   Recovers the controller after a failed transfer. ABORT makes
 *          the master send STOP and flush the TX FIFO even when it is
 *          holding the bus mid-transfer; then the TX abort latch is
 *          cleared and the block is disabled and re-enabled (waiting for
 *          IC_ENABLE_STATUS each time, both bounded), which keeps IC_CON,
 *          IC_TAR and the baud-rate counters. Finally the RX FIFO and the
 *          STOP_DET latch are cleared so the next transfer starts clean.
 * @return  none
 * @context imu_task
 */
static void lsm303_recover(void)
{
    uint32_t start_us = sys_tick_now_us();

    i2c1_hw->enable = I2C_IC_ENABLE_ENABLE_BITS | I2C_IC_ENABLE_ABORT_BITS;
    while ((0u != (i2c1_hw->enable & I2C_IC_ENABLE_ABORT_BITS)) &&
           ((sys_tick_now_us() - start_us) < LSM303_ENABLE_WAIT_US))
    {
        /* bounded: the abort completes within one byte time (~23 us) */
    }
    (void)i2c1_hw->clr_tx_abrt; /* read clears; always reads 0 */

    lsm303_set_enable(false);
    lsm303_set_enable(true);
    lsm303_begin();
}

/**
 * lsm303_set_enable
 * @brief   Writes IC_ENABLE and waits (bounded) until IC_ENABLE_STATUS
 *          follows, so registers that are only writable while the block
 *          is disabled (IC_TAR, IC_CON, the SCL counts) really are.
 * @param   b_on  true to enable, false to disable
 * @return  none
 * @context imu_task or usermain
 */
static void lsm303_set_enable(bool b_on)
{
    uint32_t const want     = b_on ? 1u : 0u;
    uint32_t       start_us = sys_tick_now_us();

    i2c1_hw->enable = want;
    while (((i2c1_hw->enable_status & 1u) != want) &&
           ((sys_tick_now_us() - start_us) < LSM303_ENABLE_WAIT_US))
    {
        /* bounded: a few ic_clk cycles when the bus is idle */
    }
}

/**
 * lsm303_rx_drain
 * @brief   Empties the RX FIFO before a transaction starts.
 * @return  none
 * @context imu_task
 */
static void lsm303_rx_drain(void)
{
    while (0u != i2c1_hw->rxflr)
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
