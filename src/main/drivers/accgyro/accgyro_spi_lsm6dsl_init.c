/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_LSM6DSL

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsl.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 10 MHz max SPI frequency
#define LSM6DSL_MAX_SPI_CLK_HZ 10000000

#define LSM6DSL_CHIP_ID 0x6A

// LSM6DSL register configuration values
typedef enum {
    LSM6DSL_VAL_DRDY_PULSED_CFG_G_DRDY_PULSED = BIT(7),// (bit 7) enable data ready pulsed mode
    LSM6DSL_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    LSM6DSL_VAL_INT2_CTRL = 0x00,             // disable gyro data ready interrupt pin 2
    LSM6DSL_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    LSM6DSL_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    LSM6DSL_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    LSM6DSL_VAL_CTRL1_XL_ODR6664 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    LSM6DSL_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    LSM6DSL_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    LSM6DSL_VAL_CTRL1_XL_LPF1 = 0x00,         // accelerometer output from LPF1
    LSM6DSL_VAL_CTRL1_XL_LPF2 = 0x01,         // accelerometer output from LPF2
    LSM6DSL_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    LSM6DSL_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    // LSM6DSL_VAL_CTRL3_C_BDU = BIT(6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    LSM6DSL_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    LSM6DSL_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    LSM6DSL_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    LSM6DSL_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads
    LSM6DSL_VAL_CTRL4_C_DRDY_MASK = BIT(3),   // (bit 3) data ready interrupt mask
    LSM6DSL_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    LSM6DSL_VAL_CTRL4_C_LPF1_SEL_G = BIT(1),  // (bit 1) enable gyro LPF1
    LSM6DSL_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    LSM6DSL_VAL_CTRL6_C_FTYPE_315HZ = 0x00,   // (bits 1:0) gyro LPF1 cutoff 315Hz
    LSM6DSL_VAL_CTRL6_C_FTYPE_237HZ = 0x01,   // (bits 1:0) gyro LPF1 cutoff 237Hz
    LSM6DSL_VAL_CTRL6_C_FTYPE_173HZ = 0x02,   // (bits 1:0) gyro LPF1 cutoff 173Hz
    LSM6DSL_VAL_CTRL6_C_FTYPE_937HZ = 0x03,   // (bits 1:0) gyro LPF1 cutoff 937Hz
    LSM6DSL_VAL_CTRL7_G_HP_EN_G = BIT(6),   // (bit 6) enable gyro high-pass filter
    LSM6DSL_VAL_CTRL7_G_HPM_G_16 = 0x00,      // (bits 5:4) gyro HPF cutoff 16mHz
    LSM6DSL_VAL_CTRL7_G_HPM_G_65 = 0x01,      // (bits 5:4) gyro HPF cutoff 65mHz
    LSM6DSL_VAL_CTRL7_G_HPM_G_260 = 0x02,     // (bits 5:4) gyro HPF cutoff 260mHz
    LSM6DSL_VAL_CTRL7_G_HPM_G_1040 = 0x03,    // (bits 5:4) gyro HPF cutoff 1.04Hz
} lsm6dslConfigValues_e;

// LSM6DSL register configuration bit masks
typedef enum {
    LSM6DSL_MASK_DRDY_PULSED_CFG_G = 0x80,    // 0b10000000
    LSM6DSL_MASK_CTRL3_C = 0x3C,         // 0b00111100
    LSM6DSL_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    LSM6DSL_MASK_CTRL4_C = 0x0E,         // 0b00001110
    LSM6DSL_MASK_CTRL6_C = 0x13,         // 0b00010011
    LSM6DSL_MASK_CTRL7_G = 0x70,         // 0b01110000
} lsm6dslConfigMasks_e;

uint8_t lsm6dslDetect(const extDevice_t *dev)
{
    uint8_t chipID = 0;

    if (busReadRegisterBuffer(dev, LSM6DSL_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == LSM6DSL_CHIP_ID) {
            return LSM6DSL_SPI;
        }
    }

    return MPU_NONE;
}

static void lsm6dslWriteRegister(const extDevice_t *dev, lsm6dslRegister_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void lsm6dslWriteRegisterBits(const extDevice_t *dev, lsm6dslRegister_e registerID, lsm6dslConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        lsm6dslWriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getLsmDlpfBandwidth()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return LSM6DSL_VAL_CTRL6_C_FTYPE_237HZ;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return LSM6DSL_VAL_CTRL6_C_FTYPE_315HZ;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return LSM6DSL_VAL_CTRL6_C_FTYPE_937HZ;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return LSM6DSL_VAL_CTRL6_C_FTYPE_937HZ;
    }
    return 0;
}

static void lsm6dslConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_CTRL3_C, LSM6DSL_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure data ready pulsed mode
    lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_DRDY_PULSED_CFG_G, LSM6DSL_MASK_DRDY_PULSED_CFG_G, LSM6DSL_VAL_DRDY_PULSED_CFG_G_DRDY_PULSED, 0);

    // Configure interrupt pin 1 for gyro data ready only
    lsm6dslWriteRegister(dev, LSM6DSL_REG_INT1_CTRL, LSM6DSL_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6dslWriteRegister(dev, LSM6DSL_REG_INT2_CTRL, LSM6DSL_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF2 output (default with ODR/4 cutoff)
    lsm6dslWriteRegister(dev, LSM6DSL_REG_CTRL1_XL, (LSM6DSL_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSL_VAL_CTRL1_XL_16G << 2) | (LSM6DSL_VAL_CTRL1_XL_LPF2 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6dslWriteRegister(dev, LSM6DSL_REG_CTRL2_G, (LSM6DSL_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSL_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_CTRL3_C, LSM6DSL_MASK_CTRL3_C, (LSM6DSL_VAL_CTRL3_C_H_LACTIVE | LSM6DSL_VAL_CTRL3_C_PP_OD | LSM6DSL_VAL_CTRL3_C_SIM | LSM6DSL_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; enable gyro LPF1
    lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_CTRL4_C, LSM6DSL_MASK_CTRL4_C, (LSM6DSL_VAL_CTRL4_C_DRDY_MASK | LSM6DSL_VAL_CTRL4_C_I2C_DISABLE | LSM6DSL_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; set gyro LPF1 cutoff according to gyro_hardware_lpf setting
    lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_CTRL6_C, LSM6DSL_MASK_CTRL6_C, (LSM6DSL_VAL_CTRL6_C_XL_HM_MODE | getLsmDlpfBandwidth()), 1);

    // Configure control register 7
    // lsm6dslWriteRegisterBits(dev, LSM6DSL_REG_CTRL7_G, LSM6DSL_MASK_CTRL7_G, (LSM6DSL_VAL_CTRL7_G_HP_EN_G | LSM6DSL_VAL_CTRL7_G_HPM_G_16), 1);

}

#ifdef USE_GYRO_EXTI
static void lsm6dslIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, lsm6dslExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void lsm6dslSpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    lsm6dslConfig(gyro);

#ifdef USE_GYRO_EXTI
    lsm6dslIntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSL_MAX_SPI_CLK_HZ));
}

static void lsm6dslSpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool lsm6dslSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSL_SPI) {
        return false;
    }

    acc->initFn = lsm6dslSpiAccInit;
    acc->readFn = lsm6dslAccRead;

    return true;
}

bool lsm6dslSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSL_SPI) {
        return false;
    }

    gyro->initFn = lsm6dslSpiGyroInit;
    gyro->readFn = lsm6dslGyroRead;
    gyro->scale = 0.07f;

    return true;
}
#endif // USE_ACCGYRO_LSM6DSL
