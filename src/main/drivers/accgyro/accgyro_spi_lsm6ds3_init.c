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

#ifdef USE_ACCGYRO_LSM6DS3

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6ds3.h"
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
#define LSM6DS3_MAX_SPI_CLK_HZ 10000000

#define LSM6DS3_CHIP_ID 0x69

// LSM6DS3 register configuration values
typedef enum {
    LSM6DS3_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    LSM6DS3_VAL_INT2_CTRL = 0x00,             // disable gyro data ready interrupt pin 2
    LSM6DS3_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    LSM6DS3_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    LSM6DS3_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    LSM6DS3_VAL_CTRL1_XL_ODR6664 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    LSM6DS3_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    LSM6DS3_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    LSM6DS3_VAL_CTRL1_XL_BW_XL = 0x01,        // accelerometer filter bandwidth 200Hz
    LSM6DS3_VAL_CTRL2_G_ODR1667 = 0x08,       // gyro 1667hz output data rate
    LSM6DS3_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    LSM6DS3_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    LSM6DS3_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    LSM6DS3_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    LSM6DS3_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads
    LSM6DS3_VAL_CTRL4_C_XL_BW_SCAL_ODR = BIT(7),// (bit 7) bandwidth determined by BW_XL
    LSM6DS3_VAL_CTRL4_C_DRDY_MASK = BIT(3),   // (bit 3) data ready interrupt mask
    LSM6DS3_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    LSM6DS3_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    LSM6DS3_VAL_CTRL7_G_HP_EN_G = BIT(6),     // (bit 6) enable gyro high-pass filter
    LSM6DS3_VAL_CTRL7_G_HPM_G_8 = 0x00,       // (bits 1:0) gyro HPF cutoff 8.1mHz
    LSM6DS3_VAL_CTRL7_G_HPM_G_32 = 0x01,      // (bits 1:0) gyro HPF cutoff 32.4mHz
    LSM6DS3_VAL_CTRL7_G_HPM_G_2070 = 0x02,    // (bits 1:0) gyro HPF cutoff 2.07Hz
    LSM6DS3_VAL_CTRL7_G_HPM_G_16320 = 0x03,   // (bits 1:0) gyro HPF cutoff 16.32Hz
} lsm6ds3ConfigValues_e;

// LSM6DS3 register configuration bit masks
typedef enum {
    LSM6DS3_MASK_DRDY_PULSE_CFG_G = 0x80,// 0b10000000
    LSM6DS3_MASK_CTRL3_C = 0x3C,         // 0b00111100
    LSM6DS3_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    LSM6DS3_MASK_CTRL4_C = 0x8C,         // 0b10001100
    LSM6DS3_MASK_CTRL6_C = 0x10,         // 0b00010000
    LSM6DS3_MASK_CTRL7_G = 0x70,         // 0b01000011
    LSM6DS3_MASK_CTRL9_XL = 0x02,        // 0b00000010
} lsm6ds3ConfigMasks_e;

uint8_t lsm6ds3Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;

    if (busReadRegisterBuffer(dev, LSM6DS3_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == LSM6DS3_CHIP_ID) {
            return LSM6DS3_SPI;
        }
    }

    return MPU_NONE;
}

static void lsm6ds3WriteRegister(const extDevice_t *dev, lsm6ds3Register_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void lsm6ds3WriteRegisterBits(const extDevice_t *dev, lsm6ds3Register_e registerID, lsm6ds3ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        lsm6ds3WriteRegister(dev, registerID, newValue, delayMs);
    }
}


static void lsm6ds3Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_CTRL3_C, LSM6DS3_MASK_CTRL3_C_RESET, BIT(0), 100);

    // // Configure data ready pulsed mode
    // lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_DRDY_PULSE_CFG_G, LSM6DS3_MASK_DRDY_PULSE_CFG_G, LSM6DS3_VAL_DRDY_PULSE_CFG_G_DDRY, 0);

    // Configure interrupt pin 1 for gyro data ready only
    lsm6ds3WriteRegister(dev, LSM6DS3_REG_INT1_CTRL, LSM6DS3_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6ds3WriteRegister(dev, LSM6DS3_REG_INT2_CTRL, LSM6DS3_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF2 output (default with ODR/4 cutoff)
    lsm6ds3WriteRegister(dev, LSM6DS3_REG_CTRL1_XL, (LSM6DS3_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DS3_VAL_CTRL1_XL_16G << 2) | LSM6DS3_VAL_CTRL1_XL_BW_XL, 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6ds3WriteRegister(dev, LSM6DS3_REG_CTRL2_G, (LSM6DS3_VAL_CTRL2_G_ODR1667 << 4) | (LSM6DS3_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_CTRL3_C, LSM6DS3_MASK_CTRL3_C, (LSM6DS3_VAL_CTRL3_C_H_LACTIVE | LSM6DS3_VAL_CTRL3_C_PP_OD | LSM6DS3_VAL_CTRL3_C_SIM | LSM6DS3_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; enable gyro LPF1
    lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_CTRL4_C, LSM6DS3_MASK_CTRL4_C, (LSM6DS3_VAL_CTRL4_C_XL_BW_SCAL_ODR | LSM6DS3_VAL_CTRL4_C_DRDY_MASK | LSM6DS3_VAL_CTRL4_C_I2C_DISABLE ), 1);

    // Configure control register 6
    // disable I2C interface; set gyro LPF1 cutoff according to gyro_hardware_lpf setting
    lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_CTRL6_C, LSM6DS3_MASK_CTRL6_C, LSM6DS3_VAL_CTRL6_C_XL_HM_MODE, 1);

    // Configure control register 7
    // lsm6ds3WriteRegisterBits(dev, LSM6DS3_REG_CTRL7_G, LSM6DS3_MASK_CTRL7_G, (LSM6DS3_VAL_CTRL7_G_HP_EN_G | LSM6DS3_VAL_CTRL7_G_HPM_G_32), 1);

}

#ifdef USE_GYRO_EXTI
static void lsm6ds3IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, lsm6ds3ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void lsm6ds3SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    lsm6ds3Config(gyro);

#ifdef USE_GYRO_EXTI
    lsm6ds3IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DS3_MAX_SPI_CLK_HZ));
}

static void lsm6ds3SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool lsm6ds3SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DS3_SPI) {
        return false;
    }

    acc->initFn = lsm6ds3SpiAccInit;
    acc->readFn = lsm6ds3AccRead;

    return true;
}

bool lsm6ds3SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DS3_SPI) {
        return false;
    }

    gyro->initFn = lsm6ds3SpiGyroInit;
    gyro->readFn = lsm6ds3GyroRead;
    gyro->scale = 0.07f;

    return true;
}
#endif // USE_ACCGYRO_LSM6DS3
