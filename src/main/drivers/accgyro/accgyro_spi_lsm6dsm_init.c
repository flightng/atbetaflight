/*
 * This file is part of Cleanflight.
 *
 * Cleanflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight are distributed in the hope that they
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

#ifdef USE_ACCGYRO_LSM6DSM

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsm.h"
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
#define LSM6DSM_MAX_SPI_CLK_HZ 10000000

#define LSM6DSM_CHIP_ID 0x6A

// LSM6DSM register configuration values
typedef enum {
    LSM6DSM_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    LSM6DSM_VAL_INT2_CTRL = 0x02,             // enable gyro data ready interrupt pin 2
    LSM6DSM_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    LSM6DSM_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    LSM6DSM_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    LSM6DSM_VAL_CTRL1_XL_ODR6664 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    LSM6DSM_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8g full scale
    LSM6DSM_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16g full scale
    LSM6DSM_VAL_CTRL1_XL_LPF1_BW_SEL = 0x00,  // accelerometer low pass filter 1 (doesn't matter since we are using LPF2)
    LSM6DSM_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    LSM6DSM_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 3332hz output data rate
    LSM6DSM_VAL_CTRL3_C_BDU = BIT(6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    LSM6DSM_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    LSM6DSM_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    LSM6DSM_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    LSM6DSM_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads    
    LSM6DSM_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    LSM6DSM_VAL_CTRL4_C_LPF1_SEL_G = BIT(1),  // (bit 1) enable gyro LPF1
    LSM6DSM_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    LSM6DSM_VAL_CTRL6_C_FTYPE_351HZ = 0x00,   // (bits 2:0) gyro LPF1 cutoff 351Hz
    LSM6DSM_VAL_CTRL6_C_FTYPE_237HZ = 0x01,   // (bits 2:0) gyro LPF1 cutoff 237Hz
    LSM6DSM_VAL_CTRL6_C_FTYPE_173HZ = 0x02,   // (bits 2:0) gyro LPF1 cutoff 173Hz
    LSM6DSM_VAL_CTRL6_C_FTYPE_937HZ = 0x03,   // (bits 2:0) gyro LPF1 cutoff 937Hz
    LSM6DSM_VAL_CTRL7_G_HP_EN_G = BIT(6),     // (bit 6) enable gyro HPF
    LSM6DSM_VAL_CTRL7_G_HPM_G_16 = 0x00,      // (bits 5:4) gyro HPF cutoff 16mHz
    LSM6DSM_VAL_CTRL7_G_HPM_G_65 = 0x01,      // (bits 5:4) gyro HPF cutoff 65mHz
    LSM6DSM_VAL_CTRL7_G_HPM_G_260 = 0x02,     // (bits 5:4) gyro HPF cutoff 260mHz
    LSM6DSM_VAL_CTRL7_G_HPM_G_1040 = 0x03,    // (bits 5:4) gyro HPF cutoff 1.04Hz
} lsm6dsmConfigValues_e;

// LSM6DSM register configurationg bit masks
typedef enum {
    LSM6DSM_MASK_CTRL3_C = 0x7C,         // 0b01111100
    LSM6DSM_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    LSM6DSM_MASK_CTRL4_C = 0x06,         // 0b00000110
    LSM6DSM_MASK_CTRL6_C = 0x17,         // 0b00010111
    LSM6DSM_MASK_CTRL7_G = 0x70,         // 0b01110000
} lsm6dsmConfigMasks_e;

uint8_t lsm6dsmDetect(const extDevice_t *dev)
{
    uint8_t chipID = 0;

    if (busReadRegisterBuffer(dev, LSM6DSM_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == LSM6DSM_CHIP_ID) {
            return LSM6DSM_SPI;
        }
    }

    return MPU_NONE;
}

static void lsm6dsmWriteRegister(const extDevice_t *dev, lsm6dsmRegister_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void lsm6dsmWriteRegisterBits(const extDevice_t *dev, lsm6dsmRegister_e registerID, lsm6dsmConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        lsm6dsmWriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getLsmDlpfBandwidth()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return LSM6DSM_VAL_CTRL6_C_FTYPE_237HZ;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return LSM6DSM_VAL_CTRL6_C_FTYPE_351HZ;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return LSM6DSM_VAL_CTRL6_C_FTYPE_937HZ;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return LSM6DSM_VAL_CTRL6_C_FTYPE_937HZ;
    }
    return 0;
}

static void lsm6dsmConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    lsm6dsmWriteRegisterBits(dev, LSM6DSM_REG_CTRL3_C, LSM6DSM_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure interrupt pin 1 for gyro data ready only
    lsm6dsmWriteRegister(dev, LSM6DSM_REG_INT1_CTRL, LSM6DSM_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6dsmWriteRegister(dev, LSM6DSM_REG_INT2_CTRL, LSM6DSM_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    lsm6dsmWriteRegister(dev, LSM6DSM_REG_CTRL1_XL, (LSM6DSM_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSM_VAL_CTRL1_XL_16G << 2) | (LSM6DSM_VAL_CTRL1_XL_LPF1_BW_SEL << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6dsmWriteRegister(dev, LSM6DSM_REG_CTRL2_G, (LSM6DSM_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSM_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6dsmWriteRegisterBits(dev, LSM6DSM_REG_CTRL3_C, LSM6DSM_MASK_CTRL3_C, (LSM6DSM_VAL_CTRL3_C_BDU | LSM6DSM_VAL_CTRL3_C_H_LACTIVE | LSM6DSM_VAL_CTRL3_C_PP_OD | LSM6DSM_VAL_CTRL3_C_SIM | LSM6DSM_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; enable gyro LPF1
    lsm6dsmWriteRegisterBits(dev, LSM6DSM_REG_CTRL4_C, LSM6DSM_MASK_CTRL4_C, (LSM6DSM_VAL_CTRL4_C_I2C_DISABLE | LSM6DSM_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; set LPF bandwidth according to gyro_hardware_lpf setting
    lsm6dsmWriteRegisterBits(dev, LSM6DSM_REG_CTRL6_C, LSM6DSM_MASK_CTRL6_C, (LSM6DSM_VAL_CTRL6_C_XL_HM_MODE | getLsmDlpfBandwidth()), 1);

    // Configure control register 7
    // enable gyro high pass filter; set cutoff to 16mHz
    lsm6dsmWriteRegisterBits(dev, LSM6DSM_REG_CTRL7_G, LSM6DSM_MASK_CTRL7_G, (LSM6DSM_VAL_CTRL7_G_HP_EN_G | LSM6DSM_VAL_CTRL7_G_HPM_G_16), 1);
    // TODO: test all HPF settings
}

#ifdef USE_GYRO_EXTI
static void lsm6dsmIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, lsm6dsmExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void lsm6dsmSpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    lsm6dsmConfig(gyro);

#ifdef USE_GYRO_EXTI
    lsm6dsmIntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(LSM6DSM_MAX_SPI_CLK_HZ));
}

static void lsm6dsmSpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool lsm6dsmSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != LSM6DSM_SPI) {
        return false;
    }

    acc->initFn = lsm6dsmSpiAccInit;
    acc->readFn = lsm6dsmAccRead;

    return true;
}

bool lsm6dsmSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != LSM6DSM_SPI) {
        return false;
    }

    gyro->initFn = lsm6dsmSpiGyroInit;
    gyro->readFn = lsm6dsmGyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_ACCGYRO_LSM6DSM
