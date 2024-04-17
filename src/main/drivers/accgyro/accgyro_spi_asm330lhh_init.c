/*
 * This file is part of Cleanflight and ATBetaflight (forked by flightng).
 *
 * Cleanflight and ATBetaflight (forked by flightng) are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and ATBetaflight (forked by flightng) are distributed in the hope that they
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

#ifdef USE_ACCGYRO_ASM330LHH

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_asm330lhh.h"
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
#define ASM330LHH_MAX_SPI_CLK_HZ 10000000

#define ASM330LHH_CHIP_ID 0x6B

// ASM330LHH register configuration values
typedef enum {
    ASM330LHH_VAL_COUNTER_BDR1_DDRY_PM = BIT(7),// (bit 7) enable data ready pulsed mode
    ASM330LHH_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    ASM330LHH_VAL_INT2_CTRL = 0x00,             // disable gyro data ready interrupt pin 2
    ASM330LHH_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    ASM330LHH_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    ASM330LHH_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    ASM330LHH_VAL_CTRL1_XL_ODR6664 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    ASM330LHH_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    ASM330LHH_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    ASM330LHH_VAL_CTRL1_XL_LPF1 = 0x00,         // accelerometer output from LPF1
    ASM330LHH_VAL_CTRL1_XL_LPF2 = 0x01,         // accelerometer output from LPF2
    ASM330LHH_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    ASM330LHH_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    // ASM330LHH_VAL_CTRL3_C_BDU = BIT(6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    ASM330LHH_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    ASM330LHH_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    ASM330LHH_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    ASM330LHH_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads
    ASM330LHH_VAL_CTRL4_C_DRDY_MASK = BIT(3),   // (bit 3) data ready interrupt mask
    ASM330LHH_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    ASM330LHH_VAL_CTRL4_C_LPF1_SEL_G = BIT(1),  // (bit 1) enable gyro LPF1
    ASM330LHH_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    ASM330LHH_VAL_CTRL6_C_FTYPE_297HZ = 0x00,   // (bits 2:0) gyro LPF1 cutoff 297Hz
    ASM330LHH_VAL_CTRL6_C_FTYPE_223HZ = 0x01,   // (bits 2:0) gyro LPF1 cutoff 223Hz
    ASM330LHH_VAL_CTRL6_C_FTYPE_154HZ = 0x02,   // (bits 2:0) gyro LPF1 cutoff 154Hz
    ASM330LHH_VAL_CTRL6_C_FTYPE_470HZ = 0x03,   // (bits 2:0) gyro LPF1 cutoff 470Hz
    ASM330LHH_VAL_CTRL7_G_HP_EN_G = BIT(6),     // (bit 6) enable gyro high-pass filter
    ASM330LHH_VAL_CTRL7_G_HPM_G_16 = 0x00,      // (bits 5:4) gyro HPF cutoff 16mHz
    ASM330LHH_VAL_CTRL7_G_HPM_G_65 = 0x01,      // (bits 5:4) gyro HPF cutoff 65mHz
    ASM330LHH_VAL_CTRL7_G_HPM_G_260 = 0x02,     // (bits 5:4) gyro HPF cutoff 260mHz
    ASM330LHH_VAL_CTRL7_G_HPM_G_1040 = 0x03,    // (bits 5:4) gyro HPF cutoff 1.04Hz
    ASM330LHH_VAL_CTRL9_XL_DEVICE_CONF = BIT(1),// (bit 1) Enables the proper device configuration
} asm330lhhConfigValues_e;

// ASM330LHH register configuration bit masks
typedef enum {
    ASM330LHH_MASK_COUNTER_BDR1 = 0x80,    // 0b10000000
    ASM330LHH_MASK_CTRL3_C = 0x3C,         // 0b00111100
    ASM330LHH_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    ASM330LHH_MASK_CTRL4_C = 0x0E,         // 0b00001110
    ASM330LHH_MASK_CTRL6_C = 0x17,         // 0b00010111
    ASM330LHH_MASK_CTRL7_G = 0x70,         // 0b01110000
    ASM330LHH_MASK_CTRL9_XL = 0x02,        // 0b00000010
} asm330lhhConfigMasks_e;

uint8_t asm330lhhDetect(const extDevice_t *dev)
{
    uint8_t chipID = 0;

    if (busReadRegisterBuffer(dev, ASM330LHH_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == ASM330LHH_CHIP_ID) {
            return ASM330LHH_SPI;
        }
    }

    return MPU_NONE;
}

static void asm330lhhWriteRegister(const extDevice_t *dev, asm330lhhRegister_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void asm330lhhWriteRegisterBits(const extDevice_t *dev, asm330lhhRegister_e registerID, asm330lhhConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        asm330lhhWriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getAsmDlpfBandwidth()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return ASM330LHH_VAL_CTRL6_C_FTYPE_223HZ;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return ASM330LHH_VAL_CTRL6_C_FTYPE_297HZ;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return ASM330LHH_VAL_CTRL6_C_FTYPE_470HZ;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return ASM330LHH_VAL_CTRL6_C_FTYPE_470HZ;
    }
    return 0;
}

static void asm330lhhConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL3_C, ASM330LHH_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure data ready pulsed mode
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_COUNTER_BDR1, ASM330LHH_MASK_COUNTER_BDR1, ASM330LHH_VAL_COUNTER_BDR1_DDRY_PM, 0);

    // Configure interrupt pin 1 for gyro data ready only
    asm330lhhWriteRegister(dev, ASM330LHH_REG_INT1_CTRL, ASM330LHH_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    asm330lhhWriteRegister(dev, ASM330LHH_REG_INT2_CTRL, ASM330LHH_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF2 output (default with ODR/4 cutoff)
    asm330lhhWriteRegister(dev, ASM330LHH_REG_CTRL1_XL, (ASM330LHH_VAL_CTRL1_XL_ODR833 << 4) | (ASM330LHH_VAL_CTRL1_XL_16G << 2) | (ASM330LHH_VAL_CTRL1_XL_LPF2 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    asm330lhhWriteRegister(dev, ASM330LHH_REG_CTRL2_G, (ASM330LHH_VAL_CTRL2_G_ODR6664 << 4) | (ASM330LHH_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL3_C, ASM330LHH_MASK_CTRL3_C, (ASM330LHH_VAL_CTRL3_C_H_LACTIVE | ASM330LHH_VAL_CTRL3_C_PP_OD | ASM330LHH_VAL_CTRL3_C_SIM | ASM330LHH_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; enable gyro LPF1
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL4_C, ASM330LHH_MASK_CTRL4_C, (ASM330LHH_VAL_CTRL4_C_DRDY_MASK | ASM330LHH_VAL_CTRL4_C_I2C_DISABLE | ASM330LHH_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; set gyro LPF1 cutoff according to gyro_hardware_lpf setting
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL6_C, ASM330LHH_MASK_CTRL6_C, (ASM330LHH_VAL_CTRL6_C_XL_HM_MODE | getAsmDlpfBandwidth()), 1);

    // // Configure control register 7
    // asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL7_G, ASM330LHH_MASK_CTRL7_G, (ASM330LHH_VAL_CTRL7_G_HP_EN_G | ASM330LHH_VAL_CTRL7_G_HPM_G_16), 1);

    // Configure control register 9
    // Enables the proper device configuration
    asm330lhhWriteRegisterBits(dev, ASM330LHH_REG_CTRL9_XL, ASM330LHH_MASK_CTRL9_XL, ASM330LHH_VAL_CTRL9_XL_DEVICE_CONF, 1);
}

#ifdef USE_GYRO_EXTI
static void asm330lhhIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, asm330lhhExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void asm330lhhSpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    asm330lhhConfig(gyro);

#ifdef USE_GYRO_EXTI
    asm330lhhIntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(ASM330LHH_MAX_SPI_CLK_HZ));
}

static void asm330lhhSpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool asm330lhhSpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ASM330LHH_SPI) {
        return false;
    }

    acc->initFn = asm330lhhSpiAccInit;
    acc->readFn = asm330lhhAccRead;

    return true;
}

bool asm330lhhSpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ASM330LHH_SPI) {
        return false;
    }

    gyro->initFn = asm330lhhSpiGyroInit;
    gyro->readFn = asm330lhhGyroRead;
    gyro->scale = 0.07f;

    return true;
}
#endif // USE_ACCGYRO_ASM330LHH
