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

#ifdef USE_ACCGYRO_SH3001

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_sh3001.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 8 MHz max SPI frequency
#define SH3001_MAX_SPI_CLK_HZ 8000000

#define SH3001_CHIP_ID 0x61

compCoef_t sh3001CompCoef;
// SH3001 register configuration values
typedef enum {
    SH3001_VAL_ACC_CONFIG_0_WORK_MODE = 0,                  // bit 7, normal mode
    SH3001_VAL_ACC_CONFIG_0_ADC_DITHER_ENABLE = BIT(6),     // bit 6, enable dither
    SH3001_VAL_ACC_CONFIG_0_DIGITAL_FILTER_ENABLE = BIT(0), // bit 0, digital filter enable
    SH3001_VAL_ACC_CONFIG_1_1KHZ_ODR = 0x00,                // bit [3:0], 1000Hz ODR
    SH3001_VAL_ACC_CONFIG_2_16G_RANGE = 0x02,               // bit [2:0], 16g range
    SH3001_VAL_ACC_CONFIG_3_LPF_CUTOFF = 0x01,              // bit [7:5], ODR*0.25
    SH3001_VAL_GYRO_CONFIG_1_8KHZ_ODR = 0x0A,               // bit [3:0], 8000Hz ODR
    SH3001_VAL_GYRO_CONFIG_2_DLPF_213 = 0x03,               // bit [3:2], DLPF 213Hz
    SH3001_VAL_GYRO_CONFIG_2_DLPF_313 = 0x02,               // bit [3:2], DLPF 313Hz
    SH3001_VAL_GYRO_CONFIG_2_DLPF_438 = 0x01,               // bit [3:2], DLPF 438Hz
    SH3001_VAL_GYRO_CONFIG_2_DLPF_1313 = 0x00,              // bit [3:2], DLPF 1313Hz
    SH3001_VAL_GYRO_RANGE_2000DPS = 0x06,                   // bit [2:0], 2000dps range
    SH3001_VAL_TEMP_CONFIG_0_500HZ_ODR = 0x00,              // bit [5:4], 500Hz ODR
    SH3001_VAL_INTERRUPT_EN_1_GYRO_DRDY_ENABLE = BIT(3),    // bit 3, enable gyro data ready interrupt
    SH3001_VAL_INTERRUPT_CONFIG_ACTIVE_HIGH = 0x00,         // bit 7, active high
    SH3001_VAL_INTERRUPT_CONFIG_ANY_READ = BIT(4),        // bit 4, clear interrupt flag after any register read
    // SH3001_VAL_SH3001_REG_INTERRUPT_CONT_LIM = 0x01,        // interrupt count limit
    SH3001_VAL_INT_PINMAP_1_GYRO_DRDY_PIN = 0x00,           // bit 4, gyro data ready interrupt pin on INT pin
    SH3001_VAL_CHIP_VERSION_MCC = 0x08,                     // version C
    SH3001_VAL_CHIP_VERSION_MCD = 0x10,                     // version D
    SH3001_VAL_CHIP_VERSION_MCF = 0x20,                     // version F

} sh3001ConfigValues_e;

typedef enum {
    SH3001_MASK_ACC_CONFIG_0 = 0xC1,
    SH3001_MASK_ACC_CONFIG_1 = 0x0F,
    SH3001_MASK_ACC_CONFIG_2 = 0x07,
    SH3001_MASK_ACC_CONFIG_3 = 0xE0,
    SH3001_MASK_GYRO_CONFIG_1 = 0x0F,
    SH3001_MASK_GYRO_CONFIG_2 = 0x0C,
    SH3001_MASK_TEMP_CONFIG_0 = 0x30,
    SH3001_MASK_INTERRUPT_EN_1 = 0x08,
    SH3001_MASK_INTERRUPT_CONFIG = 0x90,    // 1001 0000
    // SH3001_MASK_INTERRUPT_CONT_LIM = 0xFF,
    SH3001_MASK_INT_PINMAP_1 = 0x10,
    SH3001_MASK_GYRO_RANGE = 0x07,  // used on REG_GYRO_CONFIG_3 4 5
} sh3001ConfigMasks_e;

uint8_t sh3001Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;
    uint8_t i = 0;

    while ((chipID != SH3001_CHIP_ID) && (i++ < 3)) {
        busReadRegisterBuffer(dev, SH3001_REG_CHIP_ID, &chipID, 1);
        if ((i == 3) && (chipID != SH3001_CHIP_ID)) {
            return MPU_NONE;
        }
    }
    return SH3001_SPI;
}

static void sh3001WriteRegister(const extDevice_t *dev, sh3001Register_e registerID, uint8_t value, unsigned delayMs)
{
    uint8_t page = (registerID > 0x7F) ? 0x01 : 0x00;
    busWriteRegister(dev, SH3001_REG_PAGE, page);
    delayMicroseconds(1);
    busWriteRegister(dev, (registerID & 0x7F), value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void sh3001WriteRegisterBits(const extDevice_t *dev, sh3001Register_e registerID, sh3001ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t regValue = 0;
    if ( busReadRegisterBuffer(dev, registerID, &regValue, 1)){
        delay(2);
        regValue = (regValue & ~mask) | value;
        sh3001WriteRegister(dev, registerID, regValue, delayMs);
    }
}

static uint8_t sh3001ReadRegister(const extDevice_t *dev, sh3001Register_e registerID)
{
    uint8_t value = 0;
    uint8_t page = (registerID > 0x7F) ? 0x01 : 0x00;
    busWriteRegister(dev, SH3001_REG_PAGE, page);
    delay(1);
    busReadRegisterBuffer(dev, (registerID & 0x7F), &value, 1);
    return value;
}

static uint16_t sh3001ReadRegister16(const extDevice_t *dev, sh3001Register_e registerID)
{
    uint8_t value[2] = {0};
    uint8_t page = (registerID > 0x7F) ? 0x01 : 0x00;
    busWriteRegister(dev, SH3001_REG_PAGE, page);
    delay(1);
    busReadRegisterBuffer(dev, (registerID & 0x7F), value, 2);
    return (value[0] << 8) | value[1];
}

static void sh3001ResetMCX(const extDevice_t *dev, sh3001ConfigValues_e type)
{
    uint8_t regAddr[9] = {0xC0, 0xD3, 0xC2, 0xD3, 0xD5, 0xD4, 0xBB, 0xB9, 0xBA};
    uint8_t regDataA[9], regDataB[9] = {0};
    if (type == SH3001_VAL_CHIP_VERSION_MCC) {
        uint8_t regDataANew[9] = {0x38, 0xC6, 0x10, 0xC1, 0x02, 0x0C, 0x18, 0x18, 0x18};
        uint8_t regDataBNew[9] = {0x3D, 0xC2, 0x20, 0xC2, 0x00, 0x04, 0x00, 0x00, 0x00};
        memcpy(regDataA, regDataANew, 9);
        memcpy(regDataB, regDataBNew, 9);
    } else if (type == SH3001_VAL_CHIP_VERSION_MCD) {
        uint8_t regDataANew[9] = {0x38, 0xD6, 0x10, 0xD1, 0x02, 0x08, 0x18, 0x18, 0x18};
        uint8_t regDataBNew[9] = {0x3D, 0xD2, 0x20, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00};
        memcpy(regDataA, regDataANew, 9);
        memcpy(regDataB, regDataBNew, 9);
    } else if (type == SH3001_VAL_CHIP_VERSION_MCF) {
        uint8_t regDataANew[9] = {0x38, 0x16, 0x10, 0x11, 0x02, 0x08, 0x18, 0x18, 0x18};
        uint8_t regDataBNew[9] = {0x3E, 0x12, 0x20, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00};
        memcpy(regDataA, regDataANew, 9);
        memcpy(regDataB, regDataBNew, 9);
    }

    // Drive start
    sh3001WriteRegister(dev, regAddr[0], regDataA[0], 1);
    sh3001WriteRegister(dev, regAddr[1], regDataA[1], 1);
    sh3001WriteRegister(dev, regAddr[2], regDataA[2], 1);
    delay(300);
    sh3001WriteRegister(dev, regAddr[0], regDataB[0], 1);
    sh3001WriteRegister(dev, regAddr[1], regDataB[1], 1);
    sh3001WriteRegister(dev, regAddr[2], regDataB[2], 1);
    delay(100);

    // ADC reset
    sh3001WriteRegister(dev, regAddr[3], regDataA[3], 1);
    sh3001WriteRegister(dev, regAddr[4], regDataA[4], 1);
    delay(1);
    sh3001WriteRegister(dev, regAddr[3], regDataB[3], 1);
    delay(1);
    sh3001WriteRegister(dev, regAddr[4], regDataB[4], 1);
    delay(50);

    // CVA reset
    sh3001WriteRegister(dev, regAddr[5], regDataA[5], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[5], regDataB[5], 1);
    delay(1);
    sh3001WriteRegister(dev, regAddr[6], regDataA[6], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[7], regDataA[7], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[8], regDataA[8], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[6], regDataB[6], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[7], regDataB[7], 1);
    delay(10);
    sh3001WriteRegister(dev, regAddr[8], regDataB[8], 1);
    delay(10);
}

static void sh3001Reset(const extDevice_t *dev)
{
    uint8_t chipVersion = sh3001ReadRegister(dev, SH3001_REG_CHIP_VERSION);

    if (chipVersion == SH3001_VAL_CHIP_VERSION_MCC) {
        sh3001ResetMCX(dev, SH3001_VAL_CHIP_VERSION_MCC);
    } else if (chipVersion == SH3001_VAL_CHIP_VERSION_MCD) {
        sh3001ResetMCX(dev, SH3001_VAL_CHIP_VERSION_MCD);
    } else if (chipVersion == SH3001_VAL_CHIP_VERSION_MCF) {
        sh3001ResetMCX(dev, SH3001_VAL_CHIP_VERSION_MCF);
    } else {
        sh3001ResetMCX(dev, SH3001_VAL_CHIP_VERSION_MCD);
    }

}

static void sh3001CompCoefInit(const extDevice_t *dev, compCoef_t *compCoef)
{
    uint16_t coefData16 = 0;
    uint8_t data = 0;

    // Acc CAS
    coefData16 = sh3001ReadRegister16(dev, 0x81);
    compCoef->cYX = (int8_t)(coefData16 >> 8);
    compCoef->cZX = (int8_t)(coefData16 & 0x00FF);
    coefData16 = sh3001ReadRegister16(dev, 0x91);
    compCoef->cXY = (int8_t)(coefData16 >> 8);
    compCoef->cZY = (int8_t)(coefData16 & 0x00FF);
    coefData16 = sh3001ReadRegister16(dev, 0xA1);
    compCoef->cXZ = (int8_t)(coefData16 >> 8);
    compCoef->cYZ = (int8_t)(coefData16 & 0x00FF);

    // Gyro Zero
    data = sh3001ReadRegister(dev, 0x60);
    compCoef->jX = (int8_t)data;
    data = sh3001ReadRegister(dev, 0x68);
    compCoef->jY = (int8_t)data;
    data = sh3001ReadRegister(dev, 0x70);
    compCoef->jZ = (int8_t)data;

    data = sh3001ReadRegister(dev, SH3001_REG_GYRO_CONFIG_3);
    data = data & 0x07;
    compCoef->xMulti = ((data<2)||(data>=7)) ? 1 : (1<<(6-data));
    data = sh3001ReadRegister(dev, SH3001_REG_GYRO_CONFIG_4);
    data = data & 0x07;
    compCoef->yMulti = ((data<2)||(data>=7)) ? 1 : (1<<(6-data));
    data = sh3001ReadRegister(dev, SH3001_REG_GYRO_CONFIG_5);
    data = data & 0x07;
    compCoef->zMulti = ((data<2)||(data>=7)) ? 1 : (1<<(6-data));

    data = sh3001ReadRegister(dev, 0x2E);
    compCoef->paramP0 = data & 0x1F;
}

static uint8_t getShDlpfBandwidth()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return SH3001_VAL_GYRO_CONFIG_2_DLPF_213;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return SH3001_VAL_GYRO_CONFIG_2_DLPF_313;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return SH3001_VAL_GYRO_CONFIG_2_DLPF_438;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return SH3001_VAL_GYRO_CONFIG_2_DLPF_1313;
    }
    return 0;
}

static void sh3001Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset device
    sh3001Reset(dev);

    // Configure accelerometer
    sh3001WriteRegisterBits(dev, SH3001_REG_ACC_CONFIG_0, SH3001_MASK_ACC_CONFIG_0, (SH3001_VAL_ACC_CONFIG_0_WORK_MODE <<7 | SH3001_VAL_ACC_CONFIG_0_ADC_DITHER_ENABLE | SH3001_VAL_ACC_CONFIG_0_DIGITAL_FILTER_ENABLE), 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_ACC_CONFIG_1, SH3001_MASK_ACC_CONFIG_1, SH3001_VAL_ACC_CONFIG_1_1KHZ_ODR, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_ACC_CONFIG_2, SH3001_MASK_ACC_CONFIG_2, SH3001_VAL_ACC_CONFIG_2_16G_RANGE, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_ACC_CONFIG_3, SH3001_MASK_ACC_CONFIG_3, (SH3001_VAL_ACC_CONFIG_3_LPF_CUTOFF << 5), 1);

    // Configure gyroscope
    sh3001WriteRegisterBits(dev, SH3001_REG_GYRO_CONFIG_1, SH3001_MASK_GYRO_CONFIG_1, SH3001_VAL_GYRO_CONFIG_1_8KHZ_ODR, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_GYRO_CONFIG_2, SH3001_MASK_GYRO_CONFIG_2, (getShDlpfBandwidth() << 2), 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_GYRO_CONFIG_3, SH3001_MASK_GYRO_RANGE, SH3001_VAL_GYRO_RANGE_2000DPS, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_GYRO_CONFIG_4, SH3001_MASK_GYRO_RANGE, SH3001_VAL_GYRO_RANGE_2000DPS, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_GYRO_CONFIG_5, SH3001_MASK_GYRO_RANGE, SH3001_VAL_GYRO_RANGE_2000DPS, 1);

    // Configure temperature sensor
    sh3001WriteRegisterBits(dev, SH3001_REG_TEMP_CONFIG_0, SH3001_MASK_TEMP_CONFIG_0, SH3001_VAL_TEMP_CONFIG_0_500HZ_ODR, 1);

    // Configure interrupt
    sh3001WriteRegisterBits(dev, SH3001_REG_INTERRUPT_EN_1, SH3001_MASK_INTERRUPT_EN_1, SH3001_VAL_INTERRUPT_EN_1_GYRO_DRDY_ENABLE, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_INTERRUPT_CONFIG, SH3001_MASK_INTERRUPT_CONFIG, ( (SH3001_VAL_INTERRUPT_CONFIG_ACTIVE_HIGH << 7) | SH3001_VAL_INTERRUPT_CONFIG_ANY_READ ), 1);

    // sh3001WriteRegisterBits(dev, SH3001_REG_INTERRUPT_CONT_LIM, SH3001_MASK_INTERRUPT_CONT_LIM, SH3001_VAL_SH3001_REG_INTERRUPT_CONT_LIM, 1);

    sh3001WriteRegisterBits(dev, SH3001_REG_INT_PINMAP_1, SH3001_MASK_INT_PINMAP_1, SH3001_VAL_INT_PINMAP_1_GYRO_DRDY_PIN, 1);

    // Read compensation coefficients
    sh3001CompCoefInit(dev, &sh3001CompCoef);

    // Make sure page 0 is selected
    sh3001WriteRegister(dev, SH3001_REG_PAGE, 0x00, 1);
}

#ifdef USE_GYRO_EXTI

static void sh3001IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, sh3001ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void sh3001SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    sh3001Config(gyro);

#ifdef USE_GYRO_EXTI
    sh3001IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(SH3001_MAX_SPI_CLK_HZ));
}

static void sh3001SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}


bool sh3001SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != SH3001_SPI) {
        return false;
    }

    acc->initFn = sh3001SpiAccInit;
    acc->readFn = sh3001AccRead;

    return true;
}

bool sh3001SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != SH3001_SPI) {
        return false;
    }

    gyro->initFn = sh3001SpiGyroInit;
    gyro->readFn = sh3001GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif // USE_ACCGYRO_SH3001
