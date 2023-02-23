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

#ifdef USE_ACCGYRO_QMI8658

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_qmi8658.h"
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
#define QMI8658_MAX_SPI_CLK_HZ 15000000

#define QMI8658_CHIP_ID 0x05

// QMI8658 register configuration values
typedef enum {
    QMI8658_VAL_CTRL1 = 0x76,                           // 4 wire, address auto increment, int2 output, fifo on int1
    QMI8658_VAL_CTRL2_ENABLE_ACC_SELF_TEST = BIT(7),    // bit 7, enable acc self test
    QMI8658_VAL_CTRL2_ACC_FS_16G = 0x03,                // bit[6:4], acc full scale 16g
    QMI8658_VAL_CTRL2_ACC_ODR_896 = 0x03,               // bit[3:0], 896.8Hz ODR
    QMI8658_VAL_CTRL3_ENABLE_GYRO_SELF_TEST = BIT(7),   // bit 7, enable gyro self test
    QMI8658_VAL_CTRL3_GYRO_FS_2048DPS = 0x07,           // bit[6:4], gyro full scale 2048dps
    QMI8658_VAL_CTRL3_GYRO_ODR_7174 = 0x00,             // bit[3:0], 7174.4Hz ODR
    QMI8658_VAL_CTRL5_GLPF_ODR_266 = 0x00,              // bit[6:5], 2.66% ODR, 190.5Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_363 = 0x01,              // bit[6:5], 3.63% ODR, 260.4Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_539 = 0x02,              // bit[6:5], 5.39% ODR, 386.7Hz
    QMI8658_VAL_CTRL5_GLPF_ODR_1337 = 0x03,             // bit[6:5], 13.37% ODR, 959.2Hz
    QMI8658_VAL_CTRL5_GLPF_EN = BIT(4),                 // bit 4, enable gyro low pass filter
    QMI8658_VAL_CTRL5_ALPF_ODR_1337 = 0x03,             // bit[2:1], 13.37% ODR, 119.9Hz
    QMI8658_VAL_CTRL5_ALPF_EN = BIT(0),                 // bit 0, enable acc low pass filter
    QMI8658_VAL_CTRL7_G_EN = BIT(1),                    // bit 1, enable gyro
    QMI8658_VAL_CTRL7_A_EN = BIT(0),                    // bit 0, enable acc
    QMI8658_VAL_CTRL9_CMD_NOP = 0x00,                   // no operation
    QMI8658_VAL_CTRL9_CMD_ON_DEMAND_CALI = 0xA2,        // on demand cali
    QMI8658_VAL_RESET = 0x0B,                           // reset the device immediately
} qmi8658ConfigReg_e;

typedef enum {
    QMI8658_MASK_CTRL5 = 0x77,
    QMI8658_MASK_CTRL7 = 0x03,
} qmi8658ConfigMasks_e;

uint8_t qmi8658Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;
    uint8_t i = 0;

    while ((chipID != QMI8658_CHIP_ID) && (i++ < 5)) {
        busReadRegisterBuffer(dev, QMI8658_REG_WHO_AM_I, &chipID, 1);
        if ((i == 5) && (chipID != QMI8658_CHIP_ID)) {
            return MPU_NONE;
        }
    }
    return QMI_8658_SPI;
}

static void qmi8658WriteRegister(const extDevice_t *dev, qmi8658Register_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void qmi8658WriteRegisterBits(const extDevice_t *dev, qmi8658Register_e registerID, qmi8658ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        qmi8658WriteRegister(dev, registerID, newValue, delayMs);
    }
}

static uint8_t getQmiDlpfBandwidth()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return QMI8658_VAL_CTRL5_GLPF_ODR_266;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return QMI8658_VAL_CTRL5_GLPF_ODR_363;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return QMI8658_VAL_CTRL5_GLPF_ODR_539;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return QMI8658_VAL_CTRL5_GLPF_ODR_1337;
    }
    return 0;
}

static void qmi8658Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // reset the device
    qmi8658WriteRegister(dev, QMI8658_REG_RESET, QMI8658_VAL_RESET, 20);

    // On demand cali
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL9, QMI8658_VAL_CTRL9_CMD_ON_DEMAND_CALI, 2200);
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL9, QMI8658_VAL_CTRL9_CMD_NOP, 100);

    // Configure the CTRL1
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL1, QMI8658_VAL_CTRL1, 1);

    // Disable all sensors
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL7, 0x00, 1);

    // Configure the CTRL2 - ACC configuration
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL2, ((QMI8658_VAL_CTRL2_ACC_FS_16G << 4) | QMI8658_VAL_CTRL2_ACC_ODR_896), 1);

    // Configure the CTRL3 - GYRO configuration
    qmi8658WriteRegister(dev, QMI8658_REG_CTRL3, ((QMI8658_VAL_CTRL3_GYRO_FS_2048DPS << 4) | QMI8658_VAL_CTRL3_GYRO_ODR_7174), 1);

    // Configure the CTRL5 - GYRO LPF and ACC LPF configuration
    qmi8658WriteRegisterBits(dev, QMI8658_REG_CTRL5, QMI8658_MASK_CTRL5, ((getQmiDlpfBandwidth()<<5) | QMI8658_VAL_CTRL5_GLPF_EN | (QMI8658_VAL_CTRL5_ALPF_ODR_1337<<1) | QMI8658_VAL_CTRL5_ALPF_EN), 1);

    // Enable acc gyro
    qmi8658WriteRegisterBits(dev, QMI8658_REG_CTRL7, QMI8658_MASK_CTRL7, (QMI8658_VAL_CTRL7_G_EN | QMI8658_VAL_CTRL7_A_EN), 100);
}

#ifdef USE_GYRO_EXTI
static void qmi8658IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, qmi8658ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void qmi8658SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    qmi8658Config(gyro);

#ifdef USE_GYRO_EXTI
    qmi8658IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(QMI8658_MAX_SPI_CLK_HZ));
}

static void qmi8658SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool qmi8658SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != QMI_8658_SPI) {
        return false;
    }

    acc->initFn = qmi8658SpiAccInit;
    acc->readFn = qmi8658AccRead;

    return true;
}

bool qmi8658SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != QMI_8658_SPI) {
        return false;
    }

    gyro->initFn = qmi8658SpiGyroInit;
    gyro->readFn = qmi8658GyroRead;
    gyro->scale = GYRO_SCALE_2048DPS; // 16.4 LSB/dps

    return true;
}

#endif // USE_ACCGYRO_QMI8658
