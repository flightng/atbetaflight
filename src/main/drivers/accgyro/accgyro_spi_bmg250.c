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

#ifdef USE_GYRO_BMG250

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmg250.h"
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
#define BMG250_MAX_SPI_CLK_HZ 10000000

#define BMG250_CHIP_ID 0xD5

// BMG250 registers (not the complete list)
typedef enum{
    BMG250_REG_CHIPID = 0x00,
    BMG250_REG_PMU_STATUS = 0x03,
    BMG250_REG_GYR_DATA_X_LSB = 0x12,
    BMG250_REG_SENSORTIME = 0x18,
    BMG250_REG_STATUS = 0x1B,
    BMG250_REG_INT_STATUS_1 = 0x1D,
    BMG250_REG_TEMPERATURE = 0x20,
    BMG250_REG_GYR_CONF = 0x42,
    BMG250_REG_GYR_RANGE = 0x43,
    BMG250_REG_INT_EN_1 = 0x51,
    BMG250_REG_INT_OUT_CTRL = 0x53,
    BMG250_REG_INT_IN_CTRL = 0x54,
    BMG250_REG_INT_MAP_1 = 0x56,
    BMG250_REG_CONF = 0x6A,
    BMG250_REG_SELF_TEST = 0x6D,
    BMG250_REG_NV_CONF = 0x70,
    BMG250_REG_OFFSET_3 = 0x74,
    BMG250_REG_CMD = 0x7E,
} bmg250Register_e;

// BMG250 register configuration values
typedef enum {
    BMG250_VAL_CMD_GYR_FSUP = 0x17,             // bit(2:1) Sets the PMU mode for the Gyroscope to fast start-up.
    BMG250_VAL_CMD_SOFTRESET = 0xB6,            // Soft reset
    BMG250_VAL_GYRO_CONF_BWP_OSR4 = 0x00,       // bit(5:4) Sets the oversampling rate of the Gyroscope to 4x.
    BMG250_VAL_GYRO_CONF_BWP_OSR2 = 0x01,       // bit(5:4) Sets the oversampling rate of the Gyroscope to 2x.
    BMG250_VAL_GYRO_CONF_BWP_NORM = 0x02,       // bit(5:4) Sets the oversampling rate of the Gyroscope to 1x.
    BMG250_VAl_GYRO_CONF_ODR6400 = 0x0E,        // bit(3:0) Sets the output data rate of the Gyroscope to 6400Hz.
    BMG250_VAL_GYRO_RANGE_2000DPS = 0x00,       // bit(2:0) Sets the range of the Gyroscope to 2000dps.
    BMG250_VAL_INT_EN_1_DRDY = 0x10,            // bit(4) DRDY interrupt enable
    BMG250_VAL_INT_OUT_CTRL_INT1_CONFIG = 0x0A, // active high, push-pull, output enabled, input disabled
    BMG250_VAL_INT_MAP_1_INT1_CONFIG = 0x80,    // bit(7) INT1_DRDY interrupt mapped to INT1 pin
} bmg250ConfigValues_e;

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// BMB250 register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.

static uint8_t bmg250RegisterRead(const extDevice_t *dev, bmg250Register_e registerID)
{
    uint8_t data[2] = { 0, 0 };
    if (spiReadRegMskBufRB(dev, registerID, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmg250RegisterWrite(const extDevice_t *dev, bmg250Register_e registerID, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmg250EnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}

uint8_t bmg250Detect(const extDevice_t *dev)
{
    bmg250EnableSPI(dev);

    if (bmg250RegisterRead(dev, BMG250_REG_CHIPID) == BMG250_CHIP_ID) {
        return BMG_250_SPI;
    }

    return MPU_NONE;
}

static uint8_t getBmgOsrMode()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMG250_VAL_GYRO_CONF_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMG250_VAL_GYRO_CONF_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMG250_VAL_GYRO_CONF_BWP_NORM;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMG250_VAL_GYRO_CONF_BWP_NORM;
    }
    return 0;
}

static int32_t bmg250Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Set fast start-up power mode for gyro
    // Delay 100ms before continuing configuration
    bmg250RegisterWrite(dev, BMG250_REG_CMD, BMG250_VAL_CMD_GYR_FSUP, 100);

    // Toggle the chip into SPI mode
    bmg250EnableSPI(dev);

    // Verify that normal power mode was entered
    uint8_t pmu_status = spiReadRegMsk(dev, BMG250_REG_PMU_STATUS);
    if ((pmu_status & 0x0C) != 0x0C) {
        return -3;
    }

    // Configure the gyro osr mode and odr
    bmg250RegisterWrite(dev, BMG250_REG_GYR_CONF, (getBmgOsrMode() << 4) | BMG250_VAl_GYRO_CONF_ODR6400, 1);

    // Configure the gyro range
    bmg250RegisterWrite(dev, BMG250_REG_GYR_RANGE, BMG250_VAL_GYRO_RANGE_2000DPS, 1);

    // Enable offset compensation
    uint8_t val = spiReadRegMsk(dev, BMG250_REG_OFFSET_3);
    bmg250RegisterWrite(dev, BMG250_REG_OFFSET_3, val | 0xC0, 1);

    // Enable DRDY interrupt
    bmg250RegisterWrite(dev, BMG250_REG_INT_EN_1, BMG250_VAL_INT_EN_1_DRDY, 1);

    // Enable INT1 pin
    bmg250RegisterWrite(dev, BMG250_REG_INT_OUT_CTRL, BMG250_VAL_INT_OUT_CTRL_INT1_CONFIG, 1);

    // Map DRDY interrupt to INT1 pin
    bmg250RegisterWrite(dev, BMG250_REG_INT_MAP_1, BMG250_VAL_INT_MAP_1_INT1_CONFIG, 1);

    return 0;
}

extiCallbackRec_t bmg250IntCallbackRec;

#ifdef USE_GYRO_EXTI
// CAlled in ISR context
// Gyro read has just completed
busStatus_e bmg250Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmg250ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }

    gyro->detectedEXTI++;

}

static void bmg250IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmg250ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#else
void bmg250ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

static bool bmg250GyroRead(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 14);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[1] = BMG250_REG_GYR_DATA_X_LSB | 0x80;
                gyro->segments[0].len = 14;
                gyro->segments[0].callback = bmg250Intcallback;
                gyro->segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else
#endif
        {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[1] = BMG250_REG_GYR_DATA_X_LSB | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        // Fall through
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];

        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    default:
        break;
    }

    return true;

}

void bmg250SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    bmg250Config(gyro);
#if defined(USE_GYRO_EXTI)
    bmg250IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMG250_MAX_SPI_CLK_HZ));
}

bool bmg250SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMG_250_SPI) {
        return false;
    }

    gyro->initFn = bmg250SpiGyroInit;
    gyro->readFn = bmg250GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // USE_GYRO_BMG250
