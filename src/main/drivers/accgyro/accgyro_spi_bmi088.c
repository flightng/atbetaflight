/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight is distributed in the hope that it
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

#ifdef USE_ACCGYRO_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define BMI088_EXTI_DETECT_THRESHOLD 0

// bool bmi088AccRead(gyroDev_t *gyro)
bool bmi088AccRead(accDev_t *acc)
{
    uint16_t *gyroData = (uint16_t *)acc->gyro->dev.rxBuf;
    // acc data reading
    acc->gyro->dev.txBuf[0] = BMI088_ACC_REG_RATE_X_LSB | 0x80;
    busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 8, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
    segments[0].u.buffers.rxData = acc->gyro->dev.rxBuf;

    spiSequence(&acc->gyro->dev, &segments[0]);

    // Wait for completion
    spiWait(&acc->gyro->dev);

    acc->ADCRaw[X] = gyroData[1];
    acc->ADCRaw[Y] = gyroData[2];
    acc->ADCRaw[Z] = gyroData[3];

    return true;
}

#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
busStatus_e bmi088GyroIntcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmi088GyroExtiHandler(extiCallbackRec_t *cb)
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

#else
void bmi088GyroExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif


bool bmi088GyroRead(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 8);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > BMI088_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[1] = BMI088_GYRO_RET_RATE_X_LSB | 0x80;
                gyro->segments[0].len = 8;
                gyro->segments[0].callback = bmi088GyroIntcallback;
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
        // gyro data reading
        gyro->dev.txBuf[1] = BMI088_GYRO_RET_RATE_X_LSB | 0x80;
        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

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

#endif // USE_ACCGYRO_BMI088
