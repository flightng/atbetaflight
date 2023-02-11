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
//use global compCoef defined in accgryo_spi_init.c 
extern compCoef_t sh3001CompCoef; 

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define SH3001_EXTI_DETECT_THRESHOLD 0 // since the no-latched clean time is too long

#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
busStatus_e sh3001Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void sh3001ExtiHandler(extiCallbackRec_t* cb)
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
void sh3001ExtiHandler(extiCallbackRec_t* cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

bool sh3001AccRead(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[1] = SH3001_REG_ACC_X_DATA_L | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 8, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &acc->gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &acc->gyro->dev.rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&acc->gyro->dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // This data was read from the gyro, which is the same SPI device as the acc
        int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;

        int32_t accTempX =  (int32_t)(  accData[1] + \
                                        accData[2] * ((float)sh3001CompCoef.cXY/1024.0f) + \
                                        accData[3] * ((float)sh3001CompCoef.cXZ/1024.0f) );
        int32_t accTempY =  (int32_t)(  accData[1] * ((float)sh3001CompCoef.cYX/1024.0f) + \
                                        accData[2] + \
                                        accData[3] * ((float)sh3001CompCoef.cYZ/1024.0f) );
        int32_t accTempZ =  (int32_t)(  accData[1] * ((float)sh3001CompCoef.cZX/1024.0f) + \
                                        accData[2] * ((float)sh3001CompCoef.cZY/1024.0f) + \
                                        accData[3] );
        if (accTempX > 32767) {
            accTempX = 32767;
        } else if (accTempX < -32768) {
            accTempX = -32768;
        }
        if (accTempY > 32767) {
            accTempY = 32767;
        } else if (accTempY < -32768) {
            accTempY = -32768;
        }
        if (accTempZ > 32767) {
            accTempZ = 32767;
        } else if (accTempZ < -32768) {
            accTempZ = -32768;
        }
        acc->ADCRaw[X] = accTempX;
        acc->ADCRaw[Y] = accTempY;
        acc->ADCRaw[Z] = accTempZ;
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static uint8_t sh3001InitCont = 0;
bool sh3001GyroRead(gyroDev_t *gyro)
{
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // 0 R 1L 1H 2L 2H 3L 3H AL AH BL BH CL CH TL TH PP
        //     1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
        // 0   1     2     3     4     5     6     7     8
        sh3001InitCont++;
        if (sh3001InitCont > 200 ){
#ifdef USE_GYRO_EXTI
            // Initialise the tx buffer to all 0x00
            memset(gyro->dev.txBuf, 0x00, 18);
            // We need some offset from the gyro interrupts to ensure sampling after the interrupt
            gyro->gyroDmaMaxDuration = 5;
            // Check that minimum number of interrupts have been detected
            if (gyro->detectedEXTI > SH3001_EXTI_DETECT_THRESHOLD) {
                if (spiUseDMA(&gyro->dev)) {
                    gyro->dev.callbackArg = (uint32_t)gyro;
                    gyro->dev.txBuf[1] = SH3001_REG_ACC_X_DATA_L | 0x80;
                    gyro->segments[0].len = 16;
                    gyro->segments[0].callback = sh3001Intcallback;
                    gyro->segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
                    gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                    gyro->segments[0].negateCS = true;
                    gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
                    // clear the data ready flag inside of the sensor
                    spiSequence(&gyro->dev, gyro->segments);
                    spiWait(&gyro->dev);
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
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[1] = SH3001_REG_GYRO_X_DATA_L | 0x80;
        // 0 R 1L 1H 2L 2H 3L 3H TL TH PP
        //   1 2  3  4  5  6  7  8  9  10   txBuf & rxBuf
        // 0   1     2     3     4     5
        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 10, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        uint8_t paramP = gyro->dev.rxBuf[10] & 0x1F;
        int32_t gyroTempX = gyroData[1] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jX * sh3001CompCoef.xMulti;
        int32_t gyroTempY = gyroData[2] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jY * sh3001CompCoef.yMulti;
        int32_t gyroTempZ = gyroData[3] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jZ * sh3001CompCoef.zMulti;
        if (gyroTempX > 32767) {
            gyroTempX = 32767;
        } else if (gyroTempX < -32768) {
            gyroTempX = -32768;
        }
        if (gyroTempY > 32767) {
            gyroTempY = 32767;
        } else if (gyroTempY < -32768) {
            gyroTempY = -32768;
        }
        if (gyroTempZ > 32767) {
            gyroTempZ = 32767;
        } else if (gyroTempZ < -32768) {
            gyroTempZ = -32768;
        }

        gyro->gyroADCRaw[X] = (int16_t)gyroTempX;
        gyro->gyroADCRaw[Y] = (int16_t)gyroTempY;
        gyro->gyroADCRaw[Z] = (int16_t)gyroTempZ;
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        uint8_t paramP = gyro->dev.rxBuf[15] & 0x1F;

        int32_t gyroTempX = gyroData[4] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jX * sh3001CompCoef.xMulti;
        int32_t gyroTempY = gyroData[5] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jY * sh3001CompCoef.yMulti;
        int32_t gyroTempZ = gyroData[6] - (paramP - sh3001CompCoef.paramP0) * sh3001CompCoef.jZ * sh3001CompCoef.zMulti;
        if (gyroTempX > 32767) {
            gyroTempX = 32767;
        } else if (gyroTempX < -32768) {
            gyroTempX = -32768;
        }
        if (gyroTempY > 32767) {
            gyroTempY = 32767;
        } else if (gyroTempY < -32768) {
            gyroTempY = -32768;
        }
        if (gyroTempZ > 32767) {
            gyroTempZ = 32767;
        } else if (gyroTempZ < -32768) {
            gyroTempZ = -32768;
        }
        gyro->gyroADCRaw[X] = (int16_t)gyroTempX;
        gyro->gyroADCRaw[Y] = (int16_t)gyroTempY;
        gyro->gyroADCRaw[Z] = (int16_t)gyroTempZ;
        break;
    }

    default:
        break;
    }

    return true;
}
#endif
