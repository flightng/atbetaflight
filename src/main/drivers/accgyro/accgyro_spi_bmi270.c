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

#ifdef USE_ACCGYRO_BMI270

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"
// #include "build/debug.h"

// 10 MHz max SPI frequency
#define BMI270_MAX_SPI_CLK_HZ 10000000

#define BMI270_FIFO_FRAME_SIZE 6

#define BMI270_CONFIG_SIZE 8192

// Declaration for the device config (microcode) that must be uploaded to the sensor
extern const uint8_t bmi270_config_file[BMI270_CONFIG_SIZE];

#define BMI270_CHIP_ID 0x24

#define BMI270_GYRO_CAS_MASK 0x7F
#define BMI270_GYRO_CAS_SIGN_BIT_MASK 0x40

// BMI270 registers (not the complete list)
typedef enum {
    BMI270_REG_CHIP_ID = 0x00,
    BMI270_REG_ERR_REG = 0x02,
    BMI270_REG_STATUS = 0x03,
    BMI270_REG_ACC_DATA_X_LSB = 0x0C,
    BMI270_REG_GYR_DATA_X_LSB = 0x12,
    BMI270_REG_SENSORTIME_0 = 0x18,
    BMI270_REG_SENSORTIME_1 = 0x19,
    BMI270_REG_SENSORTIME_2 = 0x1A,
    BMI270_REG_EVENT = 0x1B,
    BMI270_REG_INT_STATUS_0 = 0x1C,
    BMI270_REG_INT_STATUS_1 = 0x1D,
    BMI270_REG_INTERNAL_STATUS = 0x21,
    BMI270_REG_TEMPERATURE_LSB = 0x22,
    BMI270_REG_TEMPERATURE_MSB = 0x23,
    BMI270_REG_FIFO_LENGTH_LSB = 0x24,
    BMI270_REG_FIFO_LENGTH_MSB = 0x25,
    BMI270_REG_FIFO_DATA = 0x26,
    BMI270_REG_FEAT_PAGE = 0x2F,
    BMI270_REG_FEATURES_0_GYR_CAS = 0x3C,
    BMI270_REG_FEATURES_1_GEN_SET_1 = 0x34,
    BMI270_REG_ACC_CONF = 0x40,
    BMI270_REG_ACC_RANGE = 0x41,
    BMI270_REG_GYRO_CONF = 0x42,
    BMI270_REG_GYRO_RANGE = 0x43,
    BMI270_REG_AUX_CONF = 0x44,
    BMI270_REG_FIFO_DOWNS = 0x45,
    BMI270_REG_FIFO_WTM_0 = 0x46,
    BMI270_REG_FIFO_WTM_1 = 0x47,
    BMI270_REG_FIFO_CONFIG_0 = 0x48,
    BMI270_REG_FIFO_CONFIG_1 = 0x49,
    BMI270_REG_SATURATION = 0x4A,
    BMI270_REG_INT1_IO_CTRL = 0x53,
    BMI270_REG_INT2_IO_CTRL = 0x54,
    BMI270_REG_INT_LATCH = 0x55,
    BMI270_REG_INT1_MAP_FEAT = 0x56,
    BMI270_REG_INT2_MAP_FEAT = 0x57,
    BMI270_REG_INT_MAP_DATA = 0x58,
    BMI270_REG_INIT_CTRL = 0x59,
    BMI270_REG_INIT_DATA = 0x5E,
    BMI270_REG_IF_CONF = 0x6B,
    BMI270_REG_ACC_SELF_TEST = 0x6D,
    BMI270_REG_GYR_SELF_TEST_AXES = 0x6E,
    BMI270_REG_OFFSET_6 = 0x77,
    BMI270_REG_PWR_CONF = 0x7C,
    BMI270_REG_PWR_CTRL = 0x7D,
    BMI270_REG_CMD = 0x7E,
} bmi270Register_e;

// BMI270 register configuration values
typedef enum {
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
    BMI270_VAL_PWR_CTRL_DISABLE_ALL = 0x00,  // disable all sensors
    BMI270_VAL_PWR_CTRL = 0x0E,              // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CONF = 0x02,              // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_PAGE_0 = 0x00,                // select page 0
    BMI270_VAL_PAGE_1 = 0x01,                // select page 1
    BMI270_VAL_ACC_CONF_ODR800 = 0x0B,       // set acc sample rate to 800hz
    BMI270_VAL_ACC_CONF_ODR1600 = 0x0C,      // set acc sample rate to 1600hz
    BMI270_VAL_ACC_CONF_BWP = 0x01,          // set acc filter in osr2 mode
    BMI270_VAL_ACC_CONF_HP = 0x01,           // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_8G = 0x02,          // set acc to 8G full scale
    BMI270_VAL_ACC_RANGE_16G = 0x03,         // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR3200 = 0x0D,     // set gyro sample rate to 3200hz
    BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,    // set gyro filter in OSR4 mode
    BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,    // set gyro filter in OSR2 mode
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,    // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,  // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01, // set gyro in high performance filter mode

    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,    // set gyro to 2000dps full scale
                                             // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                             // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)

    BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,// enable the data ready interrupt pin 1
    BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,  // enable the FIFO watermark interrupt pin 1
    BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,  // active high, push-pull, output enabled, input disabled 
    BMI270_VAL_IF_CONF_SET_INTERFACE = 0x00, // spi 4-wire mode, disable OIS, disable AUX
    BMI270_VAL_FIFO_CONFIG_0 = 0x00,         // don't stop when full, disable sensortime frame
    BMI270_VAL_FIFO_CONFIG_1 = 0x80,         // only gyro data in FIFO, use headerless mode
    BMI270_VAL_FIFO_DOWNS = 0x00,            // select unfiltered gyro data with no downsampling (6.4KHz samples)
    BMI270_VAL_FIFO_WTM_0 = 0x06,            // set the FIFO watermark level to 1 gyro sample (6 bytes)
    BMI270_VAL_FIFO_WTM_1 = 0x00,            // FIFO watermark MSB
    BMI270_VAL_GEN_SET_1 = 0x0200,           // bit 9, enable self offset correction (IOC part 1)
    BMI270_VAL_OFFSET_6 = 0xC0,              // Enable sensitivity error compensation and gyro offset compensation (IOC part2)
} bmi270ConfigValues_e;

typedef enum {
    BMI270_MASK_FEATURES_1_GEN_SET_1 = 0x0200,
    BMI270_MASK_OFFSET_6 = 0xC0,
} bmi270ConfigMasks_e;

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// BMI270 register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi270RegisterRead(const extDevice_t *dev, bmi270Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static uint16_t bmi270RegisterRead16(const extDevice_t *dev, bmi270Register_e registerId)
{
    uint8_t data[3] = { 0, 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 3)) {
        return (uint16_t)( (data[2]<<8) | data[1] );    // LSB first since address is auto-incremented
    } else {
        return 0;
    }
}

static void bmi270RegisterWrite(const extDevice_t *dev, bmi270Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void bmi270RegisterWriteBits(const extDevice_t *dev, bmi270Register_e registerID, bmi270ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue = bmi270RegisterRead(dev, registerID);
    delayMicroseconds(2);
    newValue = (newValue & ~mask) | value;
    bmi270RegisterWrite(dev, registerID, newValue, delayMs);
}

static void bmi270RegisterWrite16(const extDevice_t *dev, bmi270Register_e registerId, uint16_t data, unsigned delayMs)
{
    uint8_t buf[2] = {
        (uint8_t)(data & 0x00FF),   // LSB first since address is auto-incremented
        (uint8_t)(data >> 8)
    };
    spiWriteRegBuf(dev, registerId, buf, 2);
    if (delayMs) {
        delay(delayMs);
    }
}

static void bmi270RegisterWriteBits16(const extDevice_t *dev, bmi270Register_e registerID, bmi270ConfigMasks_e mask, uint16_t value, unsigned delayMs)
{
    uint16_t newValue = bmi270RegisterRead16(dev, registerID);
    delayMicroseconds(2);
    newValue = (newValue & ~mask) | value;
    bmi270RegisterWrite16(dev, registerID, newValue, delayMs);
}

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmi270EnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}

uint8_t bmi270Detect(const extDevice_t *dev)
{
    bmi270EnableSPI(dev);

    if (bmi270RegisterRead(dev, BMI270_REG_CHIP_ID) == BMI270_CHIP_ID) {
        return BMI_270_SPI;
    }

    return MPU_NONE;
}

static void bmi270UploadConfig(const extDevice_t *dev)
{
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CONF, 0, 1);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 0, 1);

    // Transfer the config file
    spiWriteRegBuf(dev, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file));

    delay(10);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 1, 1);
}

static uint8_t getBmiOsrMode()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMI270_VAL_GYRO_CONF_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMI270_VAL_GYRO_CONF_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMI270_VAL_GYRO_CONF_BWP_NORM;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMI270_VAL_GYRO_CONF_BWP_NORM;
    }
    return 0;
}

static void bmi270Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // If running in hardware_lpf experimental mode then switch to FIFO-based,
    // 6.4KHz sampling, unfiltered data vs. the default 3.2KHz with hardware filtering
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    const bool fifoMode = (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL);
#else
    const bool fifoMode = false;
#endif

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_SOFTRESET, 100);

    // Toggle the chip into SPI mode
    bmi270EnableSPI(dev);

    bmi270UploadConfig(dev);

    // Disable all sensors
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL_DISABLE_ALL, 1);

    // Configure the interface
    // disable OIS, disable AUX, spi 4-wire mode
    bmi270RegisterWrite(dev, BMI270_REG_IF_CONF, BMI270_VAL_IF_CONF_SET_INTERFACE, 1);

    // Configure the FIFO
    if (fifoMode) {
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_CONFIG_0, BMI270_VAL_FIFO_CONFIG_0, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_CONFIG_1, BMI270_VAL_FIFO_CONFIG_1, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_DOWNS, BMI270_VAL_FIFO_DOWNS, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_WTM_0, BMI270_VAL_FIFO_WTM_0, 1);
        bmi270RegisterWrite(dev, BMI270_REG_FIFO_WTM_1, BMI270_VAL_FIFO_WTM_1, 1);
    }

    // Configure the accelerometer
    bmi270RegisterWrite(dev, BMI270_REG_ACC_CONF, (BMI270_VAL_ACC_CONF_HP << 7) | (BMI270_VAL_ACC_CONF_BWP << 4) | BMI270_VAL_ACC_CONF_ODR800, 1);

    // Configure the accelerometer full-scale range
    bmi270RegisterWrite(dev, BMI270_REG_ACC_RANGE, BMI270_VAL_ACC_RANGE_16G, 1);

    // Configure the gyro
    bmi270RegisterWrite(dev, BMI270_REG_GYRO_CONF, (BMI270_VAL_GYRO_CONF_FILTER_PERF << 7) | (BMI270_VAL_GYRO_CONF_NOISE_PERF << 6) | (getBmiOsrMode() << 4) | BMI270_VAL_GYRO_CONF_ODR3200, 1);

    // Configure the gyro full-range scale
    bmi270RegisterWrite(dev, BMI270_REG_GYRO_RANGE, BMI270_VAL_GYRO_RANGE_2000DPS, 1);

    // Configure the gyro data ready interrupt
    if (fifoMode) {
        // Interrupt driven by FIFO watermark level
        bmi270RegisterWrite(dev, BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_FIFO_WM_INT1, 1);
    } else {
        // Interrupt driven by data ready
        bmi270RegisterWrite(dev, BMI270_REG_INT_MAP_DATA, BMI270_VAL_INT_MAP_DATA_DRDY_INT1, 1);
    }

    // Configure the behavior of the INT1 pin
    bmi270RegisterWrite(dev, BMI270_REG_INT1_IO_CTRL, BMI270_VAL_INT1_IO_CTRL_PINMODE, 1);

    // Configure the device for high performance mode
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CONF, BMI270_VAL_PWR_CONF, 1);

    // Enable the gyro, accelerometer and temperature sensor - disable aux interface
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CTRL, BMI270_VAL_PWR_CTRL, 1);

    // Configure the feature register page to page 1
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_1, 1);

    // Enable self offset correction
    bmi270RegisterWriteBits16(dev, BMI270_REG_FEATURES_1_GEN_SET_1, BMI270_MASK_FEATURES_1_GEN_SET_1, BMI270_VAL_GEN_SET_1, 1);

    // Enable IOC
    bmi270RegisterWriteBits(dev, BMI270_REG_OFFSET_6, BMI270_MASK_OFFSET_6, 0x40, 1);

    // Configure the feature register page to page 0
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_0, 1);

    // Flush the FIFO
    if (fifoMode) {
        bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_FIFOFLUSH, 1);
    }
}

extiCallbackRec_t bmi270IntCallbackRec;

/*
 * Gyro interrupt service routine
 */
#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed, next is CAS read
busStatus_e bmi270Intcallback(uint32_t arg)
{
    UNUSED(arg);
    return BUS_READY;
}

// CAS read has just completed
busStatus_e bmi270IntCAScallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmi270ExtiHandler(extiCallbackRec_t *cb)
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

static void bmi270IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi270ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#else
void bmi270ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

static bool bmi270AccRead(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = BMI270_REG_ACC_DATA_X_LSB | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
        segments[0].u.buffers.rxData = acc->gyro->dev.rxBuf;

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
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static int8_t bmi270ProcessGyroCas(uint8_t raw)
{
    int8_t result = 0;
    raw = raw & BMI270_GYRO_CAS_MASK;
    if ((raw & BMI270_GYRO_CAS_SIGN_BIT_MASK) == 0) {
        result = (int8_t)(raw);
    } else {
        result = (int8_t)(raw - 128);
    }
    return result;
}

static uint8_t bmi270ReadGyroCas(gyroDev_t *gyro)
{
    // GYRO_CAS rading
    uint8_t casTxBuf[4] = {BMI270_REG_FEATURES_0_GYR_CAS | 0x80, 0, 0, 0};
    uint8_t casRxBuf[4] = {0, 0, 0, 0};
    busSegment_t segmentsCas[] = {
            {.u.buffers = {casTxBuf, casRxBuf}, 4, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(&gyro->dev, &segmentsCas[0]);
    // Wait for completion
    spiWait(&gyro->dev);

    uint8_t gyroCasRaw = segmentsCas[0].u.buffers.rxData[2];
    return bmi270ProcessGyroCas(gyroCasRaw);
}

static bool bmi270GyroReadRegister(gyroDev_t *gyro)
{
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 18);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                // first segments is for acc+gyro data
                gyro->dev.txBuf[0] = BMI270_REG_ACC_DATA_X_LSB | 0x80;
                gyro->segments[0].len = 14;
                gyro->segments[0].callback = bmi270Intcallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = gyro->dev.rxBuf;
                gyro->segments[0].negateCS = true;
                // second segment is for gyro CAS factor
                gyro->dev.txBuf[14] = BMI270_REG_FEATURES_0_GYR_CAS | 0x80;
                gyro->segments[1].len = 4;
                gyro->segments[1].callback = bmi270IntCAScallback;
                gyro->segments[1].u.buffers.txData = &gyro->dev.txBuf[14];
                gyro->segments[1].u.buffers.rxData = &gyro->dev.rxBuf[14];
                gyro->segments[1].negateCS = true;
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
        gyro->dev.txBuf[0] = BMI270_REG_GYR_DATA_X_LSB | 0x80;
        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 8, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = gyro->dev.rxBuf;

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        int8_t cas = bmi270ReadGyroCas(gyro);

        // only x axis need overflow check
        int32_t tempx = gyroData[1] - (int16_t)(cas * (int16_t)(gyroData[3]) / 512);
        if (tempx > 32767) {
            gyro->gyroADCRaw[X] = 32767;
        } else if (tempx < -32768) {
            gyro->gyroADCRaw[X] = -32768;
        } else {
            gyro->gyroADCRaw[X] = tempx;
        }
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];

        // DEBUG_SET(DEBUG_BMI270_GYRO, 0, lrintf(gyroData[1]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 1, lrintf(gyro->gyroADCRaw[X]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 2, lrintf(gyroData[3]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 3, lrintf((int16_t)(cas * (int16_t)(gyroData[3]) / 512)));

        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        int8_t cas = bmi270ProcessGyroCas((uint8_t)(gyroData[8]));
        // only x axis need overflow check
        int32_t tempx = gyroData[4] - (int16_t)(cas * (int16_t)(gyroData[6]) / 512);
        if (tempx > 32767) {
            gyro->gyroADCRaw[X] = 32767;
        } else if (tempx < -32768) {
            gyro->gyroADCRaw[X] = -32768;
        } else {
            gyro->gyroADCRaw[X] = tempx;
        }
        gyro->gyroADCRaw[Y] = gyroData[5];
        gyro->gyroADCRaw[Z] = gyroData[6];

        // DEBUG_SET(DEBUG_BMI270_GYRO, 0, lrintf(gyroData[4]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 1, lrintf(gyroData[6]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 2, lrintf(gyro->gyroADCRaw[X]));
        // DEBUG_SET(DEBUG_BMI270_GYRO, 3, lrintf((int16_t)cas));
        break;
    }

    default:
        break;
    }

    return true;
}

#ifdef USE_GYRO_DLPF_EXPERIMENTAL
static bool bmi270GyroReadFifo(gyroDev_t *gyro)
{
    enum {
        IDX_REG = 0,
        IDX_SKIP,
        IDX_FIFO_LENGTH_L,
        IDX_FIFO_LENGTH_H,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    bool dataRead = false;
    STATIC_DMA_DATA_AUTO uint8_t bmi270_tx_buf[BUFFER_SIZE] = {BMI270_REG_FIFO_LENGTH_LSB | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t bmi270_rx_buf[BUFFER_SIZE];

    // Burst read the FIFO length followed by the next 6 bytes containing the gyro axis data for
    // the first sample in the queue. It's possible for the FIFO to be empty so we need to check the
    // length before using the sample.
    spiReadWriteBuf(&gyro->dev, (uint8_t *)bmi270_tx_buf, bmi270_rx_buf, BUFFER_SIZE);   // receive response

    int fifoLength = (uint16_t)((bmi270_rx_buf[IDX_FIFO_LENGTH_H] << 8) | bmi270_rx_buf[IDX_FIFO_LENGTH_L]);

    if (fifoLength >= BMI270_FIFO_FRAME_SIZE) {

        const int16_t gyroX = (int16_t)((bmi270_rx_buf[IDX_GYRO_XOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_XOUT_L]);
        const int16_t gyroY = (int16_t)((bmi270_rx_buf[IDX_GYRO_YOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_YOUT_L]);
        const int16_t gyroZ = (int16_t)((bmi270_rx_buf[IDX_GYRO_ZOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_ZOUT_L]);

        // If the FIFO data is invalid then the returned values will be 0x8000 (-32768) (pg. 43 of datasheet).
        // This shouldn't happen since we're only using the data if the FIFO length indicates
        // that data is available, but this safeguard is needed to prevent bad things in
        // case it does happen.
        if ((gyroX != INT16_MIN) || (gyroY != INT16_MIN) || (gyroZ != INT16_MIN)) {
            gyro->gyroADCRaw[X] = gyroX;
            gyro->gyroADCRaw[Y] = gyroY;
            gyro->gyroADCRaw[Z] = gyroZ;
            dataRead = true;
        }
        fifoLength -= BMI270_FIFO_FRAME_SIZE;
    }

    // If there are additional samples in the FIFO then we don't use those for now and simply
    // flush the FIFO. Under normal circumstances we only expect one sample in the FIFO since
    // the gyro loop is running at the native sample rate of 6.4KHz.
    // However the way the FIFO works in the sensor is that if a frame is partially read then
    // it remains in the queue instead of bein removed. So if we ever got into a state where there
    // was a partial frame or other unexpected data in the FIFO is may never get cleared and we
    // would end up in a lock state of always re-reading the same partial or invalid sample.
    if (fifoLength > 0) {
        // Partial or additional frames left - flush the FIFO
        bmi270RegisterWrite(&gyro->dev, BMI270_REG_CMD, BMI270_VAL_CMD_FIFOFLUSH, 0);
    }

    return dataRead;
}
#endif

static bool bmi270GyroRead(gyroDev_t *gyro)
{
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
        // running in 6.4KHz FIFO mode
        return bmi270GyroReadFifo(gyro);
    } else
#endif
    {
        // running in 3.2KHz register mode
        return bmi270GyroReadRegister(gyro);
    }
}

static void bmi270SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi270Config(gyro);

#if defined(USE_GYRO_EXTI)
    bmi270IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMI270_MAX_SPI_CLK_HZ));
}

static void bmi270SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool bmi270SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_270_SPI) {
        return false;
    }

    acc->initFn = bmi270SpiAccInit;
    acc->readFn = bmi270AccRead;

    return true;
}


bool bmi270SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_270_SPI) {
        return false;
    }

    gyro->initFn = bmi270SpiGyroInit;
    gyro->readFn = bmi270GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

// Used to query the status register to determine what event caused the EXTI to fire.
// When in 3.2KHz mode the interrupt is mapped to the data ready state. However the data ready
// trigger will fire for both gyro and accelerometer. So it's necessary to check this register
// to determine which event caused the interrupt.
// When in 6.4KHz mode the interrupt is configured to be the FIFO watermark size of 6 bytes.
// Since in this mode we only put gyro data in the FIFO it's sufficient to check for the FIFO
// watermark reason as an idication of gyro data ready.
uint8_t bmi270InterruptStatus(gyroDev_t *gyro)
{
    return bmi270RegisterRead(&gyro->dev, BMI270_REG_INT_STATUS_1);
}
#endif // USE_ACCGYRO_BMI270
