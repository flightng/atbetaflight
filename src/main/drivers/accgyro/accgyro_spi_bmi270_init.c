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

#define BMI270_CONFIG_SIZE 8192

// Declaration for the device config (microcode) that must be uploaded to the sensor
extern const uint8_t bmi270_config_file[BMI270_CONFIG_SIZE];

#define BMI270_CHIP_ID 0x24

#define BMI270_GYRO_CAS_MASK 0x7F
#define BMI270_GYRO_CAS_SIGN_BIT_MASK 0x40

int8_t bmi270CasFactor = 0;

// BMI270 register configuration values
typedef enum {
    BMI270_VAL_CMD_G_TRIGGER = 0x02,
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
    BMI270_VAL_PWR_CTRL_DISABLE_ALL = 0x00,             // disable all sensors
    BMI270_VAL_PWR_CTRL = 0x0E,                         // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CTRL_ACC_ENABLE = BIT(2),            // bit 2, enable acc
    BMI270_VAL_PWR_CTRL_ACC_DISABLE = 0x00,             // bit 2, disable gyro
    BMI270_VAL_PWR_CONF_ADV_POWER_SAVE_DISABLE = 0x00,  // disable ADV PS
    BMI270_VAL_PWR_CONF_ADV_POWER_SAVE_ENABLE = BIT(0), // enable ADV PS
    BMI270_VAL_PWR_CONF = 0x02,                         // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_PAGE_0 = 0x00,                           // select page 0
    BMI270_VAL_PAGE_1 = 0x01,                           // select page 1
    BMI270_VAL_ACC_CONF_ODR800 = 0x0B,                  // set acc sample rate to 800hz
    BMI270_VAL_ACC_CONF_ODR1600 = 0x0C,                 // set acc sample rate to 1600hz
    BMI270_VAL_ACC_CONF_BWP = 0x01,                     // set acc filter in osr2 mode
    BMI270_VAL_ACC_CONF_HP = 0x01,                      // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_8G = 0x02,                     // set acc to 8G full scale
    BMI270_VAL_ACC_RANGE_16G = 0x03,                    // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR3200 = 0x0D,                // set gyro sample rate to 3200hz
    BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,               // set gyro filter in OSR4 mode
    BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,               // set gyro filter in OSR2 mode
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,               // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,             // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01,            // set gyro in high performance filter mode

    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,               // set gyro to 2000dps full scale
                                                        // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                                        // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)

    BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,           // enable the data ready interrupt pin 1
    BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,             // enable the FIFO watermark interrupt pin 1
    BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,             // active high, push-pull, output enabled, input disabled 
    BMI270_VAL_IF_CONF_SET_INTERFACE = 0x00,            // spi 4-wire mode, disable OIS, disable AUX
    BMI270_VAL_FIFO_CONFIG_0 = 0x00,                    // don't stop when full, disable sensortime frame
    BMI270_VAL_FIFO_CONFIG_1 = 0x80,                    // only gyro data in FIFO, use headerless mode
    BMI270_VAL_FIFO_DOWNS = 0x00,                       // select unfiltered gyro data with no downsampling (6.4KHz samples)
    BMI270_VAL_FIFO_WTM_0 = 0x06,                       // set the FIFO watermark level to 1 gyro sample (6 bytes)
    BMI270_VAL_FIFO_WTM_1 = 0x00,                       // FIFO watermark MSB
    BMI270_VAL_GEN_SET_1 = 0x0200,                      // bit 9, enable self offset correction (IOC part 1)
    BMI270_VAL_OFFSET_6 = 0xC0,                         // Enable sensitivity error compensation and gyro offset compensation (IOC part2)
    BMI270_VAL_OFFSET_6_GYR_GAIN_EN_ENABLE = BIT(7),    // bit 7, enable gyro gain compensation
    BMI270_VAL_GYR_CRT_CONF_CRT_RUNNING = BIT(2),       // bit 7, enable gyro crt
    BMI270_VAL_FEATURES_1_G_TRIG_1_SELECT_CRT = 0x0100, // bit 8, CRT will be executed
    BMI270_VAL_FEATURES_1_G_TRIG_1_SELECT_GYR_BIST=0x0000, // bit 8, gyro built-in self-test will be executed
    BMI270_VAL_FEATURES_1_G_TRIG_1_BLOCK_UNLOCK = 0x0000,// bit 9, do not block further G_TRIGGER commands
    BMI270_VAL_FEATURES_1_G_TRIG_1_BLOCK_BLOCK = 0x0200, // bit 9, block further G_TRIGGER commands
} bmi270ConfigValues_e;

typedef enum {
    BMI270_MASK_FEATURES_1_GEN_SET_1 = 0x0200,
    BMI270_MASK_OFFSET_6 = 0xC0,
    BMI270_MASK_OFFSET_6_GYR_GAIN_ENABLE = 0x80,        // bit 7, enable gyro gain compensation
    BMI270_MASK_PWR_ADV_POWER_SAVE = 0x01,              // bit 0, enable advanced power save
    BMI270_MASK_PWR_CTRL_ACC_ENABLE = 0x04,             // bit 2, enable acc
    BMI270_MASK_GYR_CRT_CONF_CRT_RUNNING = 0x04,        // bit 2, gyro CRT running
    BMI270_MASK_G_TRIG_1_SELECT = 0x0100,               // bit 8, select feature that should be executed
    BMI270_MASK_G_TRIG_1_BLOCK = 0x0200,                // bit 9, block feature with next G_TRIGGER CMD
    BMI270_MASK_GYR_GAIN_STATUS_G_TRIG_STATUS = 0x38    // bit[5:3]
} bmi270ConfigMasks_e;

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
        (uint8_t)(data & 0x00FF),
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

static void bmi270UploadConfig(const extDevice_t *dev)
{
    bmi270RegisterWrite(dev, BMI270_REG_PWR_CONF, 0, 1);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 0, 1);

    // Transfer the config file
    spiWriteRegBuf(dev, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file));

    delay(10);
    bmi270RegisterWrite(dev, BMI270_REG_INIT_CTRL, 1, 1);
}

static void bmi270EnableIOC(const extDevice_t *dev)
{
    // Configure the feature register page to page 1
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_1, 1);

    // Enable self offset correction
    bmi270RegisterWriteBits16(dev, BMI270_REG_FEATURES_1_GEN_SET_1, BMI270_MASK_FEATURES_1_GEN_SET_1, BMI270_VAL_GEN_SET_1, 1);

    // Enable IOC
    bmi270RegisterWriteBits(dev, BMI270_REG_OFFSET_6, BMI270_MASK_OFFSET_6, 0x40, 1);

    // Configure the feature register page to page 0
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_0, 1);
}

static void bmi270PerformCRT(const extDevice_t *dev)
{
    // disable advance power save mode
    bmi270RegisterWriteBits(dev, BMI270_REG_PWR_CONF, BMI270_MASK_PWR_ADV_POWER_SAVE, BMI270_VAL_PWR_CONF_ADV_POWER_SAVE_DISABLE, 10);

    // enable gyro gain compensation, set OFFSET_6.gain_comp_en = 1
    bmi270RegisterWriteBits(dev, BMI270_REG_OFFSET_6, BMI270_MASK_OFFSET_6_GYR_GAIN_ENABLE, BMI270_VAL_OFFSET_6_GYR_GAIN_EN_ENABLE, 10);

    // enable acc, set pwr_ctrl.acc_en = 1
    bmi270RegisterWriteBits(dev, BMI270_REG_PWR_CTRL, BMI270_MASK_PWR_CTRL_ACC_ENABLE, BMI270_VAL_PWR_CTRL_ACC_ENABLE, 10);

    // set GYR_CRT_CONF.crt_running = 0b1
    bmi270RegisterWriteBits(dev, BMI270_REG_GYR_CRT_CONF, BMI270_MASK_GYR_CRT_CONF_CRT_RUNNING, BMI270_VAL_GYR_CRT_CONF_CRT_RUNNING, 10);

    // set page to 1
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_1, 10);

    // set G_TRIG_1.select =1
    bmi270RegisterWriteBits16(dev, BMI270_REG_FEATURES_1_G_TRIG_1, BMI270_MASK_G_TRIG_1_SELECT, BMI270_VAL_FEATURES_1_G_TRIG_1_SELECT_CRT, 10);

    // set G_TRIG_1.block = 0
    // bmi270RegisterWriteBits16(dev, BMI270_REG_FEATURES_1_G_TRIG_1, BMI270_MASK_G_TRIG_1_BLOCK, BMI270_VAL_FEATURES_1_G_TRIG_1_BLOCK_UNLOCK, 1);

    // send g_trigger command to CMD reg
    bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_G_TRIGGER, 5000);

    // read GYR_CRT_CONF.crt_running and check if it is 0
    uint8_t bmiCheck = bmi270RegisterRead(dev, BMI270_REG_GYR_CRT_CONF);
    // if not, wait for 2 seconds and check again max time for 5
    uint8_t checkCount = 0;

    while ((bmiCheck!=0x00) && (checkCount++ < 5)) {
        bmiCheck = bmi270RegisterRead(dev, BMI270_REG_GYR_CRT_CONF);
        delay(1000);
        if (bmiCheck == 0x00) {
            break;
        }
    }

    delay(100);
    // set page to 0
    bmi270RegisterWrite(dev, BMI270_REG_FEAT_PAGE, BMI270_VAL_PAGE_0, 1);

    // read GYR_GAIN_STATUS.g_trig_status
    uint16_t bmiStatus = bmi270RegisterRead16(dev, BMI270_REG_FEATURES_0_GYR_GAIN_STATUS);
    if (bmiStatus == 0x0000){
        delay(10);
    }
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

    bmi270PerformCRT(dev);

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

    // Enable IOC
    bmi270EnableIOC(dev);

    // Flush the FIFO
    if (fifoMode) {
        bmi270RegisterWrite(dev, BMI270_REG_CMD, BMI270_VAL_CMD_FIFOFLUSH, 1);
    }
    // Read the CAS.zx factor
    uint8_t casRaw = bmi270RegisterRead(dev, BMI270_REG_FEATURES_0_GYR_CAS);

    bmi270CasFactor = bmi270ProcessGyroCas(casRaw);
}

#ifdef USE_GYRO_EXTI
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
#endif

static void bmi270SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi270Config(gyro);

#ifdef USE_GYRO_EXTI
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
#endif // USE_ACCGYRO_BMI270
