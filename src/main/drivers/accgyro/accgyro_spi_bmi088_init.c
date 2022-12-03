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

// 10 MHz max SPI frequency
#define BMI088_MAX_SPI_CLK_HZ 10000000
#define BMI088_ACC_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID 0x0F

// BMI088 register configuration values
typedef enum {
    // Acc Register values
    BMI088_ACC_VAL_ACC_CONF = 0x8A,                 // OSR4 mode, 400Hz ODR
    BMI088_ACC_VAL_ACC_RANGE_24G = 0x03,            // set accel to 24g full scale
    BMI088_ACC_VAL_INT1_IO_CONF_PINMODE = 0x12,     // int1, push-pull, active high
    BMI088_ACC_VAL_INT_MAP_DATA_INT1 = 0x04,        // map data ready to int1
    BMI088_ACC_VAL_SELF_TEST_OFF = 0x00,            // self-test off
    BMI088_ACC_VAL_SELF_TEST_POSITIVE = 0x0D,       // positive self-test
    BMI088_ACC_VAL_SELF_TEST_NEGATIVE = 0x09,       // negative self-test
    BMI088_ACC_VAL_ACC_PWR_CONF_ACTIVE = 0x00,      // set accel to active mode
    BMI088_ACC_VAL_ACC_PWR_CTRL_ENABLE = 0x04,      // enable accel
    BMI088_ACC_VAL_SOFTRESET = 0xB6,

    // Gyro Register values
    BMI088_GYRO_VAL_RANGE_2000DPS = 0x00,           // set gyro to 2000 dps full scale
    BMI088_GYRO_VAL_BANDWIDTH_532Hz = 0x00,         // 2000Hz ODR, 532Hz filter
    BMI088_GYRO_VAL_BANDWIDTH_230Hz = 0x01,         // 2000Hz ODR, 230Hz filter
    BMI088_GYRO_VAL_SOFTRESET = 0xB6,
    BMI088_GYRO_VAL_INT_CTRL_NEW_DATA = 0x80,       // enable new data interrupt
    BMI088_GYRO_VAL_INT3_IO_CONF_PINMODE = 0x01,    // int3, push-pull, active high
    BMI088_GYRO_VAL_INT3_INT4_IO_MAP_INT3 = 0x01,   // map data ready to int3
    BMI088_GYRO_VAL_SELF_TEST_START = 0x00,         // start the built-in self-test
} bmi088ConfigValue_e;

/*  BMI088 common func  */
static void bmi088EnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}
/*  BMI088 common func end  */

/*  BMI088 init gyro part*/
// BMI088 gyro register reads are 8bits result.
static uint8_t bmi088GyroRegisterRead(const extDevice_t *dev, bmi088Register_e registerId)
{
    uint8_t data[1] = {0};

    if (spiReadRegMskBufRB(dev, registerId, data, 1)) {
        return data[0];
    } else {
        return 0;
    }
}

static void bmi088RegisterWrite(const extDevice_t *dev, bmi088Register_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t bmi088GyroDetect(const extDevice_t *dev)
{
    bmi088EnableSPI(dev);

    if (bmi088GyroRegisterRead(dev, BMI088_GYRO_REG_CHIP_ID) == BMI088_GYRO_CHIP_ID) {
        return BMI_088_GYRO_SPI;
    }
    return MPU_NONE;
}

static uint8_t getBmiGyroBandwidth()
{
    switch (gyroConfig()->gyro_hardware_lpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return BMI088_GYRO_VAL_BANDWIDTH_230Hz;
    case GYRO_HARDWARE_LPF_OPTION_1:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    case GYRO_HARDWARE_LPF_OPTION_2:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    }
    return 0;
}

static void bmi088GyroConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_SOFTRESET, BMI088_GYRO_VAL_SOFTRESET, 100);

    // Configure the gyro range
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_RANGE, BMI088_GYRO_VAL_RANGE_2000DPS, 1);

    // Configure the gyro bandwidth
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_BANDWIDTH, getBmiGyroBandwidth(), 1);

    // Configure the INT CTRL mode
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT_CTRL, BMI088_GYRO_VAL_INT_CTRL_NEW_DATA, 1);

    // Configure the INT3 pinmode
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_CONF, BMI088_GYRO_VAL_INT3_IO_CONF_PINMODE, 1);

    // Configure the INT3 interrupt map
    bmi088RegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_MAP, BMI088_GYRO_VAL_INT3_INT4_IO_MAP_INT3, 1);
}

#ifdef USE_GYRO_EXTI
static void bmi088GyroIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi088GyroExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi088GyroConfig(gyro);

#if defined(USE_GYRO_EXTI)
    bmi088GyroIntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_GYRO_SPI) {
        return false;
    }

    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
/*  BMI088 init gyro part end  */

/*  BMI088 acc init part  */
// BMI088 Acc register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi088AccRegisterRead(const extDevice_t *dev, bmi088Register_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmi088AccConfig(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi088RegisterWrite(dev, BMI088_ACC_REG_SOFTRESET, BMI088_ACC_VAL_SOFTRESET, 100);

    // Toggle the chip into SPI mode
    bmi088EnableSPI(dev);

    // enable accelerometer
    bmi088RegisterWrite(dev, BMI088_ACC_REG_ACC_PWR_CTRL, BMI088_ACC_VAL_ACC_PWR_CTRL_ENABLE, 450);

    // Configure the accelerometer
    bmi088RegisterWrite(dev, BMI088_ACC_REG_ACC_CONF, BMI088_ACC_VAL_ACC_CONF, 1);

    // Configure the accelerometer range
    bmi088RegisterWrite(dev, BMI088_ACC_REG_ACC_RANGE, BMI088_ACC_VAL_ACC_RANGE_24G, 1);
}

static void bmi088SpiAccInit(accDev_t *acc)
{

    bmi088AccConfig(acc);
    // extDevice_t *dev = &acc->gyro->dev;
    // spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiAccDetect(accDev_t *acc)
{   
    extDevice_t *dev = &acc->gyro->dev;
    bmi088EnableSPI(dev);

    if (bmi088AccRegisterRead(dev, BMI088_ACC_REG_CHIP_ID) != BMI088_ACC_CHIP_ID) {
        return false;
    }

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = bmi088AccRead;
    acc->acc_1G = 1365;   // 24g sensor scale

    return true;
}
/*  BMI088 acc init part end  */

#endif // USE_ACCGYRO_BMI088