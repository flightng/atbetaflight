/*
 * This file is part of ATBetaflight
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI323

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi323.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "sensors/gyro.h"

#define BMI323_MAX_SPI_CLK_HZ 10000000

#define BMI323_CHIP_ID 0x0043

typedef enum
{
    BMI323_VAL_CMD_SOFT_RESET = 0xDEAF,
    BMI323_VAL_CMD_GYRO_SELF_CALI = 0x0101,
    BMI323_VAL_ACC_CONF_ODR_100HZ = 0x8,        // bit[3:0], ODR 100Hz
    BMI323_VAL_ACC_CONF_ODR_800HZ = 0xB,        // bit[3:0], ODR 800Hz
    BMI323_VAL_ACC_CONF_RANGE_16G = 0x3,        // bit[6:4], range 16G
    BMI323_VAL_ACC_CONF_BW_ODR_QUARTER = 0x1,   // bit[7], BW=4/ODR
    BMI323_VAL_ACC_CONF_MODE_HIGH_PERF = 0x7,   // bit[14:12], high performance mode
    BMI323_VAL_GYRO_CONF_ODR_3200HZ = 0xD,      // bit[3:0], ODR 3200Hz
    BMI323_VAL_GYRO_CONF_RANGE_2000DPS = 0x4,   // bit[6:4], range 2000dps
    BMI323_VAL_GYRO_CONF_BW_ODR_HALF = 0x0,     // bit[7], BW=2/ODR
    BMI323_VAL_GYRO_CONF_BW_ODR_QUARTER = 0x1,  // bit[7], BW=4/ODR
    BMI323_VAL_GYRO_CONF_MODE_HIGH_PERF = 0x7,  // bit[14:12], high performance mode
    BMI323_VAL_ALT_ACC_CONF_DISABLE_ACC = 0x0,  // bit[14:12], disable acc
    BMI323_VAL_ALT_GYRO_CONF_DISABLE_GYRO = 0x0,// bit[14:12], disable gyro
    BMI323_VAL_IO_INT_CTRL = 0x0500,    // INT1 enabled, push-pull, active high
    BMI323_VAL_INT_LATCH_CONF = 0x0000, // no latch
    BMI323_VAL_INT_MAP2 = 0x0100,       // gyr_drdy_int map to INT1

} bmi323ConfigValues_e;

typedef enum
{
    BMI323_MASK_ACC_CONF = 0x70FF,
    BMI323_MASK_GYRO_CONF = 0x70FF,
    BMI323_MASK_ALT_ACC_CONF = 0x7000,
    BMI323_MASK_ALT_GYRO_CONF = 0x7000,
    BMI323_MASK_IO_INT_CTRL = 0x0707,
    BMI323_MASK_INT_LATCH_CONF = 0x0001,
    BMI323_MASK_INT_MAP2 = 0x0300,
    BMI323_MASK_FEATURE_IO1_STATE = 0x1800,
    BMI323_MASK_FEATURE_IO1_GYRO_SC_SUCC = 0x0030,
} bmi323ConfigMasks_e;

static uint16_t bmi323RegisterRead(const extDevice_t *dev, bmi323Register_e registerId)
{
    uint8_t data[3] = {0, 0, 0};

    if (spiReadRegMskBufRB(dev, registerId, data, 3))
    {
        return (uint16_t)((data[2] << 8) | data[1]); // LSB first since address is auto-incremented
    }
    else
    {
        return 0;
    }
}

static void bmi323RegisterWrite(const extDevice_t *dev, bmi323Register_e registerId, uint16_t data, unsigned delayMs)
{
    uint8_t buf[2] = {
        (uint8_t)(data & 0x00FF),
        (uint8_t)(data >> 8)};
    spiWriteRegBuf(dev, registerId, buf, 2);
    if (delayMs)
    {
        delay(delayMs);
    }
}

static void bmi323RegisterWriteBits(const extDevice_t *dev, bmi323Register_e registerId, bmi323ConfigMasks_e mask, uint16_t value, unsigned delayMs)
{
    uint16_t newValue = bmi323RegisterRead(dev, registerID);
    delayMicroseconds(2);
    newValue = (newValue & ~mask) | value;
    bmi323RegisterWrite(dev, registerID, newValue, delayMs);
}

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmi323EnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}

uint8_t bmi323Detect(const extDevice_t *dev)
{
    bmi323EnableSPI(dev);

    if (bmi323RegisterRead(dev, BMI323_REG_CHIP_ID) == BMI323_CHIP_ID)
    {
        return BMI_323_SPI;
    }

    return MPU_NONE;
}

uint8_t bmi323PerformCRT(const extDevice_t *dev)
{
    // check self-test status
    uint16_t data = bmi323RegisterRead(dev, BMI323_REG_FEATURE_IO1);
    for (int i = 0; i < 3; i++)
    {
        if ((data & BMI323_FEATURE_IO1_MASK_STATE) == 0x00)
        {
            break;
        }
        data = bmi323RegisterRead(dev, BMI323_REG_FEATURE_IO1);
        delay(10);
    }
    data = 0;
    // prerequisites for CRT
    // set acc to high performance mode and 100hz ODR
    bmi323RegisterWriteBits(dev, BMI323_REG_ACC_CONF, BMI323_MASK_ACC_CONF,( BMI323_VAL_GYRO_CONF_MODE_HIGH_PERF << 12 | BMI323_VAL_GYRO_CONF_BW_ODR_QUARTER << 7 | BMI323_VAL_ACC_CONF_RANGE_16G << 4 | BMI323_VAL_ACC_CONF_ODR_100HZ), 1);
    // disable altacc and altgyro
    bmi323RegisterWriteBits(dev, BMI323_REG_ALT_ACC_CONF, BMI323_MASK_ALT_ACC_CONF, BMI323_VAL_ALT_ACC_CONF_DISABLE_ACC << 12, 1);
    bmi323RegisterWriteBits(dev, BMI323_REG_ALT_GYRO_CONF, BMI323_MASK_ALT_GYRO_CONF, BMI323_VAL_ALT_GYRO_CONF_DISABLE_GYRO << 12, 1);
    // set crt option GYRO_OFFSET_EN GYRO_SENS_EN ,GYRO_APPLY_CORR TO AUTO APPLY TO OUTPUT  this is default value
    // NOOP
    // start self-calibration
    bmi323RegisterWrite(dev, BMI323_REG_CMD, BMI323_CMD_GYRO_SELF_CALI, 500);
    data = bmi323RegisterRead(dev, BMI323_REG_FEATURE_IO1);
    for (int i = 0; i < 6; i++)
    {
        if ((data & BMI323_MASK_FEATURE_IO1_GYRO_SC_SUCC) == BMI323_MASK_FEATURE_IO1_GYRO_SC_SUCC)
        {
            break;
        }
        delay(100);
        data = bmi323RegisterRead(dev, BMI323_REG_FEATURE_IO1);
    }
}

static uint8_t getBmiOsrMode()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMI323_VAL_GYRO_CONF_BW_ODR_QUARTER;
        case GYRO_HARDWARE_LPF_OPTION_1:
        case GYRO_HARDWARE_LPF_OPTION_2:
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMI323_VAL_GYRO_CONF_BW_ODR_HALF;
    }
    return 0;
}

static void bmi323Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi323EnableSPI(dev);
    bmi323RegisterWrite(dev, BMI323_REG_CMD, BMI323_CMD_SOFT_RESET, 100);

    // Toggle the chip into SPI mode
    bmi323EnableSPI(dev);

    bmi323PerformCRT(dev);

    // (Re-)Configure the accelerometer
    bmi323RegisterWriteBits(dev, BMI323_REG_ACC_CONF, BMI323_MASK_ACC_CONF,(BMI323_VAL_GYRO_CONF_MODE_HIGH_PERF << 12 | BMI323_VAL_GYRO_CONF_BW_ODR_QUARTER << 7 | BMI323_VAL_ACC_CONF_RANGE_16G << 4 | BMI323_VAL_ACC_CONF_ODR_800HZ), 1);

    // Configure the gyroscope
    bmi323RegisterWriteBits(dev, BMI323_REG_GYRO_CONF, BMI323_MASK_GYRO_CONF, (BMI323_VAL_GYRO_CONF_MODE_HIGH_PERF << 12 | getBmiOsrMode() << 7 | BMI323_VAL_GYRO_CONF_RANGE_2000DPS << 4 | BMI323_VAL_GYRO_CONF_ODR_3200HZ), 1);

    // Configure the interrupt
    bmi323RegisterWriteBits(dev, BMI323_REG_IO_INT_CTRL, BMI323_MASK_IO_INT_CTRL, BMI323_VAL_IO_INT_CTRL, 1);
    bmi323RegisterWriteBits(dev, BMI323_REG_INT_LATCH_CONF, BMI323_MASK_INT_LATCH_CONF, BMI323_VAL_INT_LATCH_CONF, 1);
    bmi323RegisterWriteBits(dev, BMI323_REG_INT_MAP2, BMI323_MASK_INT_MAP2, BMI323_VAL_INT_MAP2, 1);
}

static void bmi323_init(extDevice_t *dev)
{
    bmi323EnableSPI(dev);
    bmi323RegisterWrite(dev, BMI323_REG_CMD, BMI323_CMD_SOFT_RESET, 100);
    bmi323EnableSPI(dev);
}

#ifdef USE_GYRO_EXTI
static void bmi323IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE)
    {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi323ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif

static void bmi323SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi323Config(gyro);

#ifdef USE_GYRO_EXTI
    bmi323IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMI323_MAX_SPI_CLK_HZ));
}

static void bmi323SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4; // 16G sensor scale
}

bool bmi323SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_323_SPI)
    {
        return false;
    }

    acc->initFn = bmi323SpiAccInit;
    acc->readFn = bmi323AccRead;

    return true;
}

bool bmi323SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_323_SPI)
    {
        return false;
    }

    gyro->initFn = bmi323SpiGyroInit;
    gyro->readFn = bmi323GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

#endif // end of USE_ACCGYRO_BMI323
