/*
 * This file is part of AtBetaflight a fork of Betaflight 
 * @author EMSR(shanggl@wo.cn)
 * 
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
#define BMI323_INIT_SPI_CLK_HZ 4000000



static uint8_t bmiRegisterRead(const extDevice_t *dev, uint8_t registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static uint16_t bmiRegisterRead16(const extDevice_t *dev, uint8_t registerId)
{
    uint8_t data[3] = { 0, 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 3)) {
        return (uint16_t)( (data[2]<<8) | data[1] );    // LSB first since address is auto-incremented
    } else {
        return 0;
    }
}



static void bmiRegisterWrite16(const extDevice_t *dev, uint8_t registerId, uint16_t data, unsigned delayMs)
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

    if (bmiRegisterRead(dev, BMI323_REG_CHIP_ID) == BMI323_WHO_AMI) {
        return BMI_323_SPI;
    }

    return MPU_NONE;
}


static void bmi323_init( extDevice_t *dev ) {
  bmi323EnableSPI(dev);
  bmiRegisterWrite16(dev,BMI323_REG_CMD, BMI323_CMD_SOFT_RESET, 100);
  bmi323EnableSPI(dev);
}

static void bmi323_init_config( extDevice_t *dev ) {
  // init acc conf
  uint16_t regdata = 0;
  regdata = BMI3_ACC_BW_ODR_QUARTER << 7 | BMI323_ACC_RANGE_16G << 4 | BMI323_ACC_ODR_800HZ;
  regdata |= BMI3_ACC_MODE_HIGH_PERF << 12 | BMI323_ACC_AVG1 << 8;
  bmiRegisterWrite16(dev,BMI323_REG_ACC_CONF, regdata, 1);

  // init gyro conf
  regdata = 0;
  regdata = BMI323_GYR_BW_ODR_QUARTER << 7 | BMI323_GYR_RANGE_2000DPS << 4 | BMI323_GYR_ODR_3200HZ;
  regdata |= BMI323_GYR_MODE_HIGH_PERF << 12 | BMI323_GYR_AVG1 << 8;
  bmiRegisterWrite16(dev,BMI323_REG_GYRO_CONF, regdata, 1);

  // init data ready interupt to pin int1/ push_pull /active_high  NO_LATCH(default)

  bmiRegisterWrite16(dev,BMI323_REG_INT_LATCH_CONF, 0x00, 1);
  // BMI323_REG_INT_MAP2 accready /gyro_ready
  bmiRegisterWrite16(dev,BMI323_REG_INT_MAP2, 0x140, 15);
  // BMI323_REG_IO_INT_CTRL push_pull/active_high ENABLE
  regdata=0;
  regdata=BMI3_INT_OUTPUT_ENABLE << 2 | BMI3_INT_PUSH_PULL << 1 | BMI3_INT_ACTIVE_HIGH <<0 ;
  bmiRegisterWrite16(dev,BMI323_REG_IO_INT_CTRL, regdata, 1);

}

//enable bmi323 crt self calibration feature
static void bmi323_enable_crt( extDevice_t *dev){
//step.0 check self-test status 
  uint16_t regData=bmiRegisterRead16(dev,BMI323_REG_FEATURE_IO1);
  for(int i=0; i<3;i++){
    if((regData & BMI323_FEATURE_IO1_MASK_STATE )==0x00){
      break;
    }
    regData=bmiRegisterRead16(dev,BMI323_REG_FEATURE_IO1);
    delay(10);
  }
  regData=0;
//step.1 set acc high-proformence mode and  ord set to 100hz
  regData = BMI3_ACC_BW_ODR_QUARTER << 7 | BMI323_ACC_RANGE_16G << 4 | BMI323_ACC_ODR_100HZ;
  regData |= BMI3_ACC_MODE_HIGH_PERF << 12 | BMI323_ACC_AVG1 << 8;
  bmiRegisterWrite16(dev,BMI323_REG_ACC_CONF, regData, 1);
  //disable alt-acc and alt-gyro 
  bmiRegisterWrite16(dev,BMI323_REG_ALT_ACC_CONF,0x00,1);
  bmiRegisterWrite16(dev,BMI323_REG_ALT_GYRO_CONF,0X00,1);

//step.2 set crt option GYRO_OFFSET_EN GYRO_SENS_EN ,GYRO_APPLY_CORR TO AUTO APPLY TO OUTPUT  this is default value 
//NOOP
//step.3 start self-calibration &delay 350+80ms 
  bmiRegisterWrite16(dev,BMI323_REG_CMD, BMI323_CMD_GYRO_SELF_CALI, 500);
//step.4 pull status to check till end  
  regData=bmiRegisterRead16(dev,BMI323_REG_FEATURE_IO1);
  for(int i=0; i<6;i++){
    if((regData & BMI323_FEATURE_IO1_MASK_GYRO_SC_SUCC)==0x30){
      break;
    }
    delay(100);
    regData=bmiRegisterRead16(dev,BMI323_REG_FEATURE_IO1);
  }
}

void bmi323Config(gyroDev_t *gyro) {
  extDevice_t *dev = &gyro->dev;
  bmi323_init(dev);
  bmi323_enable_crt(dev);
  bmi323_init_config(dev);
}



#ifdef USE_GYRO_EXTI
static void bmi323IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
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
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool bmi323SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_323_SPI) {
        return false;
    }

    acc->initFn = bmi323SpiAccInit;
    acc->readFn = bmi323AccRead;

    return true;
}

bool bmi323SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_323_SPI) {
        return false;
    }

    gyro->initFn = bmi323SpiGyroInit;
    gyro->readFn = bmi323GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}



#endif //end of USE_ACCGYRO_BMI323
