/*
 * This file is part of ATBetaflight
 */

#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

// BMI323 registers (not the complete list)
typedef enum{
    BMI323_REG_CHIP_ID = 0x00,

    BMI323_REG_ACC_DATA_X_LSB = 0x03,
    BMI323_REG_GYR_DATA_X_LSB = 0x06,

    BMI323_REG_FEATURE_IO1 = 0x11,

    BMI323_REG_ACC_CONF = 0x20,
    BMI323_REG_GYRO_CONF = 0x21,
    BMI323_REG_ALT_ACC_CONF = 0x28,
    BMI323_REG_ALT_GYRO_CONF = 0x29,

    BMI323_REG_IO_INT_CTRL = 0x38,
    BMI323_REG_INT_LATCH_CONF = 0x39,
    BMI323_REG_INT_MAP2 = 0x3B,

    BMI323_REG_CMD = 0x7E,
    BMI323_REG_CFG_RES = 0x7F,
} bmi323Register_e;

// Contained in accgyro_spi_bmi323_init.c which is size-optimized
uint8_t bmi323Detect(const extDevice_t *dev);
bool bmi323SpiAccDetect(accDev_t *acc);
bool bmi323SpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_bmi323.c which is speed-optimized
void bmi323ExtiHandler(extiCallbackRec_t* cb);
bool bmi323AccRead(accDev_t *acc);
bool bmi323GyroRead(gyroDev_t *gyro);
