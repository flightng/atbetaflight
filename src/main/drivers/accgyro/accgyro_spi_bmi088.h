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

#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

// BMI088 registers (not the complete list)
typedef enum {
    /// Acc Registers
    BMI088_ACC_REG_CHIP_ID = 0x00,
    BMI088_ACC_REG_RATE_X_LSB = 0x12,
    BMI088_ACC_REG_SENSORTIME_0 = 0x18,
    BMI088_ACC_REG_SENSORTIME_1 = 0x19,
    BMI088_ACC_REG_SENSORTIME_2 = 0x1A,
    BMI088_ACC_REG_TEMP_MSB = 0x22,
    BMI088_ACC_REG_TEMP_LSB = 0x23,
    BMI088_ACC_REG_ACC_CONF = 0x40,
    BMI088_ACC_REG_ACC_RANGE = 0x41,
    BMI088_ACC_REG_INT1_IO_CONF = 0x53,
    BMI088_ACC_REG_INT_MAP_DATA = 0x58,
    BMI088_ACC_REG_ACC_PWR_CONF = 0x7C,
    BMI088_ACC_REG_ACC_PWR_CTRL = 0x7D,
    BMI088_ACC_REG_SOFTRESET = 0x7E,
    /// Gyro Registers
    BMI088_GYRO_REG_CHIP_ID = 0x00,
    BMI088_GYRO_RET_RATE_X_LSB = 0x02,
    BMI088_GYRO_REG_RANGE = 0x0F,
    BMI088_GYRO_REG_BANDWIDTH = 0x10,
    BMI088_GYRO_REG_SOFTRESET = 0x14,
    BMI088_GYRO_REG_INT_CTRL = 0x15,
    BMI088_GYRO_REG_INT3_INT4_IO_CONF = 0x16,
    BMI088_GYRO_REG_INT3_INT4_IO_MAP = 0x18,
    BMI088_GYRO_REG_SELF_TEST = 0x3C,
} bmi088Register_e;

// Contained in init.c which is size-optimized
uint8_t bmi088GyroDetect(const extDevice_t *dev);
bool bmi088SpiAccDetect(accDev_t *acc);
bool bmi088SpiGyroDetect(gyroDev_t *gyro);

// Comtained in .c which is speed-optimized
// acc functions
bool bmi088AccRead(accDev_t *acc);
// gyro functions
bool bmi088GyroRead(gyroDev_t *gyro);
void bmi088GyroExtiHandler(extiCallbackRec_t *cb);
