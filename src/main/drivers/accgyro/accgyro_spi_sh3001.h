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

#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

// SH3001 registers (not the complete list)
typedef enum {
    SH3001_REG_ACC_X_DATA_L = 0x00,
    SH3001_REG_ACC_X_DATA_H = 0x01,
    SH3001_REG_ACC_Y_DATA_L = 0x02,
    SH3001_REG_ACC_Y_DATA_H = 0x03,
    SH3001_REG_ACC_Z_DATA_L = 0x04,
    SH3001_REG_ACC_Z_DATA_H = 0x05,
    SH3001_REG_GYRO_X_DATA_L = 0x06,
    SH3001_REG_GYRO_X_DATA_H = 0x07,
    SH3001_REG_GYRO_Y_DATA_L = 0x08,
    SH3001_REG_GYRO_Y_DATA_H = 0x09,
    SH3001_REG_GYRO_Z_DATA_L = 0x0A,
    SH3001_REG_GYRO_Z_DATA_H = 0x0B,
    SH3001_REG_TEMP_DATA_L = 0x0C,
    SH3001_REG_TEMP_DATA_H = 0x0D,
    // SH3001_REG_MAGIC
    SH3001_REG_CHIP_ID = 0x0F,          // chip id, should be 0x61
    SH3001_REG_TEMP_CONFIG_0 = 0x20,    // temperature sensor ODR
    SH3001_REG_TEMP_CONFIG_1 = 0x21,    // temperature sensor
    SH3001_REG_ACC_CONFIG_0 = 0x22,     // work mode, ADC dither, digital filter
    SH3001_REG_ACC_CONFIG_1 = 0x23,     // output data rate
    SH3001_REG_ACC_CONFIG_2 = 0x25,     // range
    SH3001_REG_ACC_CONFIG_3 = 0x26,     // lpf cutoff frequency
    SH3001_REG_GYRO_CONFIG_0 = 0x28,    // digital filter
    SH3001_REG_GYRO_CONFIG_1 = 0x29,    // output data rate
    SH3001_REG_GYRO_CONFIG_2 = 0x2B,    // lpf cutoff frequency
    SH3001_REG_INTERRUPT_EN_1 = 0x41,   // gyro DDRY interrupt enable
    SH3001_REG_INTERRUPT_CONFIG = 0x44, // interrupt pin config
    SH3001_REG_INTERRUPT_CONT_LIM = 0x45,// interrupt pin config
    SH3001_REG_INT_PINMAP_1 = 0x7A,     // interrupt pin config
    SH3001_REG_PAGE = 0x7F,             // page select

    SH3001_REG_GYRO_CONFIG_3 = 0x8F,    // x range
    SH3001_REG_GYRO_CONFIG_4 = 0x9F,    // y range
    SH3001_REG_GYRO_CONFIG_5 = 0xAF,    // z range
    SH3001_REG_CHIP_VERSION = 0xDD,     // chip version
} sh3001Register_e;

typedef struct {
    int8_t cXY;
    int8_t cXZ;
    int8_t cYX;
    int8_t cYZ;
    int8_t cZX;
    int8_t cZY;
    int8_t jX;
    int8_t jY;
    int8_t jZ;
    uint8_t xMulti;
    uint8_t yMulti;
    uint8_t zMulti;
    uint8_t paramP0;	
} compCoef_t;

// Contained in accgyro_spi_sh3001_init.c which is size optimized
uint8_t sh3001Detect(const extDevice_t *dev);
bool sh3001SpiAccDetect(accDev_t *acc);
bool sh3001SpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_sh3001.c which is speed optimized
void sh3001ExtiHandler(extiCallbackRec_t *cb);
bool sh3001AccRead(accDev_t *acc);
bool sh3001GyroRead(gyroDev_t *gyro);
