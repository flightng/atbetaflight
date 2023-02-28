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

// ASM330LHH registers (not the complete list)
typedef enum {
    ASM330LHH_REG_COUNTER_BDR1 = 0x0B,// Counter batch data rate register
    ASM330LHH_REG_INT1_CTRL = 0x0D,  // int pin 1 control
    ASM330LHH_REG_INT2_CTRL = 0x0E,  // int pin 2 control
    ASM330LHH_REG_WHO_AM_I = 0x0F,   // chip ID
    ASM330LHH_REG_CTRL1_XL = 0x10,   // accelerometer control
    ASM330LHH_REG_CTRL2_G = 0x11,    // gyro control
    ASM330LHH_REG_CTRL3_C = 0x12,    // control register 3
    ASM330LHH_REG_CTRL4_C = 0x13,    // control register 4
    ASM330LHH_REG_CTRL5_C = 0x14,    // control register 5
    ASM330LHH_REG_CTRL6_C = 0x15,    // control register 6
    ASM330LHH_REG_CTRL7_G = 0x16,    // control register 7
    ASM330LHH_REG_CTRL8_XL = 0x17,   // control register 8
    ASM330LHH_REG_CTRL9_XL = 0x18,   // control register 9
    ASM330LHH_REG_CTRL10_C = 0x19,   // control register 10
    ASM330LHH_REG_STATUS = 0x1E,     // status register
    ASM330LHH_REG_OUT_TEMP_L = 0x20, // temperature LSB
    ASM330LHH_REG_OUT_TEMP_H = 0x21, // temperature MSB
    ASM330LHH_REG_OUTX_L_G = 0x22,   // gyro X axis LSB
    ASM330LHH_REG_OUTX_H_G = 0x23,   // gyro X axis MSB
    ASM330LHH_REG_OUTY_L_G = 0x24,   // gyro Y axis LSB
    ASM330LHH_REG_OUTY_H_G = 0x25,   // gyro Y axis MSB
    ASM330LHH_REG_OUTZ_L_G = 0x26,   // gyro Z axis LSB
    ASM330LHH_REG_OUTZ_H_G = 0x27,   // gyro Z axis MSB
    ASM330LHH_REG_OUTX_L_A = 0x28,   // acc X axis LSB
    ASM330LHH_REG_OUTX_H_A = 0x29,   // acc X axis MSB
    ASM330LHH_REG_OUTY_L_A = 0x2A,   // acc Y axis LSB
    ASM330LHH_REG_OUTY_H_A = 0x2B,   // acc Y axis MSB
    ASM330LHH_REG_OUTZ_L_A = 0x2C,   // acc Z axis LSB
    ASM330LHH_REG_OUTZ_H_A = 0x2D,   // acc Z axis MSB
} asm330lhhRegister_e;

// Contained in accgyro_spi_asm330lhh_init.c which is size-optimized
uint8_t asm330lhhDetect(const extDevice_t *dev);
bool asm330lhhSpiAccDetect(accDev_t *acc);
bool asm330lhhSpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_asm330lhh.c which is speed-optimized
void asm330lhhExtiHandler(extiCallbackRec_t *cb);
bool asm330lhhAccRead(accDev_t *acc);
bool asm330lhhGyroRead(gyroDev_t *gyro);
