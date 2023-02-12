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

// QMI8658 registers (not the complete list)
typedef enum {
    QMI8658_REG_WHO_AM_I = 0x00,       // chip id, should be 0x05
    QMI8658_REG_REVISION_ID = 0x01,    // chip revision, should be 0x7C
    QMI8658_REG_CTRL1 = 0x02,          // SPI Interface and Sensor Enable
    QMI8658_REG_CTRL2 = 0x03,          // Accelerometer: Output Data Rate, Full Scale, Self-Test
    QMI8658_REG_CTRL3 = 0x04,          // Gyroscope: Output Data Rate, Full Scale, Self-Test
    QMI8658_REG_CTRL5 = 0x06,          // Low pass filter setting
    QMI8658_REG_CTRL7 = 0x08,          // Enable Sensors
    QMI8658_REG_CTRL9 = 0x0A,          // Host Commands
    QMI8658_REG_AX_L = 0x35,           // Accelerometer X axis LSB
    QMI8658_REG_AX_H = 0x36,           // Accelerometer X axis MSB
    QMI8658_REG_AY_L = 0x37,           // Accelerometer Y axis LSB
    QMI8658_REG_AY_H = 0x38,           // Accelerometer Y axis MSB
    QMI8658_REG_AZ_L = 0x39,           // Accelerometer Z axis LSB
    QMI8658_REG_AZ_H = 0x3A,           // Accelerometer Z axis MSB
    QMI8658_REG_GX_L = 0x3B,           // Gyroscope X axis LSB
    QMI8658_REG_GX_H = 0x3C,           // Gyroscope X axis MSB
    QMI8658_REG_GY_L = 0x3D,           // Gyroscope Y axis LSB
    QMI8658_REG_GY_H = 0x3E,           // Gyroscope Y axis MSB
    QMI8658_REG_GZ_L = 0x3F,           // Gyroscope Z axis LSB
    QMI8658_REG_GZ_H = 0x40,           // Gyroscope Z axis MSB
    QMI8658_REG_RESET = 0x60,          // Soft Reset Register
} qmi8658Register_e;

// Contained in accgyro_spi_qmi8658_init.c which is size optimized
uint8_t qmi8658Detect(const extDevice_t *dev);
bool qmi8658SpiAccDetect(accDev_t *acc);
bool qmi8658SpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_qmi8658.c which is speed optimized
void qmi8658ExtiHandler(extiCallbackRec_t *cb);
bool qmi8658AccRead(accDev_t *acc);
bool qmi8658GyroRead(gyroDev_t *gyro);
