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

#pragma once

#include "drivers/bus.h"


#define QMI8658A_WHO_AM_I_CONST 		(0x05)
#define QMI8658_DISABLE_ALL				(0x0)
#define QMI8658_ACC_ENABLE				(0x1)
#define QMI8658_GYR_ENABLE				(0x2)
#define QMI8658_ACCGYR_ENABLE			(QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)

#define QMI8658_STATUS1_CMD_DONE		(0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT	(0x04)

#define QMI8658_CTRL8_DATAVALID_EN		0x40		// bit6:1 int1, 0 int2
#define QMI8658_CTRL8_PEDOMETER_EN		0x10
#define QMI8658_CTRL8_SIGMOTION_EN		0x08
#define QMI8658_CTRL8_NOMOTION_EN		0x04
#define QMI8658_CTRL8_ANYMOTION_EN		0x02
#define QMI8658_CTRL8_TAP_EN			0x01

#define QMI8658_INT1_ENABLE				0x08
#define QMI8658_INT2_ENABLE				0x10

#define QMI8658_DRDY_DISABLE			0x20	// ctrl7

#define QMI8658_FIFO_MAP_INT1			0x04	// ctrl1
#define QMI8658_FIFO_MAP_INT2			~0x04	// ctrl1

#define QMI8658_FIFO_WATER_MARK			0xC		// 1 sample 12byte

enum Qmi8658Register
{
	Qmi8658Register_WhoAmI = 0,
	Qmi8658Register_Revision,
	Qmi8658Register_Ctrl1,
	Qmi8658Register_Ctrl2,
	Qmi8658Register_Ctrl3,
	Qmi8658Register_Ctrl4,
	Qmi8658Register_Ctrl5,
	Qmi8658Register_Ctrl6,
	Qmi8658Register_Ctrl7,
	Qmi8658Register_Ctrl8,
	Qmi8658Register_Ctrl9,
	Qmi8658Register_Cal1_L = 11,
	Qmi8658Register_Cal1_H,
	Qmi8658Register_Cal2_L,
	Qmi8658Register_Cal2_H,
	Qmi8658Register_Cal3_L,
	Qmi8658Register_Cal3_H,
	Qmi8658Register_Cal4_L,
	Qmi8658Register_Cal4_H,
	Qmi8658Register_FifoWmkTh = 19,
	Qmi8658Register_FifoCtrl = 20,
	Qmi8658Register_FifoCount = 21,
	Qmi8658Register_FifoStatus = 22,
	Qmi8658Register_FifoData = 23,
	Qmi8658Register_StatusI2CM = 44,
	Qmi8658Register_StatusInt = 45,
	Qmi8658Register_Status0,
	Qmi8658Register_Status1,
	Qmi8658Register_Timestamp_L = 48,
	Qmi8658Register_Timestamp_M,
	Qmi8658Register_Timestamp_H,
	Qmi8658Register_Tempearture_L = 51,
	Qmi8658Register_Tempearture_H,
	Qmi8658Register_Ax_L = 0x35,
	Qmi8658Register_Ax_H,
	Qmi8658Register_Ay_L,
	Qmi8658Register_Ay_H,
	Qmi8658Register_Az_L,
	Qmi8658Register_Az_H,
	Qmi8658Register_Gx_L = 0x3B,
	Qmi8658Register_Gx_H,
	Qmi8658Register_Gy_L,
	Qmi8658Register_Gy_H,
	Qmi8658Register_Gz_L,
	Qmi8658Register_Gz_H,
	Qmi8658Register_Mx_L = 65,
	Qmi8658Register_Mx_H,
	Qmi8658Register_My_L,
	Qmi8658Register_My_H,
	Qmi8658Register_Mz_L,
	Qmi8658Register_Mz_H,
	Qmi8658Register_firmware_id = 73,
	Qmi8658Register_uuid = 81,

	Qmi8658Register_Pedo_L = 90,
	Qmi8658Register_Pedo_M = 91,
	Qmi8658Register_Pedo_H = 92,

	Qmi8658Register_Reset = 0x60
};

enum qmi8658_Ois_Register
{
	qmi8658_OIS_Reg_Ctrl1 = 0x02,
	qmi8658_OIS_Reg_Ctrl2,
	qmi8658_OIS_Reg_Ctrl3,
	qmi8658_OIS_Reg_Ctrl5	= 0x06,
	qmi8658_OIS_Reg_Ctrl7   = 0x08,
	qmi8658_OIS_Reg_StatusInt = 0x2D,
	qmi8658_OIS_Reg_Status0   = 0x2E,
	qmi8658_OIS_Reg_Ax_L  = 0x33,
	qmi8658_OIS_Reg_Ax_H,
	qmi8658_OIS_Reg_Ay_L,
	qmi8658_OIS_Reg_Ay_H,
	qmi8658_OIS_Reg_Az_L,
	qmi8658_OIS_Reg_Az_H,

	qmi8658_OIS_Reg_Gx_L  = 0x3B,
	qmi8658_OIS_Reg_Gx_H,
	qmi8658_OIS_Reg_Gy_L,
	qmi8658_OIS_Reg_Gy_H,
	qmi8658_OIS_Reg_Gz_L,
	qmi8658_OIS_Reg_Gz_H,
};

enum qmi8658_Ctrl9Command
{
	qmi8658_Ctrl9_Cmd_NOP					= 0X00,
	qmi8658_Ctrl9_Cmd_GyroBias				= 0X01,
	qmi8658_Ctrl9_Cmd_Rqst_Sdi_Mod			= 0X03,
	qmi8658_Ctrl9_Cmd_Rst_Fifo				= 0X04,
	qmi8658_Ctrl9_Cmd_Req_Fifo				= 0X05,
	qmi8658_Ctrl9_Cmd_I2CM_Write			= 0X06,
	qmi8658_Ctrl9_Cmd_WoM_Setting			= 0x08,
	qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset	= 0x09,
	qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset	= 0x0A,
	qmi8658_Ctrl9_Cmd_EnableExtReset		= 0x0B,
	qmi8658_Ctrl9_Cmd_EnableTap				= 0x0C,
	qmi8658_Ctrl9_Cmd_EnablePedometer		= 0x0D,
	qmi8658_Ctrl9_Cmd_Motion				= 0x0E,
	qmi8658_Ctrl9_Cmd_CopyUsid				= 0x10,
	qmi8658_Ctrl9_Cmd_SetRpu				= 0x11,
	qmi8658_Ctrl9_Cmd_On_Demand_Cali		= 0xA2,
	qmi8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable	= 0xF8
};


enum qmi8658_LpfConfig
{
	Qmi8658Lpf_Disable,
	Qmi8658Lpf_Enable
};

enum qmi8658_HpfConfig
{
	Qmi8658Hpf_Disable,
	Qmi8658Hpf_Enable
};

enum qmi8658_StConfig
{
	Qmi8658St_Disable,
	Qmi8658St_Enable
};

enum qmi8658_LpfMode
{
	A_LSP_MODE_0 = 0x00<<1,
	A_LSP_MODE_1 = 0x01<<1,
	A_LSP_MODE_2 = 0x02<<1,
	A_LSP_MODE_3 = 0x03<<1,

	G_LSP_MODE_0 = 0x00<<5,
	G_LSP_MODE_1 = 0x01<<5,
	G_LSP_MODE_2 = 0x02<<5,
	G_LSP_MODE_3 = 0x03<<5
};

enum qmi8658_AccRange
{
	Qmi8658AccRange_2g = 0x00 << 4,
	Qmi8658AccRange_4g = 0x01 << 4,
	Qmi8658AccRange_8g = 0x02 << 4,
	Qmi8658AccRange_16g = 0x03 << 4
};


enum qmi8658_AccOdr
{
	Qmi8658AccOdr_8000Hz = 0x00,
	Qmi8658AccOdr_4000Hz = 0x01,
	Qmi8658AccOdr_2000Hz = 0x02,
	Qmi8658AccOdr_1000Hz = 0x03,
	Qmi8658AccOdr_500Hz = 0x04,
	Qmi8658AccOdr_250Hz = 0x05,
	Qmi8658AccOdr_125Hz = 0x06,
	Qmi8658AccOdr_62_5Hz = 0x07,
	Qmi8658AccOdr_31_25Hz = 0x08,
	Qmi8658AccOdr_LowPower_128Hz = 0x0c,
	Qmi8658AccOdr_LowPower_21Hz = 0x0d,
	Qmi8658AccOdr_LowPower_11Hz = 0x0e,
	Qmi8658AccOdr_LowPower_3Hz = 0x0f
};

enum qmi8658_GyrRange
{
	Qmi8658GyrRange_16dps = 0 << 4,
	Qmi8658GyrRange_32dps = 1 << 4,
	Qmi8658GyrRange_64dps = 2 << 4,
	Qmi8658GyrRange_128dps = 3 << 4,
	Qmi8658GyrRange_256dps = 4 << 4,
	Qmi8658GyrRange_512dps = 5 << 4,
	Qmi8658GyrRange_1024dps = 6 << 4,
	Qmi8658GyrRange_2048dps = 7 << 4
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum qmi8658_GyrOdr
{
	Qmi8658GyrOdr_8000Hz = 0x00,
	Qmi8658GyrOdr_4000Hz = 0x01,
	Qmi8658GyrOdr_2000Hz = 0x02,
	Qmi8658GyrOdr_1000Hz = 0x03,
	Qmi8658GyrOdr_500Hz	= 0x04,
	Qmi8658GyrOdr_250Hz	= 0x05,
	Qmi8658GyrOdr_125Hz	= 0x06,
	Qmi8658GyrOdr_62_5Hz 	= 0x07,
	Qmi8658GyrOdr_31_25Hz	= 0x08
};

enum qmi8658_AccUnit
{
	Qmi8658AccUnit_g,
	Qmi8658AccUnit_ms2
};

enum qmi8658_GyrUnit
{
	Qmi8658GyrUnit_dps,
	Qmi8658GyrUnit_rads
};

enum qmi8658_FifoMode
{
	qmi8658_Fifo_Bypass = 0,
	qmi8658_Fifo_Fifo = 1,
	qmi8658_Fifo_Stream = 2,
	qmi8658_Fifo_StreamToFifo = 3
};


enum qmi8658_FifoWmkLevel
{
	qmi8658_Fifo_WmkEmpty = 		(0 << 4),
	qmi8658_Fifo_WmkOneQuarter =	(1 << 4),
	qmi8658_Fifo_WmkHalf =			(2 << 4),
	qmi8658_Fifo_WmkThreeQuarters =	(3 << 4)
};

enum qmi8658_FifoSize
{
	qmi8658_Fifo_16 = (0 << 2),
	qmi8658_Fifo_32 = (1 << 2),
	qmi8658_Fifo_64 = (2 << 2),
	qmi8658_Fifo_128 = (3 << 2)
};

enum qmi8658_Interrupt
{
	qmi8658_Int_none,
	qmi8658_Int1,
	qmi8658_Int2,

	qmi8658_Int_total
};

enum qmi8658_InterruptState
{
	Qmi8658State_high = (1 << 7),
	Qmi8658State_low  = (0 << 7)
};

bool qmi8658xAccDetect(accDev_t *acc);
bool qmi8658xGyroDetect(gyroDev_t *gyro);

void qmi8658xAccInit(accDev_t *acc);
void qmi8658xGyroInit(gyroDev_t *gyro);

uint8_t qmi8658xSpiDetect(const extDevice_t *dev);

bool qmi8658xSpiAccDetect(accDev_t *acc);
bool qmi8658xSpiGyroDetect(gyroDev_t *gyro);
