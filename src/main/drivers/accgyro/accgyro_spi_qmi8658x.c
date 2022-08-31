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

/*
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_QMI8658A)

#include "common/axis.h"
#include "common/maths.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_qmi8658x.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 15 MHz max SPI frequency
#define QMI8658X_MAX_SPI_CLK_HZ 15000000

// 10 MHz max SPI frequency for intialisation
#define QMI8658X_MAX_SPI_INIT_CLK_HZ 1000000

static void qmi8658xSpiInit(const extDevice_t *dev)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }


    spiSetClkDivisor(dev, spiCalculateDivider(QMI8658X_MAX_SPI_CLK_HZ));

    hardwareInitialised = true;
}

uint8_t qmi8658xSpiDetect(const extDevice_t *dev)
{
	qmi8658xSpiInit(dev);

    spiSetClkDivisor(dev, spiCalculateDivider(QMI8658X_MAX_SPI_INIT_CLK_HZ));


    uint8_t mpuDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiReadRegMsk(dev, Qmi8658Register_WhoAmI);
        switch (whoAmI) {
        case QMI8658A_WHO_AM_I_CONST:
        	mpuDetected = QMI_8658A_SPI;
            break;
        default:
        	mpuDetected = MPU_NONE;
            break;
        }
        if (mpuDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    spiSetClkDivisor(dev, spiCalculateDivider(QMI8658X_MAX_SPI_CLK_HZ));

    return mpuDetected;
}

void qmi8658xAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool qmi8658xSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case QMI_8658A_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = qmi8658xAccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

void qmi8658xGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro); //init exti 
    gyro->accDataReg = Qmi8658Register_Ax_L; //reconfig register
    gyro->gyroDataReg = Qmi8658Register_Gx_L;//reconfig register

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(QMI8658X_MAX_SPI_INIT_CLK_HZ));
//reset and cali 
    spiWriteReg(&gyro->dev,Qmi8658Register_Reset, 0xb0);
	delay(20);	// delay cost 15ms to reset 
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	delay(2200);	// delay 2000ms above
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_NOP);
	delay(100);	// delay

//  enable spi/ int1 /fifo
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl1, 0x60 | QMI8658_INT1_ENABLE |QMI8658_FIFO_MAP_INT1);  //0010	1100 not auto increase address & big end
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl8,0xc0); //disable motion engine  tap Pedometer
//  disable all sensors 
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl7,QMI8658_DISABLE_ALL);
    delay(15);
//init fifo
    spiWriteReg(&gyro->dev,Qmi8658Register_FifoCtrl,(unsigned char)(qmi8658_Fifo_16 | qmi8658_Fifo_Fifo));
    spiWriteReg(&gyro->dev,Qmi8658Register_FifoWmkTh, QMI8658_FIFO_WATER_MARK);

//  enable acc & gyro
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl7,QMI8658_ACCGYR_ENABLE);
    delay(100);
//Configure Accelerometer, Enable Self-Test
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl2, Qmi8658AccRange_16g | Qmi8658AccOdr_8000Hz );// | 0x80 to enable self test 
	 uint8_t ctl_dada= spiReadRegMsk(&gyro->dev,Qmi8658Register_Ctrl5);
	 ctl_dada &= 0xf0;
	 ctl_dada |= A_LSP_MODE_3 | 0x01;
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl5,ctl_dada);
    delay(100);
//Configure Gyroscope, Enable Self-Test
    spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl3,  Qmi8658GyrRange_2048dps| Qmi8658GyrOdr_8000Hz );//| 0x80 to enable self test 
     ctl_dada= spiReadRegMsk(&gyro->dev,Qmi8658Register_Ctrl5);
	 ctl_dada &= 0x0f;
	 ctl_dada |= G_LSP_MODE_3 | 0x10;
	 spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl5,ctl_dada);

//FLASH FIFO
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_Rst_Fifo);
	delay(15);
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_NOP);


    delay(100);
   spiSetClkDivisor(&gyro->dev, spiCalculateDivider(QMI8658X_MAX_SPI_CLK_HZ));
}

#ifdef USE_GYRO_DLPF_EXPERIMENTAL

#define QMI8658_FIFO_FRAME_SIZE 0xC

static bool qmi8658xGyroReadFifo(gyroDev_t * gyro){

    enum {
    	IDX_ACC_XOUT_L=0,
		IDX_ACC_XOUT_H,
		IDX_ACC_YOUT_L,
		IDX_ACC_YOUT_H,
		IDX_ACC_ZOUT_L,
		IDX_ACC_ZOUT_H,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t fifo_status[2] = {0,0};
	uint8_t fifo_bytes = 0;
	bool dataRead=false;


	spiReadRegMskBufRB(&gyro->dev,(uint8_t)Qmi8658Register_FifoCount, fifo_status, 2);
	fifo_bytes = (uint8_t)(((fifo_status[1]&0x03)<<8)|fifo_status[0]);


    STATIC_DMA_DATA_AUTO uint8_t qmi_tx_buf[BUFFER_SIZE] = {Qmi8658Register_FifoData | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0};
    STATIC_DMA_DATA_AUTO uint8_t qmi_rx_buf[BUFFER_SIZE];


	if(fifo_bytes > 0)
	{
		spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9,qmi8658_Ctrl9_Cmd_Req_Fifo);   //switch fifo to read mode 
		spiReadWriteBuf(&gyro->dev, (uint8_t *)qmi_tx_buf, qmi_rx_buf, BUFFER_SIZE);   // read data
		spiWriteReg(&gyro->dev,Qmi8658Register_FifoCtrl,(unsigned char)(qmi8658_Fifo_16 | qmi8658_Fifo_Fifo));//SWITCH fifo to write mode
	}
	spiWriteReg(&gyro->dev,Qmi8658Register_Ctrl9,qmi8658_Ctrl9_Cmd_Rst_Fifo);


	const int16_t gyroX = (int16_t)((qmi_rx_buf[IDX_GYRO_XOUT_H] << 8) | qmi_rx_buf[IDX_GYRO_XOUT_L]);
	const int16_t gyroY = (int16_t)((qmi_rx_buf[IDX_GYRO_YOUT_H] << 8) | qmi_rx_buf[IDX_GYRO_YOUT_L]);
	const int16_t gyroZ = (int16_t)((qmi_rx_buf[IDX_GYRO_ZOUT_H] << 8) | qmi_rx_buf[IDX_GYRO_ZOUT_L]);

	if ((gyroX != INT16_MIN) || (gyroY != INT16_MIN) || (gyroZ != INT16_MIN)) {
		gyro->gyroADCRaw[X] = gyroX;
		gyro->gyroADCRaw[Y] = gyroY;
		gyro->gyroADCRaw[Z] = gyroZ;
		dataRead = true;
	}


	return dataRead;
}

#endif

bool qmi8658xSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case QMI_8658A_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = qmi8658xGyroInit;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    gyro->readFn=qmi8658xGyroReadFifo;
#else
    gyro->readFn = mpuGyroReadSPI;//int2 is not ready . use pull first
#endif
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
