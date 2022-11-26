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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

//fixme: 关于传输的addr_ 左移 1位的问题 前7位是地址， 最后一位 1 表示读， 0 表示写,因为对传感器都一直是读，所以addr_<<1

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

//#define I2C_TIMEOUT                      0x3FFFFF
#define I2C_TIMEOUT                      0x8700 //about 120 us at 288 mhz

#ifdef USE_I2C_DEVICE_1
void I2C1_ERR_IRQHandler(void)
{
	i2c_err_irq_handler(&i2cDevice[I2CDEV_1].handle);
}

void I2C1_EVT_IRQHandler(void)
{
	i2c_evt_irq_handler(&i2cDevice[I2CDEV_1].handle);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ERR_IRQHandler(void)
{
	i2c_err_irq_handler(&i2cDevice[I2CDEV_2].handle);
}

void I2C2_EVT_IRQHandler(void)
{
	i2c_evt_irq_handler(&i2cDevice[I2CDEV_2].handle);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ERR_IRQHandler(void)
{
	i2c_err_irq_handler(&i2cDevice[I2CDEV_3].handle);
}

void I2C3_EVT_IRQHandler(void)
{
	i2c_evt_irq_handler(&i2cDevice[I2CDEV_3].handle);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ERR_IRQHandler(void)
{
	i2c_err_irq_handler(&i2cDevice[I2CDEV_4].handle);
}

void I2C4_EVT_IRQHandler(void)
{
	i2c_evt_irq_handler(&i2cDevice[I2CDEV_4].handle);
}
#endif

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    (void)device;
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Blocking write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_handle_type *pHandle = &i2cDevice[device].handle;

    if (!pHandle->i2cx) {
        return false;
    }

    i2c_status_type status;

    if (reg_ == 0xFF)
    {
		status = i2c_master_transmit(pHandle ,addr_ << 1 , &data, 1, I2C_TIMEOUT); // addr_ << 1

		if(status !=  I2C_OK)
		{
		  /* wait for the stop flag to be set  */
		  i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

		  /* clear stop flag */
		i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
		}
    }

    else
    {
        status = i2c_memory_write(pHandle ,I2C_MEM_ADDR_WIDIH_8,addr_ << 1 , reg_, &data, 1, I2C_TIMEOUT_US); // addr_ << 1

        if(status !=  I2C_OK)
        {
          /* wait for the stop flag to be set  */
          i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

          /* clear stop flag */
      	i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
        }
    }
    if (status != I2C_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Non-blocking write
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_handle_type *pHandle = &i2cDevice[device].handle;

    if (!pHandle->i2cx) {
        return false;
    }

    i2c_status_type status;
    //i2c_memory_write_int(i2c_handle_type* hi2c, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
    status = i2c_memory_write_int(pHandle ,I2C_MEM_ADDR_WIDIH_8,addr_ << 1, reg_,data, len_,I2C_TIMEOUT);

    if(status !=  I2C_OK)
    {
      /* wait for the stop flag to be set  */
      i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

      /* clear stop flag */
    	i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
    }
    else
    {
      /* wait for the communication to end */
    }

    if (status == I2C_ERR_STEP_1) {//BUSY
        return false;
    }

    if (status != I2C_OK)
    {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Blocking read
/* read reg value from device
 * @device  eg I2C1
 * @addr_ 	device addr eg 0x58
 * @reg_ 	device reg addr
 * @len		read data length
 * @buf 	read data to buf
 */
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_handle_type *pHandle = &i2cDevice[device].handle;

    if (!pHandle->i2cx) {
        return false;
    }

    i2c_status_type status;

    if (reg_ == 0xFF)//任意地址
    {
    	status = i2c_master_receive(pHandle ,addr_ << 1 , buf, len, I2C_TIMEOUT);

        if(status !=  I2C_OK)
        {
          /* wait for the stop flag to be set  */
          i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

          /* clear stop flag */
         i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
        }
    }

    else
    {
      status = i2c_memory_read(pHandle,I2C_MEM_ADDR_WIDIH_8, addr_ << 1, reg_, buf, len, I2C_TIMEOUT);

      if(status !=  I2C_OK)
      {
        /* wait for the stop flag to be set  */
        i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

        /* clear stop flag */
    	i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
      }
    }


    if (status != I2C_OK) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Non-blocking read
/*i2cReadBuffer read reg value from device
 * @device  i2c device eg i2c1
 * @addr_   device address
 * @reg_	reg value
 * @len		read data length
 * @buf 	read data to buf
 */
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_handle_type *pHandle = &i2cDevice[device].handle;

    if (!pHandle->i2cx) {
        return false;
    }

    i2c_status_type status;

    status = i2c_memory_read_int(pHandle,I2C_MEM_ADDR_WIDIH_8, addr_ << 1, reg_,buf, len,I2C_TIMEOUT);

    if(status !=  I2C_OK)
    {
      /* wait for the stop flag to be set  */
      i2c_wait_flag(pHandle, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, I2C_TIMEOUT);

      /* clear stop flag */
      i2c_flag_clear(pHandle->i2cx, I2C_STOPF_FLAG);
    }
    else
    {
      /* wait for the communication to end */
      i2c_wait_end(pHandle->i2cx, I2C_TIMEOUT);
    }


    if (status == I2C_ERR_STEP_1) {//busy
        return false;
    }

    if (status != I2C_OK) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    i2c_handle_type *pHandle = &i2cDevice[device].handle;

    if (error) {
        *error = pHandle->error_code;
    }

    if(pHandle->error_code ==I2C_OK){
    	   if (i2c_flag_get(pHandle->i2cx, I2C_BUSYF_FLAG) == SET)
    	   {
    		   return true;
    	   }
    	   return false;
    }


   return true;
}

#endif
