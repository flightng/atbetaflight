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

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"

// Number of bits in I2C protocol phase
#define LEN_ADDR 7
#define LEN_RW 1
#define LEN_ACK 1

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10

// Allow 500us for clock strech to complete during unstick
#define UNSTICK_CLK_STRETCH (500/UNSTICK_CLK_US)

static void i2cUnstick(IO_t scl, IO_t sda);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_MUX , GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_MUX , GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)


const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_MUX_4),
            I2CPINDEF(PB8, GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PB7, GPIO_MUX_4),
            I2CPINDEF(PB9, GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EVT_IRQn,
        .er_irq = I2C1_ERR_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PB10, GPIO_MUX_4),
            I2CPINDEF(PD12, GPIO_MUX_4),
	    I2CPINDEF(PH2,  GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PB3, GPIO_MUX_4),
            I2CPINDEF(PB11, GPIO_MUX_4),
	    I2CPINDEF(PH3,GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EVT_IRQn,
        .er_irq = I2C2_ERR_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = {
            I2CPINDEF(PC0, GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PC1, GPIO_MUX_4),
	    I2CPINDEF(PB14, GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EVT_IRQn,
        .er_irq = I2C3_ERR_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hardware || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable i2c RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    i2cUnstick(scl, sda);

    // Init pins

    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);


    // Init I2C peripheral

    i2c_handle_type  *pHandle = &pDev->handle;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->i2cx = pDev->hardware->reg;

    // Compute TIMINGR value based on peripheral clock for this device instance
    //    uint32_t i2cPclk;

//    pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);
//    pHandle->Init.OwnAddress1 = 0x0;
//    pHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//    pHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//    pHandle->Init.OwnAddress2 = 0x0;
//    pHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//    pHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    /*	at32 clkctrl	可以使用以下默认值
     *   I2Cx_CLKCTRL                   0xB170FFFF   //10K
  	  	 I2Cx_CLKCTRL                   0xC0E06969   //50K
	 	 I2Cx_CLKCTRL                   0x80504C4E   //100K
		 I2Cx_CLKCTRL                   0x30F03C6B   //200K
     */
    crm_clocks_freq_type crm_clk_freq;
    crm_clocks_freq_get (&crm_clk_freq);
    uint32_t i2cPclk=crm_clk_freq.apb1_freq;//at32f43x i2c123 on apb1

    uint32_t I2Cx_CLKCTRL= i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);
//    uint32_t I2Cx_CLKCTRL=0x80504C4E;

//    HAL_I2C_Init(pHandle);
    i2c_reset( pHandle->i2cx);
    /* config i2c */
    i2c_init( pHandle->i2cx, 0x0f, I2Cx_CLKCTRL);

    i2c_own_address1_set( pHandle->i2cx, I2C_ADDRESS_MODE_7BIT, 0x0);


    nvic_irq_enable(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    nvic_irq_enable(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));

    i2c_enable(pHandle->i2cx,TRUE);//i2c_init ctrl1_i2cen =0 ,so enable


}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Clock out, with SDA high:
    //   7 data bits
    //   1 READ bit
    //   1 cycle for the ACK
    for (i = 0; i < (LEN_ADDR + LEN_RW + LEN_ACK); i++) {
        // Wait for any clock stretching to finish
        int timeout = UNSTICK_CLK_STRETCH;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(UNSTICK_CLK_US);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(UNSTICK_CLK_US/2);
        IOHi(scl); // Set bus high
        delayMicroseconds(UNSTICK_CLK_US/2);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOLo(sda);
    delayMicroseconds(UNSTICK_CLK_US/2);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOHi(sda); // Set bus sda high
}

#endif
