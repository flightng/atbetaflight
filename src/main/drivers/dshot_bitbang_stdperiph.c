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

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT_BITBANG

#include "build/atomic.h"
#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h" // XXX for pwmOutputPort_t motors[]; should go away with refactoring
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;
    /* 初始化对应pin的 modemask、input模式值，output模式寄存器值，供下面switchtoinput、switchtooutput函数用
     * 好处： 直接寄存器操作对应的pin 在输入和输出模式之间快速翻转
     * 原因：dshot 是fc 和esc 之间1根线通信，需要在输入和输出之间快速切换模式
     * 移植思路：
     * f3\f4之后芯片， 输入输出mode分离为多个寄存器，但是在f1、at32f4仍然使用CRL CRH 两个寄存器进行初始设置，并且ipd\ipu 模式，
     * 需要单独写一下BSRR\BRR
     * 因此需要考虑参考GPIO_Init 函数的写法，简化
     * GPIO_CRL_MODE0 +PullUP PULLDown 设计一个输入模式，一个输出模式，不能直接用GPIO_Mode_input\output
     * 按照针脚进行移位 ，输入模式、输出模式按照针脚模式进行设置
     * bbGpioSetup中先设置对应pin的 mask位，然后在模式切换函数中，直接WRITE_REG操作
     * 先写CRL CRH 寄存器，然后判断 是否有IPD、IPU，然后单独写BSRR\BRR
     */
#if defined(AT32F4)//:FIXME MAY BE ERROR
    bbPort->gpioModeMask |= (GPIO_CRL_MODE0 << (pinIndex * 2));//对
    bbPort->gpioBRR |= 0x01 << (pinIndex *2);
    if(pinIndex <8)
    {//TODO: 需要自己做一遍运算，检查是否正确
        bbPort->gpioModeInput |= (GPIO_Mode_IPD << (pinIndex * 2));//错
        bbPort->gpioModeOutput |= (GPIO_Mode_Out_PP << (pinIndex * 2));//错
    	bbPort->gpioModeInputH |= 0x44;
    	bbPort->gpioModeOutputH |=0x44;
    }else{
        bbPort->gpioModeInputH |= (GPIO_Mode_IPD << (pinIndex * 2));//错
           bbPort->gpioModeOutputH |= (GPIO_Mode_Out_PP << (pinIndex * 2));//错
       	bbPort->gpioModeInput |= 0x44;
       	bbPort->gpioModeOutput |=0x44;
    }

#else
    bbPort->gpioModeMask |= (GPIO_MODER_MODER0 << (pinIndex * 2));
    bbPort->gpioModeInput |= (GPIO_Mode_IN << (pinIndex * 2));
    bbPort->gpioModeOutput |= (GPIO_Mode_OUT << (pinIndex * 2));
#endif



#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbPort->gpioIdleBSRR |= (1 << pinIndex);         // BS (lower half)
    } else
#endif
    {
        bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        IOWrite(bbMotor->io, 1);
    } else
#endif
    {
        IOWrite(bbMotor->io, 0);
    }

#if defined(STM32F4)
    IOConfigGPIO(bbMotor->io, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, bbPuPdMode));
#elif defined(AT32F4)
    IOConfigGPIO(bbMotor->io, IO_CONFIG(GPIO_Mode_AF_PP, GPIO_Speed_2MHz));
#else
#error MCU dependent code required
#endif
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    TIM_Cmd(bbPort->timhw->tim, DISABLE);

    timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);
    // timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
#if defined(AT32F4)
        IOConfigGPIO(io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
#endif
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        TIM_CtrlPWMOutputs(timhw->tim, ENABLE);
    }
#endif

    // Enable and keep it running

    TIM_Cmd(bbPort->timhw->tim, ENABLE);
}

#ifdef USE_DMA_REGISTER_CACHE

void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
#if defined(AT32F4)
    ((DMA_ARCH_TYPE *)dmaResource)->CCR = dmaRegCache->CCR;
    ((DMA_ARCH_TYPE *)dmaResource)->CNDTR = dmaRegCache->CNDTR;
    ((DMA_ARCH_TYPE *)dmaResource)->CPAR = dmaRegCache->CPAR;
    ((DMA_ARCH_TYPE *)dmaResource)->CMAR = dmaRegCache->CMAR;
#else
    ((DMA_Stream_TypeDef *)dmaResource)->CR = dmaRegCache->CR;
    ((DMA_Stream_TypeDef *)dmaResource)->FCR = dmaRegCache->FCR;
    ((DMA_Stream_TypeDef *)dmaResource)->NDTR = dmaRegCache->NDTR;
    ((DMA_Stream_TypeDef *)dmaResource)->PAR = dmaRegCache->PAR;
    ((DMA_Stream_TypeDef *)dmaResource)->M0AR = dmaRegCache->M0AR;
#endif
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
#if defined(AT32F4)
	dmaRegCache->CCR=((DMA_ARCH_TYPE *)dmaResource)->CCR;
	dmaRegCache->CNDTR=((DMA_ARCH_TYPE *)dmaResource)->CNDTR;
	dmaRegCache->CPAR=((DMA_ARCH_TYPE *)dmaResource)->CPAR ;
	dmaRegCache->CMAR=((DMA_ARCH_TYPE *)dmaResource)->CMAR ;
#else
    dmaRegCache->CR = ((DMA_Stream_TypeDef *)dmaResource)->CR;
    dmaRegCache->FCR = ((DMA_Stream_TypeDef *)dmaResource)->FCR;
    dmaRegCache->NDTR = ((DMA_Stream_TypeDef *)dmaResource)->NDTR;
    dmaRegCache->PAR = ((DMA_Stream_TypeDef *)dmaResource)->PAR;
    dmaRegCache->M0AR = ((DMA_Stream_TypeDef *)dmaResource)->M0AR;
#endif
}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    dbgPinHi(1);
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)
#if defined(AT32F4)
    //bsrr 设置odr 的值， brr reset odr值 to pull down
    WRITE_REG(bbPort->gpio->BSRR, bbPort->gpioIdleBSRR);
    //FIXME:一次性设置GPIO口的读写标识,f1\at32f4 需要修改 CRH CRL 两个寄存器，需要看下怎么操作
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
    	//set CRL
        MODIFY_REG(bbPort->gpio->CRL, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    	//set CRH
        MODIFY_REG(bbPort->gpio->CRH, bbPort->gpioModeMask, bbPort->gpioModeOutputH);

    }

#else
    WRITE_REG(bbPort->gpio->BSRRL, bbPort->gpioIdleBSRR);
    // Set GPIO to output
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }
#endif
    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for output

    bbPort->timhw->tim->ARR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set GPIO to input
#if defined(AT32F4)
    //set CRH
    //set CRL
    //set BRR to PULL DOWN
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->CRL, bbPort->gpioModeMask, bbPort->gpioModeOutput);
       	//set CRH
        MODIFY_REG(bbPort->gpio->CRH, bbPort->gpioModeMask, bbPort->gpioModeOutputH);
        //FIXME: 需要计算1次
        bbPort->gpio->BRR |= bbPort->gpioBRR;
        }
#else
//f3\f4\f7\h7 else
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }
#endif
    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for input

    bbPort->timhw->tim->CNT = 0;
    bbPort->timhw->tim->ARR = bbPort->inputARR;

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    dbgPinLo(1);
}
#endif
#ifndef AT32F4
void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DMA_StructInit(dmainit);

    dmainit->DMA_Mode = DMA_Mode_Normal;
    dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;

    dmainit->DMA_Channel = bbPort->dmaChannel;
    dmainit->DMA_FIFOMode = DMA_FIFOMode_Enable ;
    dmainit->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    dmainit->DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    dmainit->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->DMA_Priority = DMA_Priority_High;
        dmainit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dmainit->DMA_BufferSize = bbPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSRRL;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralToMemory;
        dmainit->DMA_BufferSize = bbPort->portInputCount;

        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->IDR;

        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portInputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}
#else
void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DMA_StructInit(dmainit);

    dmainit->DMA_Mode = DMA_Mode_Normal;
    dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;
    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->DMA_Priority = DMA_Priority_High;
        dmainit->DMA_DIR = DMA_DIR_PeripheralDST;
        dmainit->DMA_BufferSize = bbPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSRR;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_MemoryBaseAddr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralSRC;
        dmainit->DMA_BufferSize = bbPort->portInputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->IDR;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dmainit->DMA_MemoryBaseAddr = (uint32_t)bbPort->portInputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

#endif//AT32F4

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    TIM_TimeBaseInitTypeDef *init = &bbPort->timeBaseInit;

    init->TIM_Prescaler = 0; // Feed raw timerClock
    init->TIM_ClockDivision = TIM_CKD_DIV1;
    init->TIM_CounterMode = TIM_CounterMode_Up;
    init->TIM_Period = period;
    TIM_TimeBaseInit(bbPort->timhw->tim, init);
    TIM_ARRPreloadConfig(bbPort->timhw->tim, ENABLE);
}

void bbTIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    TIM_DMACmd(TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TC, ENABLE);
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    xDMA_Cmd(bbPort->dmaResource, NewState);
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}

#endif // USE_DSHOT_BB
