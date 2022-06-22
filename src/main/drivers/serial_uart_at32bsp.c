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
 * Initialization part of serial_uart.c using at32 bsp driver
 */

/*
 * Authors:
 * emsr ports the code to at32f435/7
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/dma.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort) {
#if !defined(USE_INVERTER) && !defined(STM32F303xC)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

#ifdef USE_INVERTER
    if (inverted) {
        // Enable hardware inverter if available.
        enableInverter(uartPort->USARTx, TRUE);
    }
#endif
#endif
}

void uartReconfigure(uartPort_t *uartPort)
{
//    USART_InitTypeDef USART_InitStructure;
//    USART_Cmd(uartPort->USARTx, DISABLE);
//    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;

    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    // This seems to cause RX to break on STM32F1, see https://github.com/betaflight/betaflight/pull/1654
//    if ( (uartPort->port.options & SERIAL_PARITY_EVEN)) {
//        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
//    } else {
//        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    }

//    USART_InitStructure.USART_StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_StopBits_2 : USART_StopBits_1;
//    USART_InitStructure.USART_Parity   = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;
//
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = 0;
//    if (uartPort->port.mode & MODE_RX)
//        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
//    if (uartPort->port.mode & MODE_TX)
//        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
//
//    USART_Init(uartPort->USARTx, &USART_InitStructure);
// void usart_init(usart_type* usart_x, uint32_t baud_rate, usart_data_bit_num_type data_bit, usart_stop_bit_num_type stop_bit)

	usart_enable(uartPort->USARTx,DISABLE);
	//init
	usart_init(uartPort->USARTx,
			   uartPort->port.baudRate,
			  (uartPort->port.options & SERIAL_PARITY_EVEN)? USART_DATA_9BITS:USART_DATA_8BITS,
			  (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOP_2_BIT : USART_STOP_1_BIT);

	//set parity
	usart_parity_selection_config(uartPort->USARTx,
			(uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE);

	//set hardware control
	usart_hardware_flow_control_set(uartPort->USARTx,USART_HARDWARE_FLOW_NONE);

	//set mode rx or tx
	if (uartPort->port.mode & MODE_RX)
		usart_receiver_enable(uartPort->USARTx,TRUE);
	if (uartPort->port.mode & MODE_TX)
		usart_transmitter_enable(uartPort->USARTx,TRUE);

	//config pin swap
    usartConfigurePinInversion(uartPort);

    if (uartPort->port.options & SERIAL_BIDIR)
//        USART_HalfDuplexCmd(uartPort->USARTx, ENABLE);
    	usart_single_line_halfduplex_select(uartPort->USARTx, TRUE);
    else
        usart_single_line_halfduplex_select(uartPort->USARTx, FALSE);
//enable usart
	usart_enable(uartPort->USARTx,TRUE);

    // Receive DMA or IRQ
    dma_init_type DMA_InitStructure;
    if (uartPort->port.mode & MODE_RX) {
        if (uartPort->rxDMAResource) {

            dma_default_para_init(&DMA_InitStructure);
            DMA_InitStructure.loop_mode_enable=TRUE;
            DMA_InitStructure.peripheral_base_addr=uartPort->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.priority  = DMA_PRIORITY_MEDIUM;
            DMA_InitStructure.peripheral_inc_enable =FALSE;
            DMA_InitStructure.peripheral_data_width =DMA_PERIPHERAL_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_inc_enable =TRUE;
            DMA_InitStructure.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
            DMA_InitStructure.memory_base_addr=(uint32_t)uartPort->port.rxBuffer;
            DMA_InitStructure.buffer_size = uartPort->port.rxBufferSize;
            DMA_InitStructure.direction= DMA_DIR_PERIPHERAL_TO_MEMORY;

            xDMA_DeInit(uartPort->rxDMAResource);
            xDMA_Init(uartPort->rxDMAResource, &DMA_InitStructure);
            xDMA_Cmd(uartPort->rxDMAResource, ENABLE);
//            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Rx, ENABLE);
            usart_dma_receiver_enable(uartPort->USARTx,TRUE);
            uartPort->rxDMAPos = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
        } else {
//            USART_ClearITPendingBit(uartPort->USARTx, USART_IT_RXNE);
//            USART_ITConfig(uartPort->USARTx, USART_IT_RXNE, ENABLE);
//            USART_ITConfig(uartPort->USARTx, USART_IT_IDLE, ENABLE);
        	usart_flag_clear(uartPort->USARTx,USART_RDBF_FLAG);
        	usart_interrupt_enable(uartPort->USARTx,USART_RDBF_INT,TRUE);
        	usart_interrupt_enable(uartPort->USARTx,USART_IDLE_INT,TRUE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
        if (uartPort->txDMAResource) {
//            DMA_StructInit(&DMA_InitStructure);
//            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//            DMA_InitStructure.DMA_PeripheralBaseAddr = uartPort->txDMAPeripheralBaseAddr;
//            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//            DMA_InitStructure.DMA_BufferSize = uartPort->port.txBufferSize;
//            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;

			dma_default_para_init(&DMA_InitStructure);
			DMA_InitStructure.loop_mode_enable=FALSE;
			DMA_InitStructure.peripheral_base_addr=uartPort->txDMAPeripheralBaseAddr;
			DMA_InitStructure.priority  = DMA_PRIORITY_MEDIUM;
			DMA_InitStructure.peripheral_inc_enable =FALSE;
			DMA_InitStructure.peripheral_data_width =DMA_PERIPHERAL_DATA_WIDTH_BYTE;
			DMA_InitStructure.memory_inc_enable =TRUE;
			DMA_InitStructure.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
			DMA_InitStructure.memory_base_addr=(uint32_t)uartPort->port.txBuffer;
			DMA_InitStructure.buffer_size = uartPort->port.txBufferSize;
			DMA_InitStructure.direction= DMA_DIR_MEMORY_TO_PERIPHERAL;

            xDMA_DeInit(uartPort->txDMAResource);
            xDMA_Init(uartPort->txDMAResource, &DMA_InitStructure);

#ifdef STM32F4
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TC | DMA_IT_FE | DMA_IT_TE | DMA_IT_DME, ENABLE);
#else
            xDMA_ITConfig(uartPort->txDMAResource, DMA_IT_TCIF, ENABLE);
#endif

            xDMA_SetCurrDataCounter(uartPort->txDMAResource, 0);
//            USART_DMACmd(uartPort->USARTx, USART_DMAReq_Tx, ENABLE);
            usart_dma_transmitter_enable(uartPort->USARTx,TRUE);

        } else {
//            USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
        	usart_interrupt_enable(uartPort->USARTx,USART_TDBE_INT,TRUE);
        }
    }

//    USART_Cmd(uartPort->USARTx, ENABLE);
    usart_enable(uartPort->USARTx,TRUE);
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).

    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        // For F4 (and F1), there are cases that NDTR (CNDTR for F1) is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.
        // F3 is not confirmed to be vulnerable, but not excluded as a safety.

        if (xDMA_GetCurrDataCounter(s->txDMAResource)) {
            // Possible premature TC case.
            goto reenable;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit.
            s->txDMAEmpty = true;
            return;
        }

        // Start a new transaction.

//#ifdef STM32F4
//        xDMA_MemoryTargetConfig(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail], DMA_Memory_0);
//#else
//        DMAx_SetMemoryAddress(s->txDMAResource, (uint32_t)&s->port.txBuffer[s->port.txBufferTail]);
//#endif
        ((DMA_ARCH_TYPE*)s->txDMAResource) -> maddr =(uint32_t)&s->port.txBuffer[s->port.txBufferTail];

        if (s->port.txBufferHead > s->port.txBufferTail) {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferHead - s->port.txBufferTail);
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            xDMA_SetCurrDataCounter(s->txDMAResource, s->port.txBufferSize - s->port.txBufferTail);
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

    reenable:
        xDMA_Cmd(s->txDMAResource, ENABLE);
    }
}
#endif


static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        handleUsartTxDma(s);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }

}

void uartIrqHandler(uartPort_t *s)
{
	//rx 非空
    if (!s->rxDMAResource && (usart_flag_get(s->USARTx, USART_RDBF_FLAG) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->dt, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->dt;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    //tx 缓存空
    if (!s->txDMAResource && (usart_flag_get(s->USARTx, USART_TDBE_FLAG) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
        	usart_data_transmit(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
        	usart_interrupt_enable(s->USARTx, USART_TDBE_INT, DISABLE);
        }
    }

    //OverRun Error interrupt  这里可能有错误
    if (usart_flag_get(s->USARTx, USART_ROERR_FLAG) == SET) {
    	usart_flag_clear(s->USARTx, USART_ROERR_FLAG);
    }
//空闲
    if (usart_flag_get(s->USARTx, USART_IDLEF_FLAG) == SET) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        // clear
        (void) s->USARTx->sts;
        (void) s->USARTx->dt;
    }
}
#endif // USE_UART
