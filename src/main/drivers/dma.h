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

#include "drivers/resource.h"
#include "platform.h"

#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

typedef struct dmaResource_s dmaResource_t;

#if defined(STM32F4) || defined(STM32F7)
#define DMA_ARCH_TYPE DMA_Stream_TypeDef
#elif defined(STM32H7)
// H7 has stream based DMA and channel based BDMA, but we ignore BDMA (for now).
#define DMA_ARCH_TYPE DMA_Stream_TypeDef
#elif defined(AT32F43x)
//at32 has dma1 dma2 and edma ,ignore edma(for now)
#define DMA_ARCH_TYPE dma_channel_type
#else
#define DMA_ARCH_TYPE DMA_Channel_TypeDef
#endif

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
	dma_type*                   dma;
    dmaResource_t               *ref;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
    dmamux_channel_type			*dmamux;
} dmaChannelDescriptor_t;

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

#if defined(AT32F43x)

typedef enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA2_CH6_HANDLER,
    DMA2_CH7_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH7_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 7) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 7) + 1)

uint32_t dmaGetChannel(const uint8_t channel);

#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, c, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _CHANNEL ## c ##_BASE, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## c ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 ,\
	.dmamux= (dmamux_channel_type *) d ## MUX_CHANNEL ##c \
    }

#if defined(USE_CCM_CODE) && defined(STM32F3)
#define DMA_HANDLER_CODE CCM_CODE
#else
#define DMA_HANDLER_CODE
#endif


#define DEFINE_DMA_IRQ_HANDLER(d, c, i) DMA_HANDLER_CODE void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                        if (handler) \
                                                                            handler(&dmaDescriptors[index]); \
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->clr = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->sts & (flag << d->flagsShift))

/*
(#) DMA_IT_FEIFx  : specifies the interrupt source for the  FIFO Mode Transfer Error event.
(#) DMA_IT_DMEIFx : specifies the interrupt source for the Direct Mode Transfer Error event.
(#) DMA_IT_TEIFx  : specifies the interrupt source for the Transfer Error event.
(#) DMA_IT_HTIFx  : specifies the interrupt source for the Half-Transfer Complete event.
(#) DMA_IT_TCIFx  : specifies the interrupt source for the a Transfer Complete event.
*/
#define DMA_IT_GLOB         ((uint32_t)0x00000001) // channel global interput flag
#define DMA_IT_TCIF         ((uint32_t)0x00000002) // channel full transport flag
#define DMA_IT_HTIF         ((uint32_t)0x00000004) // channel half transport flag
#define DMA_IT_TEIF         ((uint32_t)0x00000008) // channel transport error flag

#define DMA_IT_DMEIF        ((uint32_t)0x00000004) // at32 has no direct mode transfer mode


// Macros to avoid direct register and register bit access

#define DMA_CCR_EN 1 // Not defined anywhere ...
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->ctrl_bit.chen & DMA_CCR_EN)

//这个和具体的端口结合宏定义，不应该在dma里实现
//#define DMAx_SetMemoryAddress(reg, address) ((DMA_ARCH_TYPE *)(reg))->CMAR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail]


dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaEnable(dmaIdentifier_e identifier);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);
const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);
uint32_t dmaGetChannel(const uint8_t channel);

#if defined(AT32F43x)
void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId);
#endif


//
// Wrapper macros to cast dmaResource_t back into DMA_ARCH_TYPE
//

#define xDMA_Init(dmaResource, initStruct) dma_init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) dma_reset((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState)  dma_channel_enable((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState)	dma_interrupt_enable((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) dma_data_number_get((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) dma_data_number_set((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) dma_flag_get((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) dma_flag_clear((DMA_ARCH_TYPE *)(dmaResource), flags)

//serial 的具体宏定义，不在这实现
//#define xDMA_MemoryTargetConfig(dmaResource, address, target) DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target)

#endif
