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

#ifdef USE_SPI

// STM32F405 can't DMA to/from FASTRAM (CCM SRAM)
#define IS_CCM(p) (((uint32_t)p & 0xffff0000) == 0x10000000)

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

static spi_init_type defaultInit = {
//    .SPI_Mode = SPI_Mode_Master,
//    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
//    .SPI_DataSize = SPI_DataSize_8b,
//    .SPI_NSS = SPI_NSS_Soft,
//    .SPI_FirstBit = SPI_FirstBit_MSB,
//    .SPI_CRCPolynomial = 7,
//    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
//    .SPI_CPOL = SPI_CPOL_High,
//    .SPI_CPHA = SPI_CPHA_2Edge
		//fixme: at32f43x的spi默认初始化，需要自己单独设置crc polynomial using spi_crc_polynomial_set
	  .master_slave_mode = SPI_MODE_MASTER,
	  .transmission_mode = SPI_TRANSMIT_FULL_DUPLEX,
	  .frame_bit_num = SPI_FRAME_8BIT,
	  .cs_mode_selection = SPI_CS_SOFTWARE_MODE,
	  .first_bit_transmission = SPI_FIRST_BIT_MSB,
	  .mclk_freq_division = SPI_MCLK_DIV_8,
	  .clock_polarity = SPI_CLOCK_POLARITY_HIGH,
	  .clock_phase = SPI_CLOCK_PHASE_2EDGE };

//分频数转换为 BR ，分频配置bit， at32f437 对应ctrl1 寄存器 bit 5:3 mdiv[2:0]  ctrl2 bit8 mdiv[3]
//与stm32F4 略增加 512分频和1024分频(mdiv[3]=1)
static uint16_t spiDivisorToBRbits(spi_type  *instance, uint16_t divisor)
{
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
#if defined(STM32F410xx) || defined(STM32F411xE)
    UNUSED(instance);
#else
    if (instance == SPI2 || instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#endif

    divisor = constrain(divisor, 2, 256);

    return (ffs(divisor) - 2) << 3; // SPI_CR1_BR_Pos
}
//bug: 不支持1000 512 1001 1024分频
static void spiSetDivisorBRreg(spi_type *instance, uint16_t divisor)
{
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
    const uint16_t tempRegister = (instance->ctrl1 & ~BR_BITS);
    instance->ctrl1 = tempRegister | spiDivisorToBRbits(instance, divisor);
#undef BR_BITS
}


void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_SCK_CFG_HIGH, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

    // Init SPI hardware
//    SPI_I2S_DeInit(spi->dev);
    spi_i2s_reset(spi->dev);

//    SPI_I2S_DMACmd(spi->dev, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    spi_i2s_dma_transmitter_enable(spi->dev,TRUE);
    spi_i2s_dma_receiver_enable(spi->dev,TRUE);

//    SPI_Init(spi->dev, &defaultInit);
    spi_init(spi->dev,&defaultInit);
    //补充设置crc，其实复位值就是7 ，还是设置一下吧
    spi_crc_polynomial_set(spi->dev,7);

//    SPI_Cmd(spi->dev, ENABLE);
    spi_enable(spi->dev,TRUE);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    dma_init_type *initTx = bus->initTx;

    dma_default_para_init(initTx);
//    initTx->DMA_Channel = bus->dmaTx->channel;
//    initTx->DMA_DIR = DMA_DIR_MemoryToPeripheral;
//    initTx->DMA_Mode = DMA_Mode_Normal;
//    initTx->DMA_PeripheralBaseAddr = (uint32_t)&bus->busType_u.spi.instance->DR;
//    initTx->DMA_Priority = DMA_Priority_Low;
//    initTx->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    initTx->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    initTx->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    initTx->direction=DMA_DIR_MEMORY_TO_PERIPHERAL;
    initTx->loop_mode_enable=FALSE;
    initTx->peripheral_base_addr=(uint32_t)&bus->busType_u.spi.instance->dt ;
	initTx->priority =DMA_PRIORITY_LOW;
	initTx->peripheral_inc_enable =FALSE;
    initTx->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
	initTx->memory_data_width =DMA_MEMORY_DATA_WIDTH_BYTE;

    if (bus->dmaRx) {
        dma_init_type *initRx = bus->initRx;

        dma_default_para_init(initRx);

        initRx->direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
//        initRx->DMA_Channel = bus->dmaRx->channel;
        initRx->loop_mode_enable = FALSE;
        initRx->peripheral_base_addr = (uint32_t)&bus->busType_u.spi.instance->dt;
        initRx->priority = DMA_PRIORITY_LOW;
        initRx->peripheral_inc_enable = FALSE;
        initRx->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;

    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
	DMA_ARCH_TYPE *streamRegs = (DMA_ARCH_TYPE *)descriptor->ref;

    // Disable the stream
//    streamRegs->CR = 0U;
	// xDMA_DeInit(streamRegs);
    xDMA_Cmd(streamRegs,DISABLE);
    // Clear any pending interrupt flags
    DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
}

static bool spiInternalReadWriteBufPolled(spi_type *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    uint8_t b;

    while (len--) {
    	/*
    	 * 逻辑理解：
    	 * SPI_I2S_FLAG_TXE Transmit buffer empty flag 当发送缓存为非空时，一直等待，直到为空
    	 * SPI_I2S_FLAG_RXNE Receive buffer not empty flag 当接收缓存区不慢时，一直等待直到缓存为满
    	 */
        b = txData ? *(txData++) : 0xFF;
//        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET);
//        SPI_I2S_SendData(instance, b);

        while(spi_i2s_flag_get(instance,SPI_I2S_TDBE_FLAG)==RESET);
        spi_i2s_data_transmit(instance,b);

//        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET);
//        b = SPI_I2S_ReceiveData(instance);

        while(spi_i2s_flag_get(instance,SPI_I2S_RDBF_FLAG)==RESET);
        b=spi_i2s_data_receive(instance);

        if (rxData) {
            *(rxData++) = b;
        }
    }

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    volatile busSegment_t *segment = bus->curSegment;

    if (preInit) {
        // Prepare the init structure for the next segment to reduce inter-segment interval
        segment++;
        if(segment->len == 0) {
            // There's no following segment
            return;
        }
    }

    int len = segment->len;

    uint8_t *txData = segment->u.buffers.txData;
    dma_init_type  *initTx = bus->initTx;

    if (txData) {
//        initTx->DMA_Memory0BaseAddr = (uint32_t)txData;
//        initTx->DMA_MemoryInc = DMA_MemoryInc_Enable;
    	initTx->memory_base_addr = (uint32_t)txData;
    	initTx->memory_inc_enable =TRUE;
    } else {
        dummyTxByte = 0xff;//默认启动字节
//        initTx->DMA_Memory0BaseAddr = (uint32_t)&dummyTxByte;
//        initTx->DMA_MemoryInc = DMA_MemoryInc_Disable;
        initTx->memory_base_addr = (uint32_t)&dummyTxByte;
        initTx->memory_inc_enable =FALSE;
    }
//    initTx->DMA_BufferSize = len;
    initTx->buffer_size =len;

    if (dev->bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        dma_init_type *initRx = bus->initRx;

        if (rxData) {
//        	initRx->DMA_Memory0BaseAddr = (uint32_t)rxData;
//            initRx->DMA_MemoryInc = DMA_MemoryInc_Enable;
        	initRx->memory_base_addr= (uint32_t)rxData;
        	initRx->memory_inc_enable=TRUE;
        } else {
        	initRx->memory_base_addr = (uint32_t)&dummyRxByte;
            initRx->memory_inc_enable = FALSE;
        }
        // If possible use 16 bit memory writes to prevent atomic access issues on gyro data

        if ((initRx->memory_base_addr & 0x1) || (len & 0x1)) {
            initRx->memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
        } else {
            initRx->memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        }
        initRx->buffer_size = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    // Assert Chip Select
    IOLo(dev->busType_u.spi.csnPin);

    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    DMA_ARCH_TYPE *streamRegsTx = (DMA_ARCH_TYPE *)dmaTx->ref;
    if (dmaRx) {
    	DMA_ARCH_TYPE *streamRegsRx = (DMA_ARCH_TYPE *)dmaRx->ref;

        // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable streams to enable update
//        streamRegsTx->CR = 0U;
//        streamRegsRx->CR = 0U;

        xDMA_Cmd(streamRegsTx, DISABLE);
        xDMA_Cmd(streamRegsRx, DISABLE);


        /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
         * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
         */
        xDMA_ITConfig(streamRegsRx, DMA_IT_TCIF, ENABLE);

        // Update streams
        xDMA_Init(streamRegsTx, dev->bus->initTx);
        xDMA_Init(streamRegsRx, dev->bus->initRx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable streams
        xDMA_Cmd(streamRegsTx, ENABLE);
        xDMA_Cmd(streamRegsRx, ENABLE);

        //fixme: ENABLE DMAMUX ? should check dmamuxen regs in debug

        /* Enable the SPI DMA Tx & Rx requests */
//        SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
        spi_i2s_dma_transmitter_enable(dev->bus->busType_u.spi.instance,TRUE);
		spi_i2s_dma_receiver_enable(dev->bus->busType_u.spi.instance,TRUE);

    } else {
        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        // Clear transfer flags
        DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

        // Disable stream to enable update
//        xDMA_DeInit(streamRegsTx);
        xDMA_Cmd(streamRegsTx, DISABLE);


        xDMA_ITConfig(streamRegsTx, DMA_IT_TCIF, ENABLE);

        // Update stream
        xDMA_Init(streamRegsTx, dev->bus->initTx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable stream
        xDMA_Cmd(streamRegsTx, ENABLE);

        /* Enable the SPI DMA Tx request */
//        SPI_I2S_DMACmd(dev->bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx, ENABLE);
        spi_i2s_dma_transmitter_enable(dev->bus->busType_u.spi.instance,TRUE);

    }
}


void spiInternalStopDMA (const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    spi_type *instance = dev->bus->busType_u.spi.instance;
    DMA_ARCH_TYPE *streamRegsTx = (DMA_ARCH_TYPE *)dmaTx->ref;

    if (dmaRx) {
    	DMA_ARCH_TYPE *streamRegsRx = (DMA_ARCH_TYPE *)dmaRx->ref;

        // Disable streams
//	    xDMA_DeInit(streamRegsTx);
//	    xDMA_DeInit(streamRegsRx);
    	xDMA_Cmd(streamRegsTx,DISABLE);
    	xDMA_Cmd(streamRegsRx,DISABLE);

    	DMA_CLEAR_FLAG(dmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    	DMA_CLEAR_FLAG(dmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);



//        SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
        spi_i2s_dma_transmitter_enable(instance,FALSE);
		spi_i2s_dma_receiver_enable(instance,FALSE);

    } else {
        // Ensure the current transmission is complete
//        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY));
    	while(spi_i2s_flag_get(instance,SPI_I2S_BF_FLAG));

        // Drain the RX buffer
        // while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE)) {
        while(spi_i2s_flag_get(instance,SPI_I2S_RDBF_FLAG)){
            // instance->DR;
            instance->dt;
        }

        // Disable stream
    	xDMA_Cmd(streamRegsTx,DISABLE);


        // SPI_I2S_DMACmd(instance, SPI_I2S_DMAReq_Tx, DISABLE);
        spi_i2s_dma_transmitter_enable(instance,FALSE);
    }
}

// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    spi_type *instance = bus->busType_u.spi.instance;
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;

    // SPI_Cmd(instance, DISABLE);
    spi_enable(instance,FALSE);

    // Switch bus speed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        spiSetDivisorBRreg(bus->busType_u.spi.instance, dev->busType_u.spi.speed);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        // Switch SPI clock polarity/phase
        // instance->CR1 &= ~(SPI_CPOL_High | SPI_CPHA_2Edge);
        //fixme: 这里是否是漏写了？

        // Apply setting
        if (dev->busType_u.spi.leadingEdge) {
            // instance->CR1 |= SPI_CPOL_Low | SPI_CPHA_1Edge;
            instance->ctrl1_bit.clkpol = SPI_CLOCK_POLARITY_LOW;
            instance->ctrl1_bit.clkpha = SPI_CLOCK_PHASE_1EDGE;
        } else
        {
            // instance->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
            instance->ctrl1_bit.clkpol = SPI_CLOCK_POLARITY_HIGH;
            instance->ctrl1_bit.clkpha = SPI_CLOCK_PHASE_2EDGE;
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    // SPI_Cmd(instance, ENABLE);
    spi_enable(instance,TRUE);

    // Check that any there are no attempts to DMA to/from CCD SRAM
    for (busSegment_t *checkSegment = bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if (((checkSegment->u.buffers.rxData) && (IS_CCM(checkSegment->u.buffers.rxData) || (bus->dmaRx == (dmaChannelDescriptor_t *)NULL))) ||
            ((checkSegment->u.buffers.txData) && IS_CCM(checkSegment->u.buffers.txData))) {
            dmaSafe = false;
            break;
        }
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }
    // Use DMA if possible
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) || (xferLen > 8))) {
        // Intialise the init structures for the first transfer
        spiInternalInitStream(dev, false);

        // Start the transfers
        spiInternalStartDMA(dev);
    } else {
        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            // Assert Chip Select
            IOLo(dev->busType_u.spi.csnPin);

            spiInternalReadWriteBufPolled(
                    bus->busType_u.spi.instance,
                    bus->curSegment->u.buffers.txData,
                    bus->curSegment->u.buffers.rxData,
                    bus->curSegment->len);

            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            if (bus->curSegment->callback) {
                switch(bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    bus->curSegment--;
                    break;

                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                    return;

                case BUS_READY:
                default:
                    // Advance to the next DMA segment
                    break;
                }
            }
            bus->curSegment++;
        }

        // If a following transaction has been linked, start it
        if (bus->curSegment->u.link.dev) {
            const extDevice_t *nextDev = bus->curSegment->u.link.dev;
            busSegment_t *nextSegments = bus->curSegment->u.link.segments;
            busSegment_t *endSegment = bus->curSegment;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    }
}
#endif
