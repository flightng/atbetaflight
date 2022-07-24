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

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"


const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
		.dmaResource=NULL //USE_DMA_SPEC
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
		.dmaResource=NULL //USE_DMA_SPEC

    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
		.dmaResource=NULL //USE_DMA_SPEC
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];

//FOR AT32F435/7
#define ADC_CHANNEL_VREFINT	ADC_CHANNEL_18 //adc1_in17 参考电压
#define ADC_CHANNEL_TEMPSENSOR_ADC1	ADC_CHANNEL_16//ADC1_IN16 内部温度传感器



/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
    // Pseudo entries for internal sensor.
    // Keep these at the beginning for easy indexing by ADC_TAG_MAP_{VREFINT,TEMPSENSOR}
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
    // VREFINT is available on all ADC instances except ADC2
    // Here, for simplicity, we force VREFINT on ADC1.
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_VREFINT,         19 },
    // TEMPSENSOR is available on ADC1 or ADC5
    // Here, for simplicity, we force TEMPSENSOR on ADC1.
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_TEMPSENSOR_ADC1, 17 },
#endif
    // Inputs available for all packages under 100 pin or smaller
	//参考 DS p35

    { DEFIO_TAG_E__PA0,  ADC_DEVICES_123,  ADC_CHANNEL_0,   1 },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_123,  ADC_CHANNEL_1,   2 },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_123,  ADC_CHANNEL_2,   3 },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_123,  ADC_CHANNEL_3,   4 },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,   ADC_CHANNEL_4,   5 },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,   ADC_CHANNEL_5,   6 },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,   ADC_CHANNEL_6,   7 },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,   ADC_CHANNEL_7,   8 },

    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,   ADC_CHANNEL_8,   9 },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,   ADC_CHANNEL_9,   10 },

    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123,  ADC_CHANNEL_10,   11 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123,  ADC_CHANNEL_11,   12 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_123,  ADC_CHANNEL_12,   13 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_123,  ADC_CHANNEL_13,   14 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,   ADC_CHANNEL_14,   15 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,   ADC_CHANNEL_15,   16 },

       // Inputs available for packages larger than 100-pin are not listed
};


static void handleError(void)
{
    while (true) {
    }
}

// Note on sampling time.
// Temperature sensor has minimum sample time of 9us.
// With prescaler = 4 at 200MHz (AHB1), fADC = 50MHz (tcycle = 0.02us), 9us = 450cycles < 810
/*
 * adcdev 需要初始化的adc 设备
 * channel count  需要初始化的通道数量
 */
void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{

    // DeInit is done in adcInit().

    //config adc base to init adcx
    /*
     * 4 分频
     * 12位分辨率
     * 数据右对齐
     * 重复序列转换
     * 软件触发
     * dma 持续请求
     * 允许 overrun （覆盖采样数据）
     * 关闭超采样
     */
	adc_base_config_type adc_base_struct;
	adc_base_default_para_init(&adc_base_struct);
	adc_base_struct.sequence_mode = TRUE;
	adc_base_struct.repeat_mode = TRUE;
	adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
	adc_base_struct.ordinary_channel_length = channelCount;
	adc_base_config(adcdev->ADCx, &adc_base_struct);
	adc_resolution_set(adcdev->ADCx, ADC_RESOLUTION_12B);
}

int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}
//不需要。stm32 需要取特定寄存器的值，用于校准计算温度
//void adcInitCalibrationValues(void)
//{
//    adcVREFINTCAL = *(uint16_t *)VREFINT_CAL_ADDR;
//    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR;
//    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR;
//    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
//}

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

//volatile DMA_RAM_R uint16_t adcConversionBuffer[ADC_CHANNEL_COUNT] __attribute__((aligned(32)));
volatile DMA_RAM_R uint32_t adcConversionBuffer[ADC_CHANNEL_COUNT] __attribute__((aligned(32)));


void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
    }

//#ifdef USE_ADC_INTERNAL
//    adcInitCalibrationValues();
//#endif


    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (i == ADC_TEMPSENSOR) {
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = ADCDEV_1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = ADCDEV_1;
        } else {
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            // Found a tag map entry for this input pin
            // Find an ADC device that can handle this input pin

            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                if (!adcDevice[dev].ADCx
#ifndef USE_DMA_SPEC
                     || !adcDevice[dev].dmaResource
#endif
                   ) {
                    // Instance not activated
                    continue;
                }
                if (adcTagMap[map].devices & (1 << dev)) {
                    // Found an activated ADC instance for this input pin
                    break;
                }

				if (dev == ADCDEV_COUNT) {
					// No valid device found, go next channel.
					continue;
				}
            }//end for find adcx
        }//end of else

	// At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i
	adcOperatingConfig[i].adcDevice = dev;
	adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
	adcOperatingConfig[i].sampleTime = ADC_SAMPLING_INTERVAL_5CYCLES;
	adcOperatingConfig[i].enabled = true;

	adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

	// Configure a pin for ADC
	if (adcOperatingConfig[i].tag) {
		IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
		IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG,GPIO_DRIVE_STRENGTH_MODERATE, 0, GPIO_PULL_NONE));
    	}//end of ioinit
    //前面已经进行了整个adc 的重启，无需单个重启
    }//end of for i in each channel  config the gpio

    int  dmaBufferIndex = 0;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        //RESET ALL ADC
		adc_reset();

		//init adc common
		 adc_common_config_type adc_common_struct;
		 adc_common_default_para_init(&adc_common_struct);
		/* config combine mode */
		adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
		/* config division,adcclk is division by hclk */
		adc_common_struct.div = ADC_HCLK_DIV_4;
		/* config common dma mode,it's not useful in independent mode */
		adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
		/* config common dma request repeat */
		adc_common_struct.common_dma_request_repeat_state = FALSE;
		/* config adjacent adc sampling interval,it's useful for ordinary shifting mode */
		adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
		/* config inner temperature sensor and vintrv */
		adc_common_struct.tempervintrv_state = TRUE;
		adc_common_struct.vbat_state = TRUE;
		/* config voltage battery */
		adc_common_config(&adc_common_struct);

        int configuredAdcChannels = BITCOUNT(adc->channelBits);

        // Configure ADCx with inputs
// init base 配置
        adcInitDevice(adc, configuredAdcChannels);

#ifdef USE_DMA_SPEC
        // Configure DMA for this ADC peripheral
        //config the dma for each adc device
	const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
		dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
		if (!dmaSpec || !dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
			return;
		}
		// Deinitialize  & Initialize the DMA for new transfer

		// dmaEnable must be called before calling HAL_DMA_Init,
		// to enable clock for associated DMA if not already done so.
		dmaEnable(dmaIdentifier);
		xDMA_DeInit(dmaSpec->ref);

		dma_init_type dma_init_struct;
		dma_default_para_init(&dma_init_struct);
		dma_init_struct.buffer_size = BITCOUNT(adc->channelBits);
		dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
		dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
		dma_init_struct.memory_inc_enable = TRUE;
		dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
		dma_init_struct.peripheral_inc_enable = FALSE;
		dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
		dma_init_struct.loop_mode_enable = FALSE;

		dma_init_struct.memory_base_addr = (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);
		dma_init_struct.peripheral_base_addr = (uint32_t)&(adc->ADCx->odt);


		xDMA_Init(dmaSpec->ref, &dma_init_struct);
		dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

		/* enable dma transfer complete interrupt */
		xDMA_ITConfig(dmaSpec->ref,DMA_IT_TCIF,ENABLE);
		xDMA_Cmd(dmaSpec->ref,ENABLE);

		// Start conversion in DMA mode ,使能ADC DMA 重复采样模式，之后自动采样到内存
		/* config dma mode,it's not useful when common dma mode is use */
		adc_dma_mode_enable(adc->ADCx, TRUE);

		/* config dma request repeat,it's not useful when common dma mode is use */
		adc_dma_request_repeat_enable(adc->ADCx, TRUE);
#endif //end of USE_DMA_SPEC
        // Configure channels
//init  each  channel 逐个通道配置
        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            adc_ordinary_channel_set(adcDevice[dev].ADCx,
            		adcOperatingConfig[adcChan].adcChannel,
					adcChan,
					ADC_SAMPLETIME_47_5);
        }//end of for each channel
    // Start channels.
    // This must be done after channel configuration is complete, as HAL_ADC_ConfigChannel
    // throws an error when configuring internal channels if ADC1 or ADC2 are already enabled.

        /* enable adc overflow interrupt */
//        adc_interrupt_enable(adc->ADCx, ADC_OCCO_INT, TRUE);

        /* adc enable */
        adc_enable(adc->ADCx, TRUE);
        while(adc_flag_get(adc->ADCx, ADC_RDY_FLAG) == RESET);

        dmaBufferIndex += BITCOUNT(adc->channelBits);
        //start calibration
        adc_calibration_init(adc->ADCx);
        while(adc_calibration_init_status_get(adc->ADCx));
        adc_calibration_start(adc->ADCx);
        while(adc_calibration_status_get(adc->ADCx));

        //start adc
        adc_ordinary_software_trigger_enable(adc->ADCx, TRUE);

    }//end of for
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    // Cache coherency should be maintained by MPU facility

    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

#ifdef USE_ADC_INTERNAL

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

uint16_t adcInternalRead(int channel)
{
    int dmaIndex = adcOperatingConfig[channel].dmaIndex;

    return adcConversionBuffer[dmaIndex];
}

int adcPrivateVref = -1;
int adcPrivateTemp = -1;

uint16_t adcInternalReadVrefint(void)
{
    uint16_t value = adcInternalRead(ADC_VREFINT);
    adcPrivateVref =((double)ADC_VREF * 4 * value) / 4095;
    return adcPrivateVref;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
    adcPrivateTemp = ((ADC_TEMP_BASE-value *ADC_VREF/4096)/ADC_TEMP_SLOPE+25);
    return adcPrivateTemp;
}
#endif // USE_ADC_INTERNAL
#endif // USE_ADC
