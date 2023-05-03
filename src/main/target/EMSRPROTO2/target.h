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

#define TARGET_BOARD_IDENTIFIER "EMSR"
#define USBD_PRODUCT_STRING     "EMSR-PROTO-2"
/**********swd debuger reserved *****************
 *
 * pa13	swdio
 * pa14 swclk
 * PA15	JTDI
 * PB4 JREST
 * pb3 swo /DTO
 *
 * other pin
 *
 * PB2 ->BOOT0 button
 * PA8  MCO1
 * PA12 OTG1 D+ DP
 * PA11 OTG1 D- DN
 * PH0 HEXT IN
 * PH1 HEXT OUT
 */
#define SYSTEM_HSE_MHZ        8
//buttons
#define USE_BUTTONS
#define BUTTON_A_PIN            PD2
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PC13 //PC14 FOR lqfp64
#define BUTTON_B_PIN_INVERTED // Active high


#define LED0_PIN                PD15 //confirm on LQFP64
#define LED1_PIN                PD14 //confirm on LQFP64
#define LED2_PIN                PD13 //confirm on LQFP64




#define USE_BEEPER
#define BEEPER_PIN              PB11
#define BEEPER_INVERTED		 //低电平触发
// #define BEEPER_PWM_HZ	2500 //0  有源bb响 | 2500Hz 无源bb响，PWM驱动

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_AUTO
#define DSHOT_BITBANG_DEFAULT   DSHOT_BITBANG_AUTO

// *************** SPI **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            PA4

//NOT USE ON AT-START BOARD ! -->OTG2
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD1//PB13 on LQFP64
#define SPI2_MISO_PIN           PD3//PB14 on LQFP64
#define SPI2_MOSI_PIN           PD4//PB15 on LQFP64
#define SPI2_NSS_PIN            PD5 //confirm on lqfp64



#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN 			PD6 //confirm on lqfp64

// *************** Gyro & ACC **********************

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN        PA15
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN          SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE    SPI1
#define GYRO_1_ALIGN           CW270_DEG


#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACCGYRO_BMI270
#define USE_ACCGYRO_ASM330LHH
#define USE_ACCGYRO_LSM6DSL
#define USE_ACCGYRO_LSM6DSO
#define USE_ACCGYRO_QMI8658
#define USE_ACCGYRO_SH3001



#define USE_ACC
#define USE_ACC_SPI_ICM42688P


//#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

// *************** OSD *****************************
//USE SPI3 ON AT-START BOARD ,USE SPI2 ON LQFP64
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

/********************BLACKBOX***********************/
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G          // 1Gb NAND flash support
#define USE_FLASH_W25M             // Stacked die support
#define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
#define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
#define FLASH_SPI_INSTANCE      SPI3
#define FLASH_CS_PIN            SPI3_NSS_PIN



// *************** Baro **************************
#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C_DEVICE              (I2CDEV_3)
#define I2C3_SCL                PC0        // SCL pad
#define I2C3_SDA                PC1        // SDA pad
#define USE_I2C_PULLUP

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define BARO_I2C_INSTANCE       (I2CDEV_3)

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define MAG_I2C_INSTANCE        (I2CDEV_3)

//#define USE_BARO_MS5611

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC15 //可以悬空忽略
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC5
#define UART3_TX_PIN            PC4

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART8
#define UART8_RX_PIN            PC3
#define UART8_TX_PIN            PC2

//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       6 // VCP  UART1 UART2 UART3 UART4 UART5

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF
#define SERIALRX_UART           SERIAL_PORT_USART1



// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1  // Default added
#define ADC1_DMA_OPT            11  //DMA 2 CH 5

#define VBAT_ADC_PIN            PB0
#define CURRENT_METER_ADC_PIN   PB1
//#define RSSI_ADC_PIN            0

#define USE_ESCSERIAL

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL |FEATURE_LED_STRIP )
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 28
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(20) )
