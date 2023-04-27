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
//https://www.arterychip.com/download/DS/DS_AT32F435_437_V2.02-EN.pdf
#pragma once

#define TARGET_BOARD_IDENTIFIER "BHER"
#define USBD_PRODUCT_STRING     "BETAFPVF435"

#define LED0_PIN                PB5

#define USE_BEEPER
#define BEEPER_PIN              PB4
#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_AUTO
#define DSHOT_BITBANG_DEFAULT   DSHOT_BITBANG_AUTO

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACCGYRO_BMI270
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_LSM6DSO
#define USE_ACC_SPI_ICM42688P

// *************** Baro **************************
#define USE_BARO
#define USE_BARO_SPI_DPS310
#define USE_BARO_SPI_BMP280
#define DEFAULT_BARO_SPI_BMP280
#define BARO_SPI_INSTANCE		SPI3
#define BARO_CS_PIN				PB3

// *************** BLACKBOX **************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G          // 1Gb NAND flash support
#define USE_FLASH_W25M             // Stacked die support
#define USE_FLASH_W25M512          // 512Kb (256Kb x 2 stacked) NOR flash support
#define USE_FLASH_W25M02G          // 2Gb (1Gb x 2 stacked) NAND flash support
#define FLASH_SPI_INSTANCE      SPI2
#define FLASH_CS_PIN            PB12

// *************** OSD *****************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI3
#define MAX7456_SPI_CS_PIN      PA15

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3      PC9

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       6

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF
#define SERIALRX_UART           SERIAL_PORT_USART3

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            11

#define VBAT_ADC_PIN            PC2
#define CURRENT_METER_ADC_PIN   PC1

#define USE_ESCSERIAL

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_LED_STRIP)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define VBAT_SCALE_DEFAULT          110
#define CURRENT_METER_SCALE_DEFAULT 800

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 28
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) )
