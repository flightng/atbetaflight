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

#define TARGET_BOARD_IDENTIFIER "AT32F437"
#define USBD_PRODUCT_STRING     "EMSRAT32F4"

//buttons
#define USE_BUTTONS
#define BUTTON_A_PIN            PA0
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PC13
#define BUTTON_B_PIN_INVERTED // Active high


#define LED0_PIN                PD13
#define LED1_PIN                PD14
#define LED2_PIN                PD15
#define LED0_INVERTED
#define LED1_INVERTED
#define LED2_INVERTED


#define USE_BEEPER
#define BEEPER_PIN              PB2
//#define BEEPER_INVERTED

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_AUTO
#define DSHOT_BITBANG_DEFAULT   DSHOT_BITBANG_OFF

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            PA4


#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12


#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5


#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN        PA1
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO_1_CS_PIN          SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE    SPI1
#define GYRO_1_ALIGN           CW90_DEG


#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

//#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1



#define USE_MAX7456
#define MAX7456_SPI_INSTANCE   SPI2
#define MAX7456_SPI_CS_PIN     SPI2_NSS_PIN


// ***************ELRS **************************


#define USE_RX_SPI
#define RX_SPI_INSTANCE        SPI3
#define RX_SPI_LED_INVERTED

#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_EXPRESSLRS

#define USE_RX_EXPRESSLRS
#define USE_RX_EXPRESSLRS_TELEMETRY
#define USE_RX_SX1280
#define RX_CHANNELS_AETR
#define RX_SPI_BIND_PIN        PC3
#define RX_NSS_PIN             PA15
#define RX_SPI_LED_PIN         PC12
#define RX_SPI_EXTI_PIN        PC13
#define RX_EXPRESSLRS_SPI_RESET_PIN      PC2
#define RX_EXPRESSLRS_SPI_BUSY_PIN       PA13
#define RX_EXPRESSLRS_TIMER_INSTANCE     TMR5


// *************** Baro **************************
#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8        // SCL pad
#define I2C1_SDA                PB9        // SDA pad
#define BARO_I2C_INSTANCE       (I2CDEV_1)

#define USE_BARO
#define USE_BARO_BMP280
//#define USE_BARO_MS5611

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC15
#define USE_USB_DETECT

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10


#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF
#define SERIALRX_UART           SERIAL_PORT_USART2

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE         ADC1  // Default added
#define ADC1_DMA_OPT            0  // DMA 2 Stream 0 Channel 0

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
