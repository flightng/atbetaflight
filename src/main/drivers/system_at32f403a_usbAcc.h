/*
 * system_at32f403a_usbAcc.h
 *
 *  Created on: 2022Äê2ÔÂ25ÈÕ
 *      Author: user
 */

#ifndef MAIN_DRIVERS_SYSTEM_AT32F403A_USBACC_H_
#define MAIN_DRIVERS_SYSTEM_AT32F403A_USBACC_H_

#include <platform.h>

#define     __IO    volatile             /*!< Defines 'read / write' permissions */


#define MAKE_VALUE(reg_offset, bit_num)  (uint32_t)(((reg_offset) << 16) | (bit_num & 0x1F))
#define REG8(addr)                       *(volatile uint8_t *)(addr)
#define REG16(addr)                      *(volatile uint16_t *)(addr)
#define REG32(addr)                      *(volatile uint32_t *)(addr)


#define PERIPH_BASE                      ((uint32_t)0x40000000)
#define APB2PERIPH_BASE                  (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE                   (PERIPH_BASE + 0x20000)
#define ACC_BASE                         (APB2PERIPH_BASE + 0x5800)
#define ACC                             ((acc_type *) ACC_BASE)
#define CRM_BASE                         (AHBPERIPH_BASE + 0x1000)

#define PERIPH_REG(periph_base, value)   REG32((periph_base + (value >> 16)))
#define PERIPH_REG_BIT(value)            (0x1U << (value & 0x1F))

#define CRM_REG(value)                   PERIPH_REG(CRM_BASE, value)
#define CRM_REG_BIT(value)               PERIPH_REG_BIT(value)

/**
  * @brief confirm state
  */
typedef enum {FALSE = 0, TRUE = !FALSE} confirm_state;



//set usb 48mhz using hsi
typedef struct
{

  /**
    * @brief acc sts register, offset:0x00
    */
  union
  {
    __IO uint32_t sts;
    struct
    {
      __IO uint32_t calrdy               : 1; /* [0] */
      __IO uint32_t rslost               : 1; /* [1] */
      __IO uint32_t reserved1            : 30;/* [31:2] */
    } sts_bit;
  };

  /**
    * @brief acc ctrl1 register, offset:0x04
    */
  union
  {
    __IO uint32_t ctrl1;
    struct
    {
      __IO uint32_t calon                : 1; /* [0] */
      __IO uint32_t entrim               : 1; /* [1] */
      __IO uint32_t reserved1            : 2; /* [3:2] */
      __IO uint32_t eien                 : 1; /* [4] */
      __IO uint32_t calrdyien            : 1; /* [5] */
      __IO uint32_t reserved2            : 2; /* [7:6] */
      __IO uint32_t step                 : 4; /* [11:8] */
      __IO uint32_t reserved3            : 20;/* [31:12] */
    } ctrl1_bit;
  };

   /**
    * @brief acc ctrl2 register, offset:0x08
    */
  union
  {
    __IO uint32_t ctrl2;
    struct
    {
      __IO uint32_t hickcal              : 8; /* [7:0] */
      __IO uint32_t hicktrim             : 6; /* [13:8] */
      __IO uint32_t reserved1            : 18;/* [31:14] */
    } ctrl2_bit;
  };

  /**
  * @brief acc acc_c1 register, offset:0x0C
  */
  union
  {
    __IO uint32_t c1;
    struct
    {
      __IO uint32_t c1                   : 16;/* [15:0] */
      __IO uint32_t reserved1            : 16;/* [31:16] */
    } c1_bit;
  };

  /**
  * @brief acc acc_c2 register, offset:0x10
  */
  union
  {
    __IO uint32_t c2;
    struct
    {
      __IO uint32_t c2                   : 16;/* [15:0] */
      __IO uint32_t reserved1            : 16;/* [31:16] */
    } c2_bit;
  };

  /**
  * @brief acc acc_c3 register, offset:0x14
  */
  union
  {
    __IO uint32_t c3;
    struct
    {
      __IO uint32_t c3                   : 16;/* [15:0] */
      __IO uint32_t reserved1            : 16;/* [31:16] */
    } c3_bit;
  };
} acc_type;

/**
  * @brief crm periph clock
  */
typedef enum
{
  /* ahb periph */
  CRM_DMA1_PERIPH_CLOCK                  = MAKE_VALUE(0x14, 0),  /*!< dma1 periph clock */
  CRM_DMA2_PERIPH_CLOCK                  = MAKE_VALUE(0x14, 1),  /*!< dma2 periph clock */
  CRM_CRC_PERIPH_CLOCK                   = MAKE_VALUE(0x14, 6),  /*!< crc periph clock */
  CRM_XMC_PERIPH_CLOCK                   = MAKE_VALUE(0x14, 8),  /*!< xmc periph clock */
  CRM_SDIO1_PERIPH_CLOCK                 = MAKE_VALUE(0x14, 10), /*!< sdio1 periph clock */
  CRM_SDIO2_PERIPH_CLOCK                 = MAKE_VALUE(0x14, 11), /*!< sdio2 periph clock */
  /* apb2 periph */
  CRM_IOMUX_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 0),  /*!< iomux periph clock */
  CRM_GPIOA_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 2),  /*!< gpioa periph clock */
  CRM_GPIOB_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 3),  /*!< gpiob periph clock */
  CRM_GPIOC_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 4),  /*!< gpioc periph clock */
  CRM_GPIOD_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 5),  /*!< gpiod periph clock */
  CRM_GPIOE_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 6),  /*!< gpioe periph clock */
  CRM_ADC1_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 9),  /*!< adc1 periph clock */
  CRM_ADC2_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 10), /*!< adc2 periph clock */
  CRM_TMR1_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 11), /*!< tmr1 periph clock */
  CRM_SPI1_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 12), /*!< spi1 periph clock */
  CRM_TMR8_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 13), /*!< tmr8 periph clock */
  CRM_USART1_PERIPH_CLOCK                = MAKE_VALUE(0x18, 14), /*!< usart1 periph clock */
  CRM_ADC3_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 15), /*!< adc3 periph clock */
  CRM_TMR9_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 19), /*!< tmr9 periph clock */
  CRM_TMR10_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 20), /*!< tmr10 periph clock */
  CRM_TMR11_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 21), /*!< tmr11 periph clock */
  CRM_ACC_PERIPH_CLOCK                   = MAKE_VALUE(0x18, 22), /*!< acc periph clock */
  CRM_I2C3_PERIPH_CLOCK                  = MAKE_VALUE(0x18, 23), /*!< i2c3 periph clock */
  CRM_USART6_PERIPH_CLOCK                = MAKE_VALUE(0x18, 24), /*!< usart6 periph clock */
  CRM_UART7_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 25), /*!< uart7 periph clock */
  CRM_UART8_PERIPH_CLOCK                 = MAKE_VALUE(0x18, 26), /*!< uart8 periph clock */
  /* apb1 periph */
  CRM_TMR2_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 0),  /*!< tmr2 periph clock */
  CRM_TMR3_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 1),  /*!< tmr3 periph clock */
  CRM_TMR4_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 2),  /*!< tmr4 periph clock */
  CRM_TMR5_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 3),  /*!< tmr5 periph clock */
  CRM_TMR6_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 4),  /*!< tmr6 periph clock */
  CRM_TMR7_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 5),  /*!< tmr7 periph clock */
  CRM_TMR12_PERIPH_CLOCK                 = MAKE_VALUE(0x1C, 6),  /*!< tmr12 periph clock */
  CRM_TMR13_PERIPH_CLOCK                 = MAKE_VALUE(0x1C, 7),  /*!< tmr13 periph clock */
  CRM_TMR14_PERIPH_CLOCK                 = MAKE_VALUE(0x1C, 8),  /*!< tmr14 periph clock */
  CRM_WWDT_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 11), /*!< wwdt periph clock */
  CRM_SPI2_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 14), /*!< spi2 periph clock */
  CRM_SPI3_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 15), /*!< spi3 periph clock */
  CRM_SPI4_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 16), /*!< spi4 periph clock */
  CRM_USART2_PERIPH_CLOCK                = MAKE_VALUE(0x1C, 17), /*!< usart2 periph clock */
  CRM_USART3_PERIPH_CLOCK                = MAKE_VALUE(0x1C, 18), /*!< usart3 periph clock */
  CRM_UART4_PERIPH_CLOCK                 = MAKE_VALUE(0x1C, 19), /*!< uart4 periph clock */
  CRM_UART5_PERIPH_CLOCK                 = MAKE_VALUE(0x1C, 20), /*!< uart5 periph clock */
  CRM_I2C1_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 21), /*!< i2c1 periph clock */
  CRM_I2C2_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 22), /*!< i2c2 periph clock */
  CRM_USB_PERIPH_CLOCK                   = MAKE_VALUE(0x1C, 23), /*!< usb periph clock */
  CRM_CAN1_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 25), /*!< can1 periph clock */
  CRM_CAN2_PERIPH_CLOCK                  = MAKE_VALUE(0x1C, 26), /*!< can2 periph clock */
  CRM_BPR_PERIPH_CLOCK                   = MAKE_VALUE(0x1C, 27), /*!< bpr periph clock */
  CRM_PWC_PERIPH_CLOCK                   = MAKE_VALUE(0x1C, 28), /*!< pwc periph clock */
  CRM_DAC_PERIPH_CLOCK                   = MAKE_VALUE(0x1C, 29)  /*!< dac periph clock */

} crm_periph_clock_type;

typedef enum
{
  CRM_HICK48_DIV6                        = 0x00, /*!< high speed internal clock (48 mhz) div6 */
  CRM_HICK48_NODIV                       = 0x01  /*!< high speed internal clock (48 mhz) no div */
} crm_hick_div_6_type;


/**
  * @brief crm usb 48 mhz clock source select
  */
typedef enum
{
  CRM_USB_CLOCK_SOURCE_PLL               = 0x00, /*!< select phase locking loop clock as usb clock source */
  CRM_USB_CLOCK_SOURCE_HICK              = 0x01  /*!< select high speed internal clock as usb clock source */
} crm_usb_clock_source_type;

/**
  * @brief crm hick as system clock frequency select
  */
typedef enum
{
  CRM_HICK_SCLK_8MHZ                     = 0x00, /*!< fixed 8 mhz when hick is selected as sclk */
  CRM_HICK_SCLK_48MHZ                    = 0x01  /*!< 8 mhz or 48 mhz depend on hickdiv when hick is selected as sclk */
} crm_hick_sclk_frequency_type;
/**
  * @brief type define crm register all
  */
typedef struct
{
  /**
    * @brief crm ctrl register, offset:0x00
    */
  union
  {
    __IO uint32_t ctrl;
    struct
    {
      __IO uint32_t hicken               : 1; /* [0] */
      __IO uint32_t hickstbl             : 1; /* [1] */
      __IO uint32_t hicktrim             : 6; /* [7:2] */
      __IO uint32_t hickcal              : 8; /* [15:8] */
      __IO uint32_t hexten               : 1; /* [16] */
      __IO uint32_t hextstbl             : 1; /* [17] */
      __IO uint32_t hextbyps             : 1; /* [18] */
      __IO uint32_t cfden                : 1; /* [19] */
      __IO uint32_t reserved1            : 4; /* [23:20] */
      __IO uint32_t pllen                : 1; /* [24] */
      __IO uint32_t pllstbl              : 1; /* [25] */
      __IO uint32_t reserved2            : 6; /* [31:26] */
    } ctrl_bit;
  };

  /**
    * @brief crm cfg register, offset:0x04
    */
  union
  {
    __IO uint32_t cfg;
    struct
    {
      __IO uint32_t sclksel              : 2; /* [1:0] */
      __IO uint32_t sclksts              : 2; /* [3:2] */
      __IO uint32_t ahbdiv               : 4; /* [7:4] */
      __IO uint32_t apb1div              : 3; /* [10:8] */
      __IO uint32_t apb2div              : 3; /* [13:11] */
      __IO uint32_t adcdiv_l             : 2; /* [15:14] */
      __IO uint32_t pllrcs               : 1; /* [16] */
      __IO uint32_t pllhextdiv           : 1; /* [17] */
      __IO uint32_t pllmult_l            : 4; /* [21:18] */
      __IO uint32_t usbdiv_l             : 2; /* [23:22] */
      __IO uint32_t clkout_sel           : 3; /* [26:24] */
      __IO uint32_t usbdiv_h             : 1; /* [27] */
      __IO uint32_t adcdiv_h             : 1; /* [28] */
      __IO uint32_t pllmult_h            : 2; /* [30:29] */
      __IO uint32_t pllrange             : 1; /* [31] */
    } cfg_bit;
  };

  /**
    * @brief crm clkint register, offset:0x08
    */
  union
  {
    __IO uint32_t clkint;
    struct
    {
      __IO uint32_t lickstblf            : 1; /* [0] */
      __IO uint32_t lextstblf            : 1; /* [1] */
      __IO uint32_t hickstblf            : 1; /* [2] */
      __IO uint32_t hextstblf            : 1; /* [3] */
      __IO uint32_t pllstblf             : 1; /* [4] */
      __IO uint32_t reserved1            : 2; /* [6:5] */
      __IO uint32_t cfdf                 : 1; /* [7] */
      __IO uint32_t lickstblien          : 1; /* [8] */
      __IO uint32_t lextstblien          : 1; /* [9] */
      __IO uint32_t hickstblien          : 1; /* [10] */
      __IO uint32_t hextstblien          : 1; /* [11] */
      __IO uint32_t pllstblien           : 1; /* [12] */
      __IO uint32_t reserved2            : 3; /* [15:13] */
      __IO uint32_t lickstblfc           : 1; /* [16] */
      __IO uint32_t lextstblfc           : 1; /* [17] */
      __IO uint32_t hickstblfc           : 1; /* [18] */
      __IO uint32_t hextstblfc           : 1; /* [19] */
      __IO uint32_t pllstblfc            : 1; /* [20] */
      __IO uint32_t reserved3            : 2; /* [22:21] */
      __IO uint32_t cfdfc                : 1; /* [23] */
      __IO uint32_t reserved4            : 8; /* [31:24] */
    } clkint_bit;
  };

  /**
    * @brief crm apb2rst register, offset:0x0C
    */
  union
  {
    __IO uint32_t apb2rst;
    struct
    {
      __IO uint32_t iomuxrst             : 1; /* [0] */
      __IO uint32_t exintrst             : 1; /* [1] */
      __IO uint32_t gpioarst             : 1; /* [2] */
      __IO uint32_t gpiobrst             : 1; /* [3] */
      __IO uint32_t gpiocrst             : 1; /* [4] */
      __IO uint32_t gpiodrst             : 1; /* [5] */
      __IO uint32_t gpioerst             : 1; /* [6] */
      __IO uint32_t reserved1            : 2; /* [8:7] */
      __IO uint32_t adc1rst              : 1; /* [9] */
      __IO uint32_t adc2rst              : 1; /* [10] */
      __IO uint32_t tmr1rst              : 1; /* [11] */
      __IO uint32_t spi1rst              : 1; /* [12] */
      __IO uint32_t tmr8rst              : 1; /* [13] */
      __IO uint32_t usart1rst            : 1; /* [14] */
      __IO uint32_t adc3rst              : 1; /* [15] */
      __IO uint32_t reserved2            : 3; /* [18:16] */
      __IO uint32_t tmr9rst              : 1; /* [19] */
      __IO uint32_t tmr10rst             : 1; /* [20] */
      __IO uint32_t tmr11rst             : 1; /* [21] */
      __IO uint32_t accrst               : 1; /* [22] */
      __IO uint32_t i2c3rst              : 1; /* [23] */
      __IO uint32_t usart6rst            : 1; /* [24] */
      __IO uint32_t uart7rst             : 1; /* [25] */
      __IO uint32_t uart8rst             : 1; /* [26] */
      __IO uint32_t reserved3            : 5; /* [31:27] */
    } apb2rst_bit;
  };

  /**
    * @brief crm apb1rst register, offset:0x10
    */
  union
  {
    __IO uint32_t apb1rst;
    struct
    {
      __IO uint32_t tmr2rst              : 1; /* [0] */
      __IO uint32_t tmr3rst              : 1; /* [1] */
      __IO uint32_t tmr4rst              : 1; /* [2] */
      __IO uint32_t tmr5rst              : 1; /* [3] */
      __IO uint32_t tmr6rst              : 1; /* [4] */
      __IO uint32_t tmr7rst              : 1; /* [5] */
      __IO uint32_t tmr12rst             : 1; /* [6] */
      __IO uint32_t tmr13rst             : 1; /* [7] */
      __IO uint32_t tmr14rst             : 1; /* [8] */
      __IO uint32_t reserved1            : 2; /* [10:9] */
      __IO uint32_t wwdtrst              : 1; /* [11] */
      __IO uint32_t reserved2            : 2; /* [13:12] */
      __IO uint32_t spi2rst              : 1; /* [14] */
      __IO uint32_t spi3rst              : 1; /* [15] */
      __IO uint32_t spi4rst              : 1; /* [16] */
      __IO uint32_t usart2rst            : 1; /* [17] */
      __IO uint32_t usart3rst            : 1; /* [18] */
      __IO uint32_t uart4rst             : 1; /* [19] */
      __IO uint32_t uart5rst             : 1; /* [20] */
      __IO uint32_t i2c1rst              : 1; /* [21] */
      __IO uint32_t i2c2rst              : 1; /* [22] */
      __IO uint32_t usbrst               : 1; /* [23] */
      __IO uint32_t reserved3            : 1; /* [24] */
      __IO uint32_t can1rst              : 1; /* [25] */
      __IO uint32_t can2rst              : 1; /* [26] */
      __IO uint32_t bprrst               : 1; /* [27] */
      __IO uint32_t pwcrst               : 1; /* [28] */
      __IO uint32_t dacrst               : 1; /* [29] */
      __IO uint32_t reserved4            : 2; /* [31:30] */
    } apb1rst_bit;
  };

  /**
    * @brief crm ahben register, offset:0x14
    */
  union
  {
    __IO uint32_t ahben;
     struct
    {
      __IO uint32_t dma1en               : 1; /* [0] */
      __IO uint32_t dma2en               : 1; /* [1] */
      __IO uint32_t sramen               : 1; /* [2] */
      __IO uint32_t reserved1            : 1; /* [3] */
      __IO uint32_t flashen              : 1; /* [4] */
      __IO uint32_t reserved2            : 1; /* [5] */
      __IO uint32_t crcen                : 1; /* [6] */
      __IO uint32_t reserved3            : 1; /* [7] */
      __IO uint32_t xmcen                : 1; /* [8] */
      __IO uint32_t reserved4            : 1; /* [9] */
      __IO uint32_t sdio1en              : 1; /* [10] */
      __IO uint32_t sdio2en              : 1; /* [11] */
      __IO uint32_t reserved5            : 20;/* [31:12] */
    } ahben_bit;
  };

  /**
    * @brief crm apb2en register, offset:0x18
    */
  union
  {
    __IO uint32_t apb2en;
    struct
    {
      __IO uint32_t iomuxen              : 1; /* [0] */
      __IO uint32_t reserved1            : 1; /* [1] */
      __IO uint32_t gpioaen              : 1; /* [2] */
      __IO uint32_t gpioben              : 1; /* [3] */
      __IO uint32_t gpiocen              : 1; /* [4] */
      __IO uint32_t gpioden              : 1; /* [5] */
      __IO uint32_t gpioeen              : 1; /* [6] */
      __IO uint32_t reserved2            : 2; /* [8:7] */
      __IO uint32_t adc1en               : 1; /* [9] */
      __IO uint32_t adc2en               : 1; /* [10] */
      __IO uint32_t tmr1en               : 1; /* [11] */
      __IO uint32_t spi1en               : 1; /* [12] */
      __IO uint32_t tmr8en               : 1; /* [13] */
      __IO uint32_t usart1en             : 1; /* [14] */
      __IO uint32_t adc3en               : 1; /* [15] */
      __IO uint32_t reserved3            : 3; /* [18:16] */
      __IO uint32_t tmr9en               : 1; /* [19] */
      __IO uint32_t tmr10en              : 1; /* [20] */
      __IO uint32_t tmr11en              : 1; /* [21] */
      __IO uint32_t accen                : 1; /* [22] */
      __IO uint32_t i2c3en               : 1; /* [23] */
      __IO uint32_t usart6en             : 1; /* [24] */
      __IO uint32_t uart7en              : 1; /* [25] */
      __IO uint32_t uart8en              : 1; /* [26] */
      __IO uint32_t reserved4            : 5; /* [31:27] */
    } apb2en_bit;
  };

  /**
    * @brief crm apb1en register, offset:0x1C
    */
  union
  {
    __IO uint32_t apb1en;
    struct
    {
      __IO uint32_t tmr2en               : 1; /* [0] */
      __IO uint32_t tmr3en               : 1; /* [1] */
      __IO uint32_t tmr4en               : 1; /* [2] */
      __IO uint32_t tmr5en               : 1; /* [3] */
      __IO uint32_t tmr6en               : 1; /* [4] */
      __IO uint32_t tmr7en               : 1; /* [5] */
      __IO uint32_t tmr12en              : 1; /* [6] */
      __IO uint32_t tmr13en              : 1; /* [7] */
      __IO uint32_t tmr14en              : 1; /* [8] */
      __IO uint32_t reserved1            : 2; /* [10:9] */
      __IO uint32_t wwdten               : 1; /* [11] */
      __IO uint32_t reserved2            : 2; /* [13:12] */
      __IO uint32_t spi2en               : 1; /* [14] */
      __IO uint32_t spi3en               : 1; /* [15] */
      __IO uint32_t spi4en               : 1; /* [16] */
      __IO uint32_t usart2en             : 1; /* [17] */
      __IO uint32_t usart3en             : 1; /* [18] */
      __IO uint32_t uart4en              : 1; /* [19] */
      __IO uint32_t uart5en              : 1; /* [20] */
      __IO uint32_t i2c1en               : 1; /* [21] */
      __IO uint32_t i2c2en               : 1; /* [22] */
      __IO uint32_t usben                : 1; /* [23] */
      __IO uint32_t reserved3            : 1; /* [24] */
      __IO uint32_t can1en               : 1; /* [25] */
      __IO uint32_t can2en               : 1; /* [26] */
      __IO uint32_t bpren                : 1; /* [27] */
      __IO uint32_t pwcen                : 1; /* [28] */
      __IO uint32_t dacen                : 1; /* [29] */
      __IO uint32_t reserved4            : 2; /* [31:30] */
    } apb1en_bit;
  };

  /**
    * @brief crm bpdc register, offset:0x20
    */
  union
  {
    __IO uint32_t bpdc;
    struct
    {
      __IO uint32_t lexten               : 1; /* [0] */
      __IO uint32_t lextstbl             : 1; /* [1] */
      __IO uint32_t lextbyps             : 1; /* [2] */
      __IO uint32_t reserved1            : 5; /* [7:3] */
      __IO uint32_t rtcsel               : 2; /* [9:8] */
      __IO uint32_t reserved2            : 5; /* [14:10] */
      __IO uint32_t rtcen                : 1; /* [15] */
      __IO uint32_t bpdrst               : 1; /* [16] */
      __IO uint32_t reserved3            : 15;/* [31:17] */
    } bpdc_bit;
  };

  /**
    * @brief crm ctrlsts register, offset:0x24
    */
  union
  {
    __IO uint32_t ctrlsts;
    struct
    {
      __IO uint32_t licken               : 1; /* [0] */
      __IO uint32_t lickstbl             : 1; /* [1] */
      __IO uint32_t reserved1            : 22;/* [23:2] */
      __IO uint32_t rstfc                : 1; /* [24] */
      __IO uint32_t reserved2            : 1; /* [25] */
      __IO uint32_t nrstf                : 1; /* [26] */
      __IO uint32_t porrstf              : 1; /* [27] */
      __IO uint32_t swrstf               : 1; /* [28] */
      __IO uint32_t wdtrstf              : 1; /* [29] */
      __IO uint32_t wwdtrstf             : 1; /* [30] */
      __IO uint32_t lprstf               : 1; /* [31] */
    } ctrlsts_bit;
  };

  /**
    * @brief crm ahbrst register, offset:0x28
    */
  union
  {
    __IO uint32_t ahbrst;
  };

  /**
    * @brief crm reserved1 register, offset:0x2C
    */
  __IO uint32_t reserved1;

  /**
    * @brief crm misc1 register, offset:0x30
    */
  union
  {
    __IO uint32_t misc1;
    struct
    {
      __IO uint32_t hickcal_key          : 8; /* [7:0] */
      __IO uint32_t reserved1            : 8; /* [15:8] */
      __IO uint32_t clkout_sel           : 1; /* [16] */
      __IO uint32_t reserved2            : 7; /* [23:17] */
      __IO uint32_t usbbufs              : 1; /* [24] */
      __IO uint32_t hickdiv              : 1; /* [25] */
      __IO uint32_t reserved3            : 2; /* [27:26] */
      __IO uint32_t clkoutdiv            : 4; /* [31:28] */
    } misc1_bit;
  };

  /**
    * @brief crm reserved2 register, offset:0x4C~0x34
    */
  __IO uint32_t reserved2[7];

  /**
    * @brief crm misc2 register, offset:0x50
    */
  union
  {
    __IO uint32_t misc2;
    struct
    {
      __IO uint32_t reserved1            : 16;/* [15:0] */
      __IO uint32_t clk_to_tmr           : 1; /* [16] */
      __IO uint32_t reserved2            : 15;/* [31:17] */
    } misc2_bit;
  };

  /**
    * @brief crm misc3 register, offset:0x54
    */
  union
  {
    __IO uint32_t misc3;
    struct
    {
      __IO uint32_t reserved1            : 4; /* [3:0] */
      __IO uint32_t auto_step_en         : 2; /* [5:4] */
      __IO uint32_t reserved2            : 2; /* [7:6] */
      __IO uint32_t hick_to_usb          : 1; /* [8] */
      __IO uint32_t hick_to_sclk         : 1; /* [9] */
      __IO uint32_t reserved3            : 2; /* [11:10] */
      __IO uint32_t hextdiv              : 2; /* [13:12] */
      __IO uint32_t reserved4            : 1; /* [14] */
      __IO uint32_t emac_pps_sel         : 1; /* [15] */
      __IO uint32_t reserved5            : 16;/* [31:16] */
    } misc3_bit;
  };

  /**
    * @brief crm reserved3 register, offset:0x58
    */
  __IO uint32_t reserved3;

  /**
    * @brief crm intmap register, offset:0x5C
    */
  union
  {
    __IO uint32_t intmap;
    struct
    {
      __IO uint32_t usbintmap            : 1; /* [0] */
      __IO uint32_t reserved1            : 31;/* [31:1] */
    } intmap_bit;
  };

} crm_type;


#define CRM                              ((crm_type *) CRM_BASE)



void crm_hick_divider_select(crm_hick_div_6_type value)
{
  CRM->misc1_bit.hickdiv = value;
}

/**
  * @brief  hick as system clock frequency select
  * @param  value
  *         this parameter can be one of the following values:
  *         - CRM_HICK_SCLK_8MHZ
  *         - CRM_HICK_SCLK_48MHZ
  * @retval none
  */
void crm_hick_sclk_frequency_select(crm_hick_sclk_frequency_type value)
{
  crm_hick_divider_select(CRM_HICK48_NODIV);
  CRM->misc3_bit.hick_to_sclk = value;
}



/**
  * @brief  usb 48 mhz clock source select
  * @param  value
  *         this parameter can be one of the following values:
  *         - CRM_USB_CLOCK_SOURCE_PLL
  *         - CRM_USB_CLOCK_SOURCE_HICK
  * @retval none
  */
void crm_usb_clock_source_select(crm_usb_clock_source_type value)
{
  if(value == CRM_USB_CLOCK_SOURCE_HICK)
  {
    crm_hick_sclk_frequency_select(CRM_HICK_SCLK_48MHZ);
  }
  CRM->misc3_bit.hick_to_usb = value;
}



/**
  * @brief  enable or disable the peripheral clock
  * @param  value
  *         this parameter can be one of the following values:
  *         - CRM_DMA1_PERIPH_CLOCK         - CRM_DMA2_PERIPH_CLOCK         - CRM_CRC_PERIPH_CLOCK         - CRM_XMC_PERIPH_CLOCK
  *         - CRM_SDIO1_PERIPH_CLOCK        - CRM_SDIO2_PERIPH_CLOCK        - CRM_EMAC_PERIPH_CLOCK        - CRM_EMACTX_PERIPH_CLOCK
  *         - CRM_EMACRX_PERIPH_CLOCK       - CRM_EMACPTP_PERIPH_CLOCK      - CRM_IOMUX_PERIPH_CLOCK       - CRM_GPIOA_PERIPH_CLOCK
  *         - CRM_GPIOB_PERIPH_CLOCK        - CRM_GPIOC_PERIPH_CLOCK        - CRM_GPIOD_PERIPH_CLOCK       - CRM_GPIOE_PERIPH_CLOCK
  *         - CRM_ADC1_PERIPH_CLOCK         - CRM_ADC2_PERIPH_CLOCK         - CRM_TMR1_PERIPH_CLOCK        - CRM_SPI1_PERIPH_CLOCK
  *         - CRM_TMR8_PERIPH_CLOCK         - CRM_USART1_PERIPH_CLOCK       - CRM_ADC3_PERIPH_CLOCK        - CRM_TMR9_PERIPH_CLOCK
  *         - CRM_TMR10_PERIPH_CLOCK        - CRM_TMR11_PERIPH_CLOCK        - CRM_ACC_PERIPH_CLOCK         - CRM_I2C3_PERIPH_CLOCK
  *         - CRM_USART6_PERIPH_CLOCK       - CRM_UART7_PERIPH_CLOCK        - CRM_UART8_PERIPH_CLOCK       - CRM_TMR2_PERIPH_CLOCK
  *         - CRM_TMR3_PERIPH_CLOCK         - CRM_TMR4_PERIPH_CLOCK         - CRM_TMR5_PERIPH_CLOCK        - CRM_TMR6_PERIPH_CLOCK
  *         - CRM_TMR7_PERIPH_CLOCK         - CRM_TMR12_PERIPH_CLOCK        - CRM_TMR13_PERIPH_CLOCK       - CRM_TMR14_PERIPH_CLOCK
  *         - CRM_WWDT_PERIPH_CLOCK         - CRM_SPI2_PERIPH_CLOCK         - CRM_SPI3_PERIPH_CLOCK        - CRM_SPI4_PERIPH_CLOCK
  *         - CRM_USART2_PERIPH_CLOCK       - CRM_USART3_PERIPH_CLOCK       - CRM_UART4_PERIPH_CLOCK       - CRM_UART5_PERIPH_CLOCK
  *         - CRM_I2C1_PERIPH_CLOCK         - CRM_I2C2_PERIPH_CLOCK         - CRM_USB_PERIPH_CLOCK         - CRM_CAN1_PERIPH_CLOCK
  *         - CRM_CAN2_PERIPH_CLOCK         - CRM_BPR_PERIPH_CLOCK          - CRM_PWC_PERIPH_CLOCK         - CRM_DAC_PERIPH_CLOCK
  * @param  new_state (TRUE or FALSE)
  * @retval none
  */
void crm_periph_clock_enable(crm_periph_clock_type value, confirm_state new_state)
{
  /* enable periph clock */
  if(1 == new_state)
  {
    CRM_REG(value) |= CRM_REG_BIT(value);
  }
  /* disable periph clock */
  else
  {
    CRM_REG(value) &= ~(CRM_REG_BIT(value));
  }
}


/* for at32f4 extend sram to 224k
 *
 */
#define SRAM_96k 0xFF
#define SRAM_224k 0xFE
#define USD_BASE     ((uint32_t)0x1FFFF800)
typedef struct
{
  __IO uint16_t fap;
  __IO uint16_t ssb;
  __IO uint16_t data0;
  __IO uint16_t data1;
  __IO uint16_t epp0;
  __IO uint16_t epp1;
  __IO uint16_t epp2;
  __IO uint16_t epp3;
  __IO uint16_t eopb0;
  __IO uint16_t reserved;
  __IO uint16_t data2;
  __IO uint16_t data3;
  __IO uint16_t data4;
  __IO uint16_t data5;
  __IO uint16_t data6;
  __IO uint16_t data7;
  __IO uint16_t ext_flash_key[8];
} usd_type;
#define USD   ((usd_type *) USD_BASE)

//config eopb0 to 0xFE to enable 224k sram
void static Extend_SRAM(void) {
	if ((USD->eopb0 & 0xFF) != 0xFE) // check if RAM has been set to 224K, if not, change EOPB0
	{
		FLASH_Unlock();
		FLASH_EraseOptionBytes();
		FLASH_ProgramOptionByteData(0x1FFFF810, SRAM_224k);
 		FLASH_Lock();
 		NVIC_SystemReset();
	}
}

#endif
