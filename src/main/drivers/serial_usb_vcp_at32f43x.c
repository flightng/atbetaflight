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
#include <stdbool.h>

#include "platform.h"

#ifdef USE_VCP

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/usb_io.h"

#include "pg/usb.h"

#include "at32f435_437_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"


#include "drivers/time.h"

#include "serial.h"
#include "serial_usb_vcp_at32f43x.h"
#include "nvic.h"
#include "at32f435_437_tmr.h"



#define USB_TIMEOUT  50

static vcpPort_t vcpPort;

otg_core_type otg_core_struct;


/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
  if(clk_s == USB_CLK_HICK)
  {
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* enable the acc calibration ready interrupt */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    /* update the c1\c2\c3 value */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
#if (USB_ID == 0)
    acc_sof_select(ACC_SOF_OTG1);
#else
    acc_sof_select(ACC_SOF_OTG2);
#endif
    /* open acc calibration */
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
  }
  else
  {
    switch(system_core_clock)
    {
      /* 48MHz */
      case 48000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1);
        break;

      /* 72MHz */
      case 72000000:
        crm_usb_clock_div_set(CRM_USB_DIV_1_5);
        break;

      /* 96MHz */
      case 96000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2);
        break;

      /* 120MHz */
      case 120000000:
        crm_usb_clock_div_set(CRM_USB_DIV_2_5);
        break;

      /* 144MHz */
      case 144000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3);
        break;

      /* 168MHz */
      case 168000000:
        crm_usb_clock_div_set(CRM_USB_DIV_3_5);
        break;

      /* 192MHz */
      case 192000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4);
        break;

      /* 216MHz */
      case 216000000:
        crm_usb_clock_div_set(CRM_USB_DIV_4_5);
        break;

      /* 240MHz */
      case 240000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5);
        break;

      /* 264MHz */
      case 264000000:
        crm_usb_clock_div_set(CRM_USB_DIV_5_5);
        break;

      /* 288MHz */
      case 288000000:
        crm_usb_clock_div_set(CRM_USB_DIV_6);
        break;

      default:
        break;

    }
  }
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

  /* dp and dm */
  gpio_init_struct.gpio_pins = OTG_PIN_DP | OTG_PIN_DM;
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);

  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DP_SOURCE, OTG_PIN_MUX);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_DM_SOURCE, OTG_PIN_MUX);

#ifdef USB_SOF_OUTPUT_ENABLE
  crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
  gpio_init_struct.gpio_pins = OTG_PIN_SOF;
  gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_SOF_SOURCE, OTG_PIN_MUX);
#endif

  /* otgfs use vbus pin */
#ifndef USB_VBUS_IGNORE
  gpio_init_struct.gpio_pins = OTG_PIN_VBUS;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_pin_mux_config(OTG_PIN_GPIO, OTG_PIN_VBUS_SOURCE, OTG_PIN_MUX);
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);
#endif


}
#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  usb low power wakeup interrupt config
  * @param  none
  * @retval none
  */
void usb_low_power_wakeup_config(void)
{
  exint_init_type exint_init_struct;

  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  exint_default_para_init(&exint_init_struct);

  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = OTG_WKUP_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  nvic_irq_enable(OTG_WKUP_IRQ, NVIC_PRIORITY_BASE(NVIC_PRIO_USB_WUP), NVIC_PRIORITY_SUB(NVIC_PRIO_USB_WUP));
}

/**
  * @brief  this function handles otgfs wakup interrupt.
  * @param  none
  * @retval none
  */
void OTG_WKUP_HANDLER(void)
{
  exint_flag_clear(OTG_WKUP_EXINT_LINE);
}

#endif


/********************************************
 * copy from cdc part
 */


uint32_t CDC_Send_FreeBytes(void)
{
  cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
  if(pcdc->g_tx_completed){
    return APP_TX_BLOCK_SIZE;
  }else{
    return 0;
  }
}


/**
 * @brief  CDC_Send_DATA
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  ptrBuffer: Buffer of data to be sent
 * @param  sendLength: Number of data to be sent (in bytes)
 * @retval Bytes sent
 */
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
  cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
  uint32_t start = millis();
   
  uint32_t pos=0;
  while(pos < sendLength || (pos==sendLength && sendLength%64 == 0) ){//`==` is intended for sending 0 length packet
    int tosend=sendLength-pos;
    if(tosend>APP_TX_BLOCK_SIZE){
      tosend=APP_TX_BLOCK_SIZE;
    }
    while(pcdc->g_tx_completed != 1) {
      if (millis() - start > USB_TIMEOUT) {
        return pos;
      }
    }
    uint32_t txed=usb_vcp_send_data(&otg_core_struct.dev,(uint8_t *)(ptrBuffer+pos), tosend);
    if(pos==sendLength){
      break;
    }
    if (txed==SUCCESS) {
      pos+=tosend;
    }
  }
  return pos;
}


//是否插入
uint8_t usbIsConnected(){
  return (USB_CONN_STATE_DEFAULT !=otg_core_struct.dev.conn_state);
}

//是否配置
uint8_t usbIsConfigured(){
  return (USB_CONN_STATE_CONFIGURED ==otg_core_struct.dev.conn_state);
}

//vcp 状态
uint8_t usbVcpIsConnected()
{
    return usbIsConnected();
}
/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}

//for usbvcp functions


//设置vcp 波特率， 可以不用实现
static void usbVcpSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}

//设置vcp 模式，双线双工还是单线半双工， 可以不用实现
static void usbVcpSetMode(serialPort_t *instance, portMode_e mode)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

//set ctrl line state change callback
// 设置控制线状态回调函数,主要是vcp 电脑端设置 RTS线状态时，需要告知串口是否需要重启，需要实现
static void usbVcpSetCtrlLineStateCb(serialPort_t *instance, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    UNUSED(instance);

    // Register upper driver control line state callback routine with USB driver
    CDC_SetCtrlLineStateCb((void (*)(void *context, uint16_t ctrlLineState))cb, context);
}

//set baudrate change callback
// vcp 收到 set_line_coding 命令时，设置波特率
static void usbVcpSetBaudRateCb(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    UNUSED(instance);

    // Register upper driver baud rate callback routine with USB driver
    CDC_SetBaudRateCb((void (*)(void *context, uint32_t baud))cb, (void *)context);
}

static bool isUsbVcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

//缓冲区是否有数据,需要实现
static uint32_t usbVcpAvailable(const serialPort_t *instance)
{
  UNUSED(instance);  
  cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
  
  uint32_t available=APP_Rx_ptr_in-APP_Rx_ptr_out;
  if(pcdc->g_rx_completed == 1){
    available+=pcdc->g_rxlen;
  }    
  return available;
}

/*
 * vcp  中断接收转缓存、逐个读取的问题
 * 1、要考虑com口会不断的发数据过来， 缓存用尽怎么办
 * 2、如何判断当前缓存大小？
 * 3、如何逐个字符读取？
 * 目前vcp 无法完整处理、回复主要是这里的问题
 *
 *
 */

//读取1 字节数据
static uint8_t usbVcpRead(serialPort_t *instance)
{
    UNUSED(instance);

//检查缓存是否非空，如空，增加一次读取
   if ((APP_Rx_ptr_in==0)||(APP_Rx_ptr_out == APP_Rx_ptr_in)){
	   APP_Rx_ptr_out=0;
	   APP_Rx_ptr_in=usb_vcp_get_rxdata(&otg_core_struct.dev,APP_Rx_Buffer);// usb 每次 最大64 字节，不会溢出
	   if(APP_Rx_ptr_in==0)
	   {
		   //没有读到数据,返回一个0 ，避免返回上次的脏数据
		   return 0;
	   }
   }
    return APP_Rx_Buffer[APP_Rx_ptr_out++];//
}

//写buffer数据到vpc 需要实现

//这里有bug， 调用write 的时候是写到了缓存里，如果缓存满了仍然未发送，则会内存溢出死机，比如在Cli 初始化时，设置serialWriteBufShim
//重新理解函数名， 为想serial 写入一个缓存块，直接走vcp发出去
//但是这样修改仍然有问题，在msp 的处理中，报文头、数据、crc 3个包被分3次发送，导致接收方无法一次性接收、校验



static void usbVcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    UNUSED(instance);

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    uint32_t start = millis();
    const uint8_t *p = data;
    while (count > 0) {
        uint32_t txed = CDC_Send_DATA(p, count);
        count -= txed;
        p += txed;

        if (millis() - start > USB_TIMEOUT) {
            break;
        }
    }
}

//flash 缓冲区
static bool usbVcpFlush(vcpPort_t *port)
{
    uint32_t count = port->txAt;
    port->txAt = 0;

    if (count == 0) {
        return true;
    }

    if (!usbIsConnected() || !usbIsConfigured()) {
        return false;
    }

    uint32_t start = millis();
   uint8_t *p = port->txBuf;
   while (count > 0) {
	   uint32_t txed = CDC_Send_DATA(p, count);
	   count -= txed;
	   p += txed;

	   if (millis() - start > USB_TIMEOUT) {
		   break;
	   }
   }
   return count == 0;
}
static void usbVcpWrite(serialPort_t *instance, uint8_t c)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);

    port->txBuf[port->txAt++] = c;
    if (!port->buffering || port->txAt >= ARRAYLEN(port->txBuf)) {
        usbVcpFlush(port);
    }
}

static void usbVcpBeginWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = true;
}

static uint32_t usbTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return CDC_Send_FreeBytes();
}

static void usbVcpEndWrite(serialPort_t *instance)
{
    vcpPort_t *port = container_of(instance, vcpPort_t, port);
    port->buffering = false;
    usbVcpFlush(port);
}

static const struct serialPortVTable usbVTable[] = {
    {
        .serialWrite = usbVcpWrite,//write char
        .serialTotalRxWaiting = usbVcpAvailable,
        .serialTotalTxFree = usbTxBytesFree,     //Fixme: find the replace
        .serialRead = usbVcpRead,
        .serialSetBaudRate = usbVcpSetBaudRate,
        .isSerialTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
        .setMode = usbVcpSetMode,
        .setCtrlLineStateCb = usbVcpSetCtrlLineStateCb,     //Fixme: serial passthougth
        .setBaudRateCb = usbVcpSetBaudRateCb,     //Fixme: serial passthougth
        .writeBuf =  usbVcpWriteBuf, //write buffer
        .beginWrite = usbVcpBeginWrite,
        .endWrite = usbVcpEndWrite
    }
};

//打开串口 修改完毕
serialPort_t *usbVcpOpen(void)
{
    vcpPort_t *s;

    IOInit(IOGetByTag(IO_TAG(PA11)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PA12)), OWNER_USB, 0);
    /* usb gpio config */
     usb_gpio_config();

   #ifdef USB_LOW_POWER_WAKUP
     usb_low_power_wakeup_config();
   #endif

     /* enable otgfs clock */
     crm_periph_clock_enable(OTG_CLOCK, TRUE);

     /* select usb 48m clcok source */
     usb_clock48m_select(USB_CLK_HEXT);

     /* enable otgfs irq,不能设置太高的优先级，会干扰spi 的dma中断通信 */
     nvic_irq_enable(OTG_IRQ, NVIC_PRIORITY_BASE(NVIC_PRIO_USB), NVIC_PRIORITY_SUB(NVIC_PRIO_USB));

     usbGenerateDisconnectPulse();

     /* init usb */
     usbd_init(&otg_core_struct,
               USB_FULL_SPEED_CORE_ID,
               USB_ID,
               &cdc_class_handler,
               &cdc_desc_handler);



    s = &vcpPort;
    s->port.vTable = usbVTable;


    return (serialPort_t *)s;
}
//查询波特率 完成修改
uint32_t usbVcpGetBaudRate(serialPort_t *instance)
{
    UNUSED(instance);
    //    return CDC_BaudRate();//pdev->bitrate

    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct.dev.class_handler->pdata;
    return pcdc->linecoding.bitrate;
}
#endif

