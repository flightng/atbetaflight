#
# AT32F4 Make file include
#
ifeq ($(TARGET),$(filter $(TARGET), $(AT32F43xxMT7_TARGETS)))
MCU_FLASH_SIZE  := 4032
else ifeq ($(TARGET),$(filter $(TARGET), $(AT32F43xxGT7_TARGETS)))
MCU_FLASH_SIZE  := 1024
else ifeq ($(TARGET),$(filter $(TARGET), $(AT32F43xxCT7_TARGETS)))
MCU_FLASH_SIZE  := 256
else
MCU_FLASH_SIZE  := 256
endif

# for the bsp dir & src 
STDPERIPH_DIR   = $(ROOT)/lib/main/AT32F43x/drivers/
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = \
				  at32f435_437_dvp.c\
				  at32f435_437_can.c\
				  at32f435_437_xmc.c\
				  at32f435_437_emac\
				  
STARTUP_SRC     = startup_at32f435_437.S
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the bsp cmsis sources
VPATH           := $(VPATH):$(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support/

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4/

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

#fixme: vcp & support later 
ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(ROOT)/src/main/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
                        $(USBPERIPH_SRC)

endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xM.ld
endif

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion


ifeq ($(DEVICE_FLAGS),)
DEVICE_FLAGS    = -DAT32F437ZMT7
endif
DEVICE_FLAGS   += -DAT32F43x -DHSE_VALUE=$(HSE_VALUE)


#VCP_SRC = \
            vcp/hw_config.c \
            vcp/stm32_it.c \
            vcp/usb_desc.c \
            vcp/usb_endp.c \
            vcp/usb_istr.c \
            vcp/usb_prop.c \
            vcp/usb_pwr.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            startup/system_at32f435_437.c\
            startup/at32f435_437_clock.c\
            # drivers/adc_stm32f10x.c \
            # drivers/bus_i2c_stm32f10x.c \
            # drivers/bus_spi_stdperiph.c \
            # drivers/dma.c \
            # drivers/inverter.c \
            # drivers/light_ws2811strip_stdperiph.c \
            # drivers/serial_uart_stdperiph.c \
            # drivers/serial_uart_stm32f10x.c \
            # drivers/system_stm32f10x.c \
            # drivers/timer_stm32f10x.c\
            # drivers/pwm_output_dshot_shared.c \
			# drivers/pwm_output_dshot.c \
			# drivers/dshot_bitbang.c \
            # drivers/dshot_bitbang_decode.c \
            # drivers/dshot_bitbang_stdperiph.c \
            
            

DSP_LIB :=

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      :=
OPTIMISE_SIZE       :=

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

DEVICE_FLAGS +=  -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING  -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4 

