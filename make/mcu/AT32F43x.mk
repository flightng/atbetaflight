#
# AT32F4 Make file include
#
ifeq ($(TARGET),$(filter $(TARGET), $(AT32F43xxMT7_TARGETS)))
MCU_FLASH_SIZE  := 4032
DEVICE_FLAGS    = -DAT32F437VMT7
else ifeq ($(TARGET),$(filter $(TARGET), $(AT32F43xxGT7_TARGETS)))
MCU_FLASH_SIZE  := 1024
DEVICE_FLAGS    = -DAT32F435RGT7
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
				  at32f435_437_emac
				  
				  
				  
STARTUP_SRC     = startup_at32f435_437.S
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#I2C_APPLICATION_DIR=$(ROOT)/lib/main/AT32F43x/i2c_application_library
#I2C_APPLICATION_SRC=$(notdir $(wildcard $(I2C_APPLICATION_DIR)/*.c))

# Search path and source files for the bsp cmsis sources
VPATH           := $(VPATH):$(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support/

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support \
                   $(ROOT)/lib/main/AT32F43x/cmsis/cm4/ \
                   #$(I2C_APPLICATION_DIR)\
                   

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)\
					# $(I2C_APPLICATION_SRC)

#fixme: vcp & support later 
ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc\
                   $(ROOT)/lib/main/AT32F43x/usbd_class/cdc/\
                  

VPATH           := $(VPATH):$(USBFS_DIR)/src\

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
                        $(USBFS_DIR)/src/usb_core.c\
                        $(USBFS_DIR)/src/usbd_core.c\
                        $(USBFS_DIR)/src/usbd_int.c\
                        $(USBFS_DIR)/src/usbd_sdr.c\

endif

ifeq ($(LD_SCRIPT),)
ifeq ($(MCU_FLASH_SIZE),4032)
	LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xM.ld
endif
ifeq ($(MCU_FLASH_SIZE),1024)
	LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xG.ld
endif
endif

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion



DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE)


VCP_SRC = 	$(ROOT)/lib/main/AT32F43x/usbd_class/cdc/cdc_class.c\
			$(ROOT)/lib/main/AT32F43x/usbd_class/cdc/cdc_desc.c\
            drivers/serial_usb_vcp_at32f43x.c \
            drivers/usb_io.c
            

MCU_COMMON_SRC = \
            startup/system_at32f435_437.c\
            startup/at32f435_437_clock.c\
            drivers/system_at32f43x.c\
            drivers/light_ws2811strip_at32f43x.c \
            drivers/timer_atbsp.c\
            drivers/timer_at32f43x.c\
            drivers/dma_at32f43x.c\
            drivers/i2c_application.c\
            drivers/bus_i2c_atbsp_init.c \
            drivers/bus_i2c_atbsp.c\
			drivers/bus_spi_at32bsp.c\
            drivers/inverter.c \
            drivers/serial_uart_at32bsp.c\
            drivers/serial_uart_at32f43x.c\
            drivers/pwm_output_dshot_shared.c \
			drivers/pwm_output_dshot.c \
			drivers/dshot_bitbang.c \
            drivers/dshot_bitbang_decode.c \
            drivers/dshot_bitbang_at32bsp.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/persistent.c\
            drivers/camera_control_atbsp.c\
            drivers/adc_at32f43x.c \

            
            

DSP_LIB :=

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      :=
OPTIMISE_SIZE       :=

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

DEVICE_FLAGS +=  -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING  -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4 

