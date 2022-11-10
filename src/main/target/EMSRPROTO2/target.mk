AT32F43xxMT7_TARGETS += $(TARGET)

CUSTOM_DEFAULTS_EXTENDED = yes
FEATURES       +=  VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_icm426xx.c\
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c\
            drivers/compass/compass_qmc5883l.c\
            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270_maximum_fifo.c \
            drivers/accgyro/accgyro_spi_bmi270.c\
            drivers/accgyro/accgyro_spi_lsm6dsm_init.c \
            drivers/accgyro/accgyro_spi_lsm6dsm.c \
            drivers/accgyro/accgyro_spi_lsm6dso_init.c \
            drivers/accgyro/accgyro_spi_lsm6dso.c \
            drivers/max7456.c \

