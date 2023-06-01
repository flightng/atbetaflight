AT32F43xxGT7_TARGETS += $(TARGET) #FOR 1024KB

CUSTOM_DEFAULTS_EXTENDED = yes
FEATURES       +=  VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_dps310.c\
            $(ROOT)/lib/main/BoschSensortec/BMI270-Sensor-API/bmi270.c \
            drivers/accgyro/accgyro_spi_bmi270_init.c \
            drivers/accgyro/accgyro_spi_bmi270.c\
            drivers/accgyro/accgyro_spi_lsm6dso_init.c \
            drivers/accgyro/accgyro_spi_lsm6dso.c \
            drivers/max7456.c \
            drivers/max7456.c \
