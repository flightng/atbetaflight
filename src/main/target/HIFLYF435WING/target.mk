#AT32F43xxMT7_TARGETS += $(TARGET)	#FOR 4032KB 
AT32F43xxGT7_TARGETS += $(TARGET) #FOR 1024KB

CUSTOM_DEFAULTS_EXTENDED = yes
FEATURES       +=  VCP ONBOARDFLASH

TARGET_SRC = \
        drivers/accgyro/accgyro_mpu6500.c \
        drivers/accgyro/accgyro_spi_mpu6500.c \
        drivers/accgyro/accgyro_spi_icm426xx.c\
        drivers/barometer/barometer_bmp280.c \
        drivers/barometer/barometer_dps310.c\
        drivers/compass/compass_hmc5883l.c\
        drivers/compass/compass_qmc5883l.c\
        drivers/max7456.c
