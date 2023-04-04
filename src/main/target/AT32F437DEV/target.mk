AT32F43xxMT7_TARGETS += $(TARGET)

CUSTOM_DEFAULTS_EXTENDED = yes
FEATURES       +=  VCP ONBOARDFLASH

TARGET_SRC = \
			drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm426xx.c\
            drivers/barometer/barometer_bmp280.c \
            drivers/compass/compass_hmc5883l.c\
            drivers/compass/compass_qmc5883l.c\
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c\
            drivers/accgyro/accgyro_spi_lsm6ds3_init.c \
            drivers/accgyro/accgyro_spi_lsm6ds3.c \
            drivers/accgyro/accgyro_spi_lsm6dsl_init.c \
            drivers/accgyro/accgyro_spi_lsm6dsl.c \
            drivers/accgyro/accgyro_spi_lsm6dso_init.c \
            drivers/accgyro/accgyro_spi_lsm6dso.c \
            drivers/accgyro/accgyro_spi_qmi8658_init.c \
            drivers/accgyro/accgyro_spi_qmi8658.c \
            drivers/accgyro/accgyro_spi_sh3001_init.c \
            drivers/accgyro/accgyro_spi_sh3001.c \
            drivers/accgyro/accgyro_spi_asm330lhh_init.c \
            drivers/accgyro/accgyro_spi_asm330lhh.c \
            drivers/max7456.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/rx/expresslrs_driver_atbsp.c \
            drivers/rx/rx_sx127x.c \
            drivers/rx/rx_sx1280.c \
            rx/expresslrs_telemetry.c \
            rx/expresslrs_common.c \
            rx/expresslrs.c\
            io/fuzzy_co_processor.c

