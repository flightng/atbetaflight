/*
*****************************************************************************
**
**  File        : stm32_flash_1m.ld
**
**  Abstract    : Linker script for STM32F103RG Device with
**                1MByte FLASH, 96KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 10K
    FLASH_CUSTOM_DEFAULTS (r) : ORIGIN = 0x08002800, LENGTH = 6K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)       : ORIGIN = 0x08008000, LENGTH = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 976K : 992K
    FLASH_CUSTOM_DEFAULTS_EXTENDED (r): ORIGIN = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 0x080FC000 : 0x08100000, LENGTH = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 16K : 0K
       
    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 224K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)
REGION_ALIAS("VECTAB", RAM)

REGION_ALIAS("MOVABLE_FLASH", FLASH1)

INCLUDE "at32_flash_f4_split.ld"
