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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(AT32F43x)

const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
    { RCC_AHB1(GPIOG) },
    { RCC_AHB1(GPIOH) }
};

#endif


ioRec_t* IO_Rec(IO_t io)
{
    return io;
}

gpio_type* IO_GPIO(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->gpio;
}

uint16_t IO_Pin(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pin;
}

// port index, GPIOA == 0
int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return 31 - __builtin_clz(IO_Pin(io));  // CLZ is a bit faster than FFS
}

int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

// mask on stm32f103, bit index on stm32f303 what for at32?
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(AT32F43x)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(SIMULATOR_BUILD)
    return 0;
#else
# error "Unknown target type"
#endif
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    return (IO_GPIO(io)->idt & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }

    IO_GPIO(io)->scr = IO_Pin(io) << (hi ? 0 : 16);
}


void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->scr =IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->clr =IO_Pin(io);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);
    // Read pin state from odt but write to scr because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.

    if (IO_GPIO(io)->odt & mask)
        mask <<= 16;   // bit is set, shift mask to reset half
    IO_GPIO(io)->scr = mask;
}

// claim IO pin, set owner and resources
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = owner;
    ioRec->index = index;
}

void IORelease(IO_t io)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = OWNER_FREE;
}

resourceOwner_e IOGetOwner(IO_t io)
{
    if (!io) {
        return OWNER_FREE;
    }
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->owner;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    resourceOwner_e owner = IOGetOwner(io);

    if (owner == OWNER_FREE || owner == OWNER_PREINIT) {
        return true;
    }

    return false;
}

/*
 * 初始化配置GPIO
 * io_t
 * ioconfig_t
 */
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_init_type init = {
        .gpio_pins = IO_Pin(io),
        .gpio_mode_type = (cfg >> 0) & 0x03,
        .gpio_drive_type = (cfg >> 2) & 0x03,
        .gpio_out_type = (cfg >> 4) & 0x01,
        .gpio_pull_type = (cfg >> 5) & 0x03,
    };
    gpio_init(IO_GPIO(io), &init);
}

/*
 * GPIO 复用&初始化
 * 需要注意的是 at32的复用标记并不是特别可读，需要对应一下GPIO_MUX_x 的具体含义
 */

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;

    gpio_pin_mux_config(IO_GPIO(io), IO_GPIO_PinSource(io), af);

    RCC_ClockCmd(rcc, ENABLE);

    gpio_init_type init = {
        .gpio_pins = IO_Pin(io),
        .gpio_mode_type = (cfg >> 0) & 0x03,
        .gpio_drive_type = (cfg >> 2) & 0x03,
        .gpio_out_type = (cfg >> 4) & 0x01,
        .gpio_pull_type = (cfg >> 5) & 0x03,
    };
    gpio_init(IO_GPIO(io), &init);
}





#if DEFIO_PORT_USED_COUNT > 0
static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
// Avoid -Wpedantic warning
static const uint16_t ioDefUsedMask[1] = {0};
static const uint8_t ioDefUsedOffset[1] = {0};
#endif
#if DEFIO_IO_USED_COUNT
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
#else
// Avoid -Wpedantic warning
ioRec_t ioRecs[1];
#endif

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
            if (ioDefUsedMask[port] & (1 << pin)) {
                ioRec->gpio = (gpio_type *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                ioRec->pin = 1 << pin;
                ioRec++;
            }
        }
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    // check if pin exists
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx))) {
        return NULL;
    }
    // count bits before this pin on single port
    int offset = __builtin_popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // and add port offset
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}

void IOTraversePins(IOTraverseFuncPtr_t fnPtr)
{
    for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        fnPtr(&ioRecs[i]);
    }
}
