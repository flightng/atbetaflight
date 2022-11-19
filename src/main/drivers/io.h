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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "resource.h"

#include "drivers/io_types.h"

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

#if defined(AT32F43x)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE )  // TODO
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_UP )
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_STRONGER , GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT , GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_MUX ,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_MUX ,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_MUX ,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_MUX ,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT ,  GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT ,  GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT ,  GPIO_DRIVE_STRENGTH_MODERATE, 0,             GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT ,  GPIO_DRIVE_STRENGTH_MODERATE , 0, GPIO_PULL_UP)


#elif defined(UNIT_TEST) || defined(SIMULATOR_BUILD)

# define IOCFG_OUT_PP         0
# define IOCFG_OUT_OD         0
# define IOCFG_AF_PP          0
# define IOCFG_AF_OD          0
# define IOCFG_IPD            0
# define IOCFG_IPU            0
# define IOCFG_IN_FLOATING    0

#else
# warning "Unknown TARGET"
#endif

#if defined(AT32F43x)
// Expose these for EXTIConfig
#define IO_CONFIG_GET_MODE(cfg) (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 4) & 0x01)
#define IO_CONFIG_GET_PULL(cfg) (((cfg) >> 5) & 0x03)
#endif

// declare available IO pins. Available pins are specified per target
#include "io_def.h"

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);

void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
void IORelease(IO_t io);  // unimplemented
resourceOwner_e IOGetOwner(IO_t io);
bool IOIsFreeOrPreinit(IO_t io);
IO_t IOGetByTag(ioTag_t tag);

void IOConfigGPIO(IO_t io, ioConfig_t cfg);

#if defined(AT32F43x)
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
#endif

void IOInitGlobal(void);

typedef void (*IOTraverseFuncPtr_t)(IO_t io);

void IOTraversePins(IOTraverseFuncPtr_t func);

gpio_type * IO_GPIO(IO_t io);
uint16_t IO_Pin(IO_t io);
