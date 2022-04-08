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

#include "rcc_types.h"
#include "rcc_at32f43x_periph.h"



#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

enum rcc_reg {
    RCC_EMPTY = 0,   // make sure that default value (0) does not enable anything
    RCC_AHB1,
    RCC_AHB2,
    RCC_AHB3,
    RCC_APB2,
    RCC_APB1,
};

#define RCC_ENCODE(reg, mask) (((reg) << 5) | LOG2_32BIT(mask))
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1,   CRM_AHB1_ ## periph ## _PER_MASK)
#define RCC_AHB2(periph) RCC_ENCODE(RCC_AHB2,   CRM_AHB2_ ## periph ## _PER_MASK)
#define RCC_AHB3(periph) RCC_ENCODE(RCC_AHB3,   CRM_AHB3_ ## periph ## _PER_MASK)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, 	CRM_APB1_ ## periph ## _PER_MASK)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, 	CRM_APB2_ ## periph ## _PER_MASK)


void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
