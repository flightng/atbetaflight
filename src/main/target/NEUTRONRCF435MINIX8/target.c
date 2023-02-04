/* * This file is part of Cleanflight and Betaflight.
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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TMR1, CH1, PA8, TIM_USE_ANY | TIM_USE_LED, 0, 8, 2),             // PWM1 - OUT1  LEDSTRIP /MCO1

    DEF_TIM(TMR4, CH1, PB6,  TIM_USE_MOTOR, 0, 0, 0), // motor1 DMA1 CH1
    DEF_TIM(TMR1, CH3, PA10, TIM_USE_MOTOR, 0, 1, 0), // motor2 DMA1 CH2
    DEF_TIM(TMR2, CH4, PA3,  TIM_USE_MOTOR, 0, 2, 1), // motor3 DMA1 CH3
    DEF_TIM(TMR3, CH4, PB1,  TIM_USE_MOTOR, 0, 3, 1), // motor4 DMA1 CH4

    DEF_TIM(TMR4, CH2, PB7,  TIM_USE_ANY | TIM_USE_MOTOR, 0, 4, 10), // motor5 FIXME
    DEF_TIM(TMR1, CH2, PA9,  TIM_USE_ANY | TIM_USE_MOTOR, 0, 5, 10), // motor6 FIXME
    DEF_TIM(TMR3, CH3, PB0,  TIM_USE_ANY | TIM_USE_MOTOR, 0, 6, 10), // motor7 FIXME
    DEF_TIM(TMR2, CH3, PA2,  TIM_USE_ANY | TIM_USE_MOTOR, 0, 7, 10), // motor8 FIXME

};
