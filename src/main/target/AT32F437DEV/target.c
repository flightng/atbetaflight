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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

 #include "drivers/dma.h"
 #include "drivers/timer.h"
 #include "drivers/timer_def.h"

 const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
 	DEF_TIM(TMR2, CH3, PB10, TIM_USE_ANY |TIM_USE_LED, 0,1,0), // PWM1 - OUT1 MCO1 DMA1 CH2
	DEF_TIM(TMR2, CH4, PB11,  TIM_USE_ANY |TIM_USE_BEEPER, 0,5,0), // PWM2 - OUT1 DMA1 CH6
	DEF_TIM(TMR3, CH1, PC6,  TIM_USE_ANY |TIM_USE_SERVO, 0,6,12), // PWM3 - OUT3  DMA1 CH7
	DEF_TIM(TMR3, CH2, PC7,  TIM_USE_ANY |TIM_USE_SERVO, 0,7,12), // PWM4 - OUT4  DMA2 CH1
	DEF_TIM(TMR3, CH3, PC8,  TIM_USE_ANY |TIM_USE_SERVO, 0,8,12), // PWM5 - OUT5  DMA2 CH2
	DEF_TIM(TMR3, CH4, PC9,  TIM_USE_ANY |TIM_USE_SERVO, 0,9,12), // PWM6 - OUT6  DMA2 CH3

 	DEF_TIM(TMR4, CH1, PB6,  TIM_USE_MOTOR, 0,13,13),  // motor1 DMA2 CH7
 	DEF_TIM(TMR4, CH2, PB7,  TIM_USE_MOTOR, 0,12,13),  // motor2 DMA2 CH6
 	DEF_TIM(TMR4, CH3, PB8,  TIM_USE_MOTOR,  0,11,13), // motor3 DMA2 CH5
 	DEF_TIM(TMR4, CH4, PB9,  TIM_USE_MOTOR,  0,10,13), // motor4 DMA2 CH4

 };
