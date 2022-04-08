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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// #include "fc/init.h"

// #include "scheduler/scheduler.h"
#include "drivers/system.h"
#include "drivers/light_led.h"
#include "drivers/io.h"
#include "drivers/exti.h"

#include "drivers/buttons.h"
#include "common/color.h"
#include "io/ledstrip.h"
#include "drivers/timer.h"
#include "drivers/light_ws2811strip.h"
#include "pg/timerio.h"

void ledstripInit(){

	ws2811LedStripInit(DEFIO_TAG(PA8));
	setUsedLedCount(32);
	ws2811LedStripEnable();

}



void init(void){
    systemInit();
    IOInitGlobal();

    statusLedConfig_t statusLedConfig={
        .ioTags={
            DEFIO_TAG(PD13),
            DEFIO_TAG(PD14)
        },
        .inversion=0
    };
    // initialize IO (needed for all IO operations)
    ledInit(&statusLedConfig);
    LED0_ON;

//    EXTIInit();
    
    buttonsInit();

    LED1_ON;

    timerInit();  // timer must be initialized before any channel is allocated


    timerIOConfig_t timerConfig={
    		.ioTag=DEFIO_TAG(PA8),
			.index=1,
			.dmaopt=0
    };
    timerIOConfig_SystemArray[1]=timerConfig; //配置对应定时器
    ledstripInit();
}

int main(void)
{
    init();

    uint32_t count=0;

    hsvColor_t hsvColors[]={
    		{0,100,100},
			{120,100,100},
			{240,100,100},
			{0,0,100}
    };

  
    // blink led
    while(1){
    	if(buttonAPressed()){
    		indicateFailure(1,1);
    		setStripColor(&hsvColors[count]);
    		//Ws2812 use  LED_GRB
    		ws2811UpdateStrip(LED_GRB,100);
    		//for dbg only
    		count+=1;
        	if (count>3) count=0;

    	}
    }
    return 0;
}

