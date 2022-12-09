/*
 * co_fuzzi_processor.h
 *
 *  Created on: 2022年10月22日
 *      Author: shanggl
 */

#pragma once

#include "flight/pid.h"
#include "flight/pid.h"


extern pidDelta_t deltaPidBuffer[4];

bool fuzzyCoProcessorInit(void);

void fuzzyCoProcessorSendError(int16_t errRoll,int16_t errPitch ,int16_t errYaw,int16_t errHigh);

// static pidDelta_t fuzzyCoProcessorRecv();
bool fuzzyCoProcessorRecv();



