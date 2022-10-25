/*
 * co_fuzzi_processor.h
 *
 *  Created on: 2022年10月22日
 *      Author: shanggl
 */

#pragma once

#include "flight/pid.h"

//for Roll=0 Pitch Yaw High 
static pidDelta_t deltaPidBuffer[4];

static int8_t coRecvBuffer[12];// 9 used for now

bool fuzzyCoProcessorInit(void);

static void fuzzyCoProcessorSendError(int16_t errRoll,int16_t errPitch ,int16_t errYaw,int16_t errHigh);

// static pidDelta_t fuzzyCoProcessorRecv();
static void fuzzyCoProcessorRecv();



