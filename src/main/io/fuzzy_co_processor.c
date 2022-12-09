#include "io/fuzzy_co_processor.h"
#include "io/serial.h"
#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#ifdef USE_FUZZY_CO_PROCESSOR

//for Roll=0 Pitch Yaw High
FAST_DATA_ZERO_INIT pidDelta_t deltaPidBuffer[4];


// baud rate should be optimized, since the flc is tested by CP210x which only give stable output at 500_000 baud rate
// we can increase the baud rate to 12_000_000 when we use the real flc

/* FCP frame structure:
	<Header>	[1 byte]	0x55
	<Type>		[1 byte]
	<Payload>	[x bytes]
	<CRC>		[1 byte], CRC8 of the <Type>+<Payload>
*/
#define CO_PROCESS_RX_LEN 9
static serialPort_t *coProcessorPort;
static int8_t coRecvBuffer[12];// 9 used for now
static int32_t timestampSend;//发送次数，有可能会溢出
static int32_t timestampRecv=0;//接收次数
static int32_t lastRecvCnt=0;
static uint8_t payloadLen;//payload长度
static uint16_t symbleErrorCountTX;// 发送误码次数
//static uint16_t symbleErrorCountRX;// 接收误码次数
static bool badFrame;

/*
	协处理器初始化，默认先使用 uart5
*/

void fuzzyProcessFrame(uint16_t c,void * unused);

bool fuzzyCoProcessorInit( void ){

	const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_CO_PROCESSOR);
	 portOptions_e portOptions = 0;

	    if (portConfig) {
	        coProcessorPort = openSerialPort(portConfig->identifier, FUNCTION_CO_PROCESSOR, NULL, NULL, 115200, MODE_RXTX, portOptions);

	    }else{
			//fixme : for debug use port5
	        coProcessorPort = openSerialPort(CO_PROCESSOR_UART, FUNCTION_CO_PROCESSOR, fuzzyProcessFrame, NULL, CO_PROCESSOR_UART_BAUD, MODE_RXTX, portOptions);

	    }

	    if (!coProcessorPort) {
	        return false;
	    }
	    //FINISH SERIAL PASS THROUGH

	    return true;
}


/* Type E0: Roll, Pitch, Yaw axex rate error
	FC -> CP
	<Payload>	[7 bytes]
		<timestampSend>	[1 bytes]
		ROLL_E_HIGH , ROLL_E_LOW 
		PITCH_E_HIGH, PITCH_E_LOW
		YAW_E_HIGH ,YAW_E_LOW
*/
void fuzzyCoProcessorSendError(int16_t errRoll,int16_t errPitch ,int16_t errYaw,int16_t errHigh){


    if (!coProcessorPort) {
        return ;
    }

	uint8_t txErrorBuffer[10];


	UNUSED(errHigh);

	sbuf_t buf;
	// prepare pointer
	buf.ptr = txErrorBuffer;
	buf.end = ARRAYEND(txErrorBuffer);

	sbufWriteU8(&buf, 0x55);
	sbufWriteU8(&buf, 0xE0);
	sbufWriteU8(&buf, timestampSend);

	sbufWriteData(&buf, &errRoll, 2);
	sbufWriteData(&buf, &errPitch, 2);
	sbufWriteData(&buf, &errYaw, 2);

	// add crc over (all) data
	crc8_dvb_s2_sbuf_append(&buf, txErrorBuffer);

	// switch to reader
	sbufSwitchToReader(&buf, txErrorBuffer);

	// send data if possible
	serialWriteBuf(coProcessorPort, sbufPtr(&buf), sbufBytesRemaining(&buf));

	timestampSend++;
}

/* Type E1: response to E0
	CP -> FC
	<Payload>	[9 bytes]
		<timestampRecv>	[1 bytes]
		ROLL_P , ROLL_I , ROLL_D 
		PITCH_P, PITCH_I, PITCH_D
		YAW_P ,YAW_I
	Noted: each PID needs to multiply 10 before apply to the real PID controller 
*/

//process 0x55 0xE1 timecount r.p r.i r.d p.p p.i p.d y.p y.i 0xCC

void fuzzyProcessFrame(uint16_t c,void * unused)
{
	UNUSED(unused);
	static enum coRecvState_e {
        FCP_HEADER=1, // Waiting for preamble 1 (0x55)
        FCP_TYPE, 	// Waiting for preamble 2 (0xE1)
        FCP_PAYLOAD,// Receiving data
        FCP_CRC,	// Waiting for CRC  0XCC
	} coRecvState = FCP_HEADER;

    static int r_index;
//	uint8_t crc;

	switch(coRecvState){
		case FCP_HEADER:
			if(0x55==c){
				coRecvState=FCP_TYPE;
			}else{ // Co-processor received bad frame
				symbleErrorCountTX++;
				badFrame=true;
				coRecvState=FCP_TYPE;
			}
			break;
		case FCP_TYPE:
			if(0xE1==c){
				coRecvState=FCP_PAYLOAD;
				payloadLen = CO_PROCESS_RX_LEN;
				r_index=0;
			}
			else{
				coRecvState=FCP_HEADER;
			}
			break;
		case FCP_PAYLOAD:
			coRecvBuffer[r_index] = c;
			if (++r_index == payloadLen) {
				coRecvState = FCP_CRC;
			}
			break;
		case FCP_CRC:
//			crc=c;
			//check crc
			//PROCESS RECV 
			deltaPidBuffer[0].DP=coRecvBuffer[1];
			deltaPidBuffer[0].DI=coRecvBuffer[2];
			deltaPidBuffer[0].DD=coRecvBuffer[3];
			deltaPidBuffer[1].DP=coRecvBuffer[4];
			deltaPidBuffer[1].DI=coRecvBuffer[5];
			deltaPidBuffer[1].DD=coRecvBuffer[6];
			deltaPidBuffer[2].DP=coRecvBuffer[7];
			deltaPidBuffer[2].DI=coRecvBuffer[8];
			timestampRecv++;
		coRecvState=FCP_HEADER;
		break;
	}
}

//在 mainpid Loop 中调用，读取串口缓存到 pid buffer 之后直接从pid buffer 获取 pid信息
// static pidDelta_t fuzzyCoProcessorRecv(){
bool fuzzyCoProcessorRecv(){

    if (badFrame || 0==timestampRecv){
    		badFrame=false;
    		for (int i=0;i<4;i++){
    			deltaPidBuffer[i].DP=0;
    			deltaPidBuffer[i].DI=0;
    			deltaPidBuffer[i].DD=0;
    		}
    	}
   if(lastRecvCnt != timestampRecv )
   {
	   lastRecvCnt =timestampRecv;
	   return true;
   }
   return false;
	// return deltaPidBuffer;
}

#endif

