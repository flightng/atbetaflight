/*
 * 模糊PID 协处理器
 * 通过 uart5 调用协处理器进行数据处理
 *
 *
 */


#ifdef  USE_FUZZI_CO_PROCESSOR
#include <io/fuzzy_co_processor.h>
#include <io/serial.h>

#define CO_PROCESSOR_BAUDRATE 12000000
// baud rate should be optimized, since the flc is tested by CP210x which only give stable output at 500_000 baud rate
// we can increase the baud rate to 12_000_000 when we use the real flc

static serialPort_t *coProcessorPort;
static int8_t sendCnt;//发送次数，有可能会溢出
static int8_t recvCnt;//接收次数

/*
	协处理器初始化，默认先使用 uart5
*/

bool fuzzyCoProcessorInit( void ){

	const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_CO_PROCESSOR);

	    if (portConfig) {
	        portOptions_e portOptions = 0;
	        portOptions = SERIAL_BIDIR;
	        coProcessorPort = openSerialPort(portConfig->identifier, FUNCTION_CO_PROCESSOR, NULL, NULL, 500000, MODE_RXTX, portOptions);

			//fixme : for debug use port5
//	        coProcessorPort = openSerialPort(CO_PROCESSOR_UART, FUNCTION_CO_PROCESSOR, NULL, NULL, CO_PROCESSOR_BAUDRATE, MODE_RXTX, portOptions);

	    }

	    if (!coProcessorPort) {
	        return false;
	    }
	    //FINISH SERIAL PASS THROUGH

	    return true;
}
/*
	在mainpid loop 中 ，发送error信息到协处理器

*/
static void fuzzyCoProcessorSendError(int16_t errRoll,int16_t errPitch ,int16_t errYaw,int16_t errHigh){

	int8_t txBuffer[8];
/* 发送数据帧类型
	0X55 
	0XE0  
	SEND_CNT
	ROLL_E_HIGH , ROLL_E_LOW 
	PITCH_E_HIGH, PITCH_E_LOW
	YAW_E_HIGH ,YAW_E_LOW
	CRC
*/
	txBuffer[0]	=0x55;
	txBuffer[1]	=0xE0;
	txBuffer[2]	=sendCnt;
	txBuffer[3]	= (uint8_t)(errRoll  	>> 8);
	txBuffer[4] = (uint8_t)(errRoll		&0x00FF);
	txBuffer[5]	= (uint8_t)(errPitch 	>> 8); 
	txBuffer[6] = (uint8_t)(errPitch	&0x00FF);
	txBuffer[7]	= (uint8_t)(errYaw   	>> 8);
	txBuffer[8] = (uint8_t)(errYaw		&0x00FF);
	UNUSED(errHigh);

	for(int8_t i=0;i<10;i++)
	{
		serialWrite(coProcessorPort, txBuffer[8]);
	}

	sendCnt++;
}

//process 0x55 0xE1 timecount r.p r.i r.d p.p p.i p.d y.p y.i 0xCC

static void FuzziReceiveFrame(uint8_t c)
{
	static enum coRecvState_e {
        CO_HEADER_1,  // Waiting for preamble 1 (0x55)
        CO_HEADER_2, // Waiting for preamble 2 (0xE1)
        CO_DATA,     // Receiving data
        CO_WAITCRC,  // Waiting for CRC  0XCC
    } state = CO_HEADER_1;

    static int r_index;

	switch(state){
		case CO_HEADER_1:
			if(0x55==c){
				state=CO_HEADER_2;
			}
			else{
				state=CO_HEADER_1;
			}
			break;
		case CO_HEADER_2:
			if(0xE1==c){
				state=CO_DATA;
				r_index=0;
			}
			else{
				state=CO_HEADER_1;
			}
			break;
		case CO_DATA:
			 coRecvBuffer[r_index] = c;
			if (++r_index == 9) {
				state = CO_WAITCRC;
			}
			break;
		case CO_WAITCRC:
			int8_t crc=c;
			//check crc
			//PROCESS RECV 
			pid_buffer[0].P=coRecvBuffer[1];
			pid_buffer[0].I=coRecvBuffer[2];
			pid_buffer[0].D=coRecvBuffer[3];
			pid_buffer[1].P=coRecvBuffer[4];
			pid_buffer[1].I=coRecvBuffer[5];
			pid_buffer[1].D=coRecvBuffer[6];
			pid_buffer[2].P=coRecvBuffer[7];
			pid_buffer[2].I=coRecvBuffer[8];
			recvCnt++;
		state=CO_HEADER_1;
		break;
	}


}

//在 mainpid Loop 中调用，读取串口缓存到 pid buffer 之后直接从pid buffer 获取 pid信息
static void fuzzyCoProcessorRecv(){

    if (coProcessorPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(coProcessorPort) > 0) {
        uint8_t c = serialRead(coProcessorPort);
        FuzziReceiveFrame(c);
    }

}



#endif
