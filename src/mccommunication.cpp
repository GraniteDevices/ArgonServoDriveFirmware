/*
 * mccommunication.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: tero
 */

#include "mccommunication.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Serial.h"
#include "EncoderIn.h"
#include "utils.h"
#include "globals.h"
#include "DSCPowerTask.h"
#include "RingBuffer.h"
#include "simplemotion_defs.h"



/* Packet format from STM to DSC:
 * u8 header:
 * 	 bits 0-3 (LSB): packet type
 * 	 bits 4-7: control bits TBD
 * u16 position feedback
 * s16 velocity feedback
 * s32 target reference (vel/torq/pos)
 * u8 user cmd id (from SM485)
 * u16 user cmd param (from SM485)
 * u8 stm cmd id (for control, such as hall sensor state change and index pulse occurrence)
 * u16 stm cmd param (for control, such as hall sensor state change and index pulse occurrence)
 * u8 GPI bytes
 * u8 crc
 *
 * =17 bytes 90% of bandiwidth
 */

//note usart sends LSB first
//and this CPU is little endian = ls byte lowest address

typedef struct {
	u8 packetType :1; //currently always 0, 1=reserved for future use
	u8 hallSensorState:3;
	u8 packetNum: 2; //counts from 0 to 3 and receiver verifies that no packets missed
	u8 stream2interpreter:2;//0=sm485 buffered 1=stm32 2=sm485 instant (0=highest priority)
	u16 positionFB;
	s16 velocityFB;
	u32 setpoint;
	SMPayloadCommand32 cmdStream2;
	u8 CRCsum;
} __attribute__ ((packed)) GCSendPacket;

typedef struct {
	u8 packetType :4;
	u8 packetNum: 2; //counts from 0 to 3 and receiver verifies that no packets missed
	bool clearController:1;//this bit is 1 if GC requests to reset setpoint counter to zero (i.e. occurs on control mode change, homing done and DIV parameter change)
	bool CRCinvalid:1; //true if CRC is knowingly invalid (happens momentarily during save to flash command). seems to repeat ~30 times during save
	SMPayloadCommandRet32 cmdStream2Ret;
	u8 CRCsum;
} __attribute__ ((packed)) GCRecvPacket;


#define TX_PACKET_SIZE sizeof(GCSendPacket)
#define RX_PACKET_SIZE sizeof(GCRecvPacket)

#define CMD_BUF_SIZE 4
#define CMD_DELAY 3

//decision bytes of what do do for return data once received from GC
#define RET_DISCARD 128
#define RET_NOP 1
#define RET_TO_SYSQUE 2
//#define RET_TO_FASTQUE 3
#define RET_TO_SM485INSTQUE 4
#define RET_TO_SM485BUFQUE 5

#define INTERPRETER_SM_BUFFERED 0
#define INTERPRETER_SYSCMD 1
#define INTERPRETER_SM_INSTANT 2

#define INIT_COMM_TIMEOUT microsecsToTicks(1200000) //threshold of working value 600-700ms
//infinite wait=portMAX_DELAY
//#define INIT_COMM_TIMEOUT portMAX_DELAY
//40ms needed as save to flash causes long break in communication (no packets from GC)
#define OPERATIONAL_COMM_TIMEOUT microsecsToTicks(40000)

#include "DigitalInputPin.h"


/*This is time critical task handling communication between I/O side and GraniteCore
 *
 * Minimum time difference from end of sent packet to beginning of received packet is NNN µs
 * (to give GC time to receive fully the sent packet before it starts processing it).
 * GC Fault location 2321 will indicate high lag from in->out. Compiler optimization sometimes necessary
 * to fullfill the time spec.
 */
void GCCommunicationTask( void *pvParameters )
{
	bool systemInitialized=false;

	GCSendPacket TXpacket;
	GCRecvPacket RXpacket;

	//decision bytes of what do do for return data once received from GC
	RingBuffer cmdStream2RetHanlding(4);
	bool handleRetData=false;

	TXpacket.packetType=0;
	TXpacket.hallSensorState=sys.getCommutationSensorState();
	TXpacket.packetNum=0;
	TXpacket.stream2interpreter=INTERPRETER_SYSCMD;
	TXpacket.positionFB=0;
	TXpacket.velocityFB=0;
	SMPayloadCommand32 cmdNOP;
	cmdNOP.ID=SMPCMD_SET_PARAM_ADDR;
	cmdNOP.param=SMP_ADDR_NOP;
	TXpacket.cmdStream2=cmdNOP;
	TXpacket.setpoint=0;

	//max waiting time for RX data fron DSC, initial value=infinite
	portTickType ReceiveTimeOut=INIT_COMM_TIMEOUT;

	// after entering normal communication mode, serial updates must be streamed at 2500Hz until DSC power off
	// Initialize the xLastWakeTime variable with the current time.
	sys.serialPortGC.armReceiveCompleteEvent(true,RX_PACKET_SIZE);

	GCPSUSetState(true);

	int RXPacketNum=0;

	//xSemaphore is in given state after creation, so take it once to have desired syncing/blocking behavior
	xSemaphoreTake( MCCommTaskSemaphore, 5 );

	DigitalInputPin pin(DigitalInputPin::GPI3_EnableNegFeed, &sys);
	for( ;; )
	{
		sys.physIO.updatePhysInputs();

        /* Block waiting for the semaphore to become available. */

    	//wait for semaphore that is released at DSC receive complete ISR
		if( xSemaphoreTake( MCCommTaskSemaphore, ReceiveTimeOut ) == pdTRUE ) //true=receive complete
        {
			ReceiveTimeOut=OPERATIONAL_COMM_TIMEOUT;//first packet received, from now on we should receive packets at 2500Hz so lower timeout to detect comm break

    		//GC is now operational so let other task to init local params
    		if(systemInitialized==false)
    		{
    			//sys init task is waiting for this to be given
    			xSemaphoreGive(SystemInitLauchSemaphore);
    			systemInitialized=true;
    		}

    		xSemaphoreGive(SystemPeriodicTaskSemaphore);

    		/*
    		 * Handle a packet that is received from GC side (drive DSC)
    		 */

    		int readbytes=sys.serialPortGC.getBuffer((u8*)&RXpacket,RX_PACKET_SIZE);
    		sys.serialPortGC.armReceiveCompleteEvent(true,RX_PACKET_SIZE);

    		//check recv crc
    		u8 rxcrc=CRC8CalcFromBuffer((u8*)&RXpacket,RX_PACKET_SIZE-1,30);
    		if( rxcrc != RXpacket.CRCsum && RXpacket.CRCinvalid==false )
    		{
    			//fault
    			sys.setFault(FLT_GC_COMM,600101);
    		}

    		//clearController bit is 1 if GC requests to reset setpoint counter to zero (i.e. occurs on control mode change, homing done and DIV parameter change)
    		if(RXpacket.clearController)
    			sys.setSerialSetpoint(0);

    		/*start handling cmd return data streams after there has been 3 or more tx packets */
    		if(RXPacketNum>=3)handleRetData=true;
    		if(handleRetData)
    		{
    			bool shouldBeDiscarded;


    			//get info of how to handle this return data packet from GC stream 2
    			u8 packetDestination=cmdStream2RetHanlding.get();
    			if(packetDestination&RET_DISCARD)
    				shouldBeDiscarded=true;
    			else
    				shouldBeDiscarded=false;
    			packetDestination=packetDestination&(~RET_DISCARD);//mask discard bit out


    			//get stream 2 handle instructions from FIFO
    			switch(packetDestination)
    			{
    			case RET_TO_SM485INSTQUE:
    				sys.GCCmdStream2_LowPriority.pushAnswer(RXpacket.cmdStream2Ret,shouldBeDiscarded);
    				break;
    			case RET_TO_SM485BUFQUE:
    				sys.GCCmdStream2_HighPriority.pushAnswer(RXpacket.cmdStream2Ret,shouldBeDiscarded);
    				break;
    			case RET_TO_SYSQUE:
    				sys.GCCmdStream2_MediumPriority.pushAnswer(RXpacket.cmdStream2Ret,shouldBeDiscarded);
    				break;
    			case RET_NOP:
    				break;

    			//case RET_DISCARD:
    			default:
    				sys.setFault(FLT_FIRMWARE,600103);
    				break;
    			}
    		}

    		RXPacketNum++;

    		/*
    		 * Make packet that is being sent to GC side (drive DSC)
    		 */


    		/* if CMD is inserted here, return data is valid after 3 cycles (this+2 latency cycles).
    		 * cycle 0 = reset, very first RX
    		 *
    		 * cycle 0. insert cmd1 return data = 0
    		 * cycle 1. insert cmd2 return data = 0
    		 * cycle 2. insert cmd3 return data = 0
    		 * cycle 3. insert cmd4 return data of cmd1 now valid
    		 * cycle 4. insert cmd5 return data of cmd2 now valid
    		 * etc
    		 */

    		//static int sysCmdsDelayed=0;//give change for syscmds to run even if buffered cmd wants to take all bandwidth

    		bool empty;
    		if( sys.GCCmdStream2_HighPriority.numberOfCommandPacketsWaiting() )
    		{
    			SMPayloadCommandForQueue txCmd=sys.GCCmdStream2_HighPriority.popNextPendingCommand(empty);
    			TXpacket.cmdStream2.ID=txCmd.ID;//found cmd, copy
    			TXpacket.cmdStream2.param=txCmd.param;//found cmd, copy
    			TXpacket.stream2interpreter=INTERPRETER_SM_BUFFERED;
    			if(txCmd.discardRetData)
    				cmdStream2RetHanlding.put(RET_TO_SM485BUFQUE|RET_DISCARD);
    			else
    				cmdStream2RetHanlding.put(RET_TO_SM485BUFQUE);
    		}
    		else if( sys.GCCmdStream2_MediumPriority.numberOfCommandPacketsWaiting() )
    		{
    			SMPayloadCommandForQueue txCmd=sys.GCCmdStream2_MediumPriority.popNextPendingCommand(empty);
    			TXpacket.cmdStream2.ID=txCmd.ID;//found cmd, copy
    			TXpacket.cmdStream2.param=txCmd.param;//found cmd, copy
    			TXpacket.stream2interpreter=INTERPRETER_SYSCMD;
    			if(txCmd.discardRetData)
    				cmdStream2RetHanlding.put(RET_TO_SYSQUE|RET_DISCARD);
    			else
    				cmdStream2RetHanlding.put(RET_TO_SYSQUE);
    		}
    		else if( sys.GCCmdStream2_LowPriority.numberOfCommandPacketsWaiting() )
    		{
    			SMPayloadCommandForQueue txCmd=sys.GCCmdStream2_LowPriority.popNextPendingCommand(empty);
    			TXpacket.cmdStream2.ID=txCmd.ID;//found cmd, copy
    			TXpacket.cmdStream2.param=txCmd.param;//found cmd, copy
    			TXpacket.stream2interpreter=INTERPRETER_SM_INSTANT;
    			if(txCmd.discardRetData)
    				cmdStream2RetHanlding.put(RET_TO_SM485INSTQUE|RET_DISCARD);
    			else
    				cmdStream2RetHanlding.put(RET_TO_SM485INSTQUE);
    		}
    		else// NOP
    		{
    			TXpacket.stream2interpreter=INTERPRETER_SM_INSTANT;
    			TXpacket.cmdStream2=cmdNOP;//no cmds
				cmdStream2RetHanlding.put(RET_NOP|RET_DISCARD);
    		}


    		/*insert other data to tx RXPacketNum */
    		sys.encoder.update();
    		TXpacket.positionFB=sys.getPositionFeedbackValue();
    		TXpacket.velocityFB=sys.getVelocityFeedbackValue();
    		TXpacket.hallSensorState=sys.getCommutationSensorState();
    		TXpacket.setpoint=sys.getInputReferenceValue();
    		TXpacket.CRCsum=CRC8CalcFromBuffer((u8*)&TXpacket,TX_PACKET_SIZE-1,30);
    		sys.serialPortGC.putBuffer((u8*)&TXpacket,TX_PACKET_SIZE);
    		sys.serialPortGC.sendBuffer();
    		TXpacket.packetNum++;

    		//commented out as now it's called in main.cpp task
    		//sys.SMComm.incrementSmBusClock( 4 ); //4 because we call it at 2500Hz freq and SM clock has freq of 10kHz

        }
		else//receive timeouted
		{
			//check if GC bootloader reported FW checksum error
			bool empty=false;
			if(sys.serialPortGC.getByte(empty)==0x67)
			{
				sys.setFault(FLT_GC_COMM|FLT_FIRMWARE,600106);//TODO add unique blinking seq for this
			}

			//reset receiver state, useful only for DSC code degugging/reset etc
			//max waiting time for RX data fron DSC, initial value=infinite
			ReceiveTimeOut=INIT_COMM_TIMEOUT;
			RXPacketNum=0;
			TXpacket.packetNum=0;
			sys.serialPortGC.armReceiveCompleteEvent(true,RX_PACKET_SIZE);
			sys.setFault(FLT_GC_COMM,600105);
			vTaskSuspend(GCCommTaskHandle);//failed badly, no handler for retry so stay here
			systemInitialized=false;
		}
	}
}
