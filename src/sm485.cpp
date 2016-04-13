#include "sm485.h"
#include "Serial.h"
#include "utils.h"
#include "types.h"
#include "globals.h"
#include "simplemotion_defs.h"
#include "System.h"

//#define bufferedCmdPeriod smRegister[SMREGISTER_BUFFERED_CMD_PERIOD]

//NOTE: these don't support accessing in 1 byte resolution on 56800 cpu! pos rounds to 2 bytes
#define bufput32bit(buf, pos, val) *((u32*)(u8*)((buf)+(pos)))=((u32)(val))
#define bufput16bit(buf, pos, val) *((u16*)(u8*)((buf)+(pos)))=((u16)(val))
#define bufget32bit(buf, pos) (*((u32*)(u8*)((buf)+(pos))))
#define bufget16bit(buf, pos) (*((u16*)(u8*)((buf)+(pos))))

#define txbyte(data) comm->putByte(data)

SimpleMotionComm::SimpleMotionComm( Serial *port, System *parent, int nodeAddress )
/*:
	localInstantCmdInterpreter(parent),
	localBufferedCmdInterpreter(parent)*/
{
	parentSystem=parent;
	//physicalIO=&parent->physIO;
	comm = port;
	userCmds.allocate( CMDBUFSIZE );
	userCmdRets.allocate( CMDBUFSIZE );
	myAddress = nodeAddress;
	cmdClock = 0;
	setBusTimeout(1000);//default 0.1sec
	setBusBufferedCmdPeriod(100);//default 1/100s
	bufferedCmdStatus=SM_BUFCMD_STAT_IDLE;
	setBusBaudRate(460800);
	setBusMode(1);
	setSMBusFaultBehavior(0);
	resetReceiverState();
	receptionBeginTime=0;

	//create queue
	localInstantCmdDelayQueue = xQueueCreate( 4, sizeof( SMPayloadCommandForQueue ) );
	//localInstantCmdDelayQueue = xQueueCreate( 10, sizeof( SMPayloadCommandForQueue ) );
	//set Q naming for kernel aware debugging, otherwise useless:
	vQueueAddToRegistry(localInstantCmdDelayQueue,(signed char*)"485InstDlyQ");
	//create queue
	localBufferedCmdDelayQueue= xQueueCreate( 10, sizeof( SMPayloadCommandForQueue ) );
	//set Q naming for kernel aware debugging, otherwise useless:
	vQueueAddToRegistry(localBufferedCmdDelayQueue,(signed char*)"485BufDlyQ");

    payloadIn.allocate(PAYLOAD_BUFSIZE);
    payloadOut.allocate(PAYLOAD_BUFSIZE);

    mutex = xSemaphoreCreateMutex();

    if( mutex == NULL )
    {
    	SMfault(FLT_SM485_ERROR|FLT_FIRMWARE|FLT_ALLOC,480101);
    }

    bufferMutex = xSemaphoreCreateMutex();

    if(bufferMutex==NULL)
    {
    	SMfault( FLT_SM485_ERROR|FLT_FIRMWARE|FLT_ALLOC,480102);
    }
    vSemaphoreCreateBinary( SimpleMotionBufferedTaskSemaphore );

    if(SimpleMotionBufferedTaskSemaphore==NULL)
    {
    	SMfault( FLT_SM485_ERROR|FLT_FIRMWARE|FLT_ALLOC,480103);
    }

    lastReceivedValidPacketTimestamp=parentSystem->getTimeMicrosecs();;
}

SimpleMotionComm::~SimpleMotionComm()
{
}

#if PROCIMG
#define First 10
#define Middle 11
#define Last 12
#define txbyte_w_loopback(byte) { txbyte(byte); UART_PushToRXbuffer(byte); }
//void makeProcImageReturnPacket( enum ProcessImageReplyPosition my_position )
void makeProcImageReturnPacket( int my_position )
{
	static int cc=0;
	if(my_position==First)
	{
		/*if(cc)asm(debughlt);
		 cc++;*/
		txbyte_w_loopback(SMCMD_PROCESS_IMAGE_RET);
		txbyte_w_loopback(smRegister[SMREGISTER_PROC_RETURN_DATA_LEN]);
		//txbyte_w_loopback(config.deviceAddress);
		txbyte_w_loopback(0);
		//return;
	}

	//if(recv_proc_returndata_sent==0)
	//if(my_position==Middle || my_position==Last )

	if(my_position==Last)
	{
		//add crc16, this is not exactly valid for checksum because its caluclated by receiver (last node), so check also CRC's of return process variables
		txbyte_w_loopback(receivedCRC>>8);
		txbyte_w_loopback(receivedCRC);
		return;
	}

	//send these only if this node is not only one to send data. This prevents sending data twice because First and Last is called. This is only node if LEN==4
	if( (smRegister[SMREGISTER_PROC_RETURN_DATA_LEN]!=4 || my_position==First) )
	{
		txbyte_w_loopback(recv_proc_returndata>>24);
		txbyte_w_loopback(recv_proc_returndata>>16);
		txbyte_w_loopback(recv_proc_returndata>>8);
		txbyte_w_loopback(recv_proc_returndata);
//		recv_proc_returndata_sent=1;//set to avoid sending again in case of single target node in this whole smcmd (when Last is sent)
	}
}
#endif

//create actual SM packet to be sent over bus
void SimpleMotionComm::makeReturnPacket( u8 retid, RingBuffer &returnPayload )
{
	int i;
	u8 data;
	u16 crc = SM_CRCINIT;

	if(receivedAddress==SM_BROADCAST_ADDR) return;//don't send reply to broadcasted packets

	int datalen=returnPayload.bytesAvailable();

	data = retid;
	txbyte( data );
	crc = calcCRC16( data, crc );

	if (retid & SMCMD_MASK_N_PARAMS) //if data len must be inlcuded in return data packet, not coded in ID
	{
		data = datalen;
		txbyte( data );
		crc = calcCRC16( data, crc );
	}

	//fromaddrs
	data = myAddress;
	txbyte( data );
	crc = calcCRC16( data, crc );

	for( i = 0; i < datalen; i++ )
	{
		data = returnPayload.get();
		txbyte( data );
		crc = calcCRC16( data, crc );
	}

	data = crc >> 8;
	txbyte( data );
	data = crc;
	txbyte( data );

	comm->sendBuffer();
}

//create actual SM packet to be sent over bus
void SimpleMotionComm::makeReturnPacket( u8 retid, u8 datalen, u8 *cmddata )
{
	int i;
	u8 data;
	u16 crc = SM_CRCINIT;

	data = retid;
	txbyte( data );
	crc = calcCRC16( data, crc );

	if (retid & SMCMD_MASK_N_PARAMS) //if data len must be inlcuded in return data packet, not coded in ID
	{
		data = datalen;
		txbyte( data );
		crc = calcCRC16( data, crc );
	}

	//fromaddrs
	data = myAddress;
	txbyte( data );
	crc = calcCRC16( data, crc );

	for( i = 0; i < datalen; i++ )
	{
		data = cmddata[i];
		txbyte( data );
		crc = calcCRC16( data, crc );
	}

	data = crc >> 8;
	txbyte( data );
	data = crc;
	txbyte( data );

	comm->sendBuffer();
}

//increment internal clock
void SimpleMotionComm::incrementSmBusClock( int cycles )
{
	xSemaphoreTake( mutex,portMAX_DELAY );
	smBusClock+=cycles;

	//let SM buffered cmd taks run one cycle
	xSemaphoreGive(SimpleMotionBufferedTaskSemaphore);

	xSemaphoreGive(mutex);
}

//for 2 byte payload
void SimpleMotionComm::makeReturnPacket( u8 retid, u8 byte1, u8 byte2 )
{
	u8 retbuffer[2];
	retbuffer[0] = byte1;
	retbuffer[1] = byte2;
	makeReturnPacket( SMCMD_ERROR_RET, 2, retbuffer );
}



#if PROCIMG
u32 executeCmd_w_return_CRC( u8 cmdid, u16 data )
{
	return parseCmd( cmdid, data, 0x1010 ); //0x1010 as CRC skips crc check, output is shifted by 8 to omit return CRC
}
#endif

//called periodically to feed new command from buffer to GC
bool SimpleMotionComm::executeBufferedCmd( bool &executedSetpointCommand )
{
//	if(getbytesinbuf(&cmdhead,&cmdtail,CMDBUFSIZE) < 3 ) 
//		return -1; //error, no enough data in buffer. buffer underflow. TODO aseta joku status?

	SMPayloadCommandForQueue newcmd;
	bool notEnoughData=false;
	extractSMPayloadCommandFromBuffer( userCmds, newcmd, notEnoughData);

	if( notEnoughData )
	{
		bufferedCmdStatus|=SM_BUFCMD_STAT_IDLE|SM_BUFCMD_STAT_UNDERRUN; //set idle bit
		bufferedCmdStatus&=~SM_BUFCMD_STAT_RUN;//clear run bit
		abortBufferedCmdExecution();
		return false;//failed because buffer underrun
	}

	//fetch ok, insert to queue
	parentSystem->SM_BUFFERED_CMD_QUEUE.sendCommand(newcmd);
//	xQueueSend(localBufferedCmdDelayQueue, &newcmd, portMAX_DELAY);//queue local commands to have same delay as GC commands (get reply same time)

	return true; //no error
}

//abort buffered cmd execution and return reference to safe value. call for example by user stop command or if comm error occurs (SM watchdog, buffer underrun etc)
void SimpleMotionComm::abortBufferedCmdExecution()
{
	userCmds.clear();//remove remainging commands

	//set reference to safe value (torque & velocity = 0)
	if(parentSystem->getControlMode()!=System::Position)//in position control leave last value active to avoid unexpected jump
		parentSystem->setParameter(SMP_ABSOLUTE_POS_TARGET,0, parentSystem->SM_BUFFERED_CMD_QUEUE);
}

/*executes command from user cmd buffer, executedSetpointCommand is set true if commadn that was executed
 * modifies setpoints. used to allow executing these fast with infinite frequency between setpoint cmds.*/
void SimpleMotionComm::bufferedCmdUpdate( bool &executedSetpointCommand )
{
	s16 diff;

	if( bufferedCmdStatus&SM_BUFCMD_STAT_RUN )
	{
		xSemaphoreTake( mutex,portMAX_DELAY ); //clock can be modified from other mc communication thread too, so lock mutex

		diff = (s16) ((u16) smBusClock - (u16) cmdClock);//time difference since last executed smpcmd

		//if too long delay from last executed cmd or not active, reset clock
		if (diff > 30000 ) //clock will overflow soon, so stop
		{
			//deactivate
			bufferedCmdStatus|=SM_BUFCMD_STAT_IDLE; //set idle bit
			bufferedCmdStatus&=~SM_BUFCMD_STAT_RUN;//clear run bit
			cmdClock = smBusClock;
		}
		xSemaphoreGive( mutex );//unlock mutex

		if (diff > 0)//its time to execute a command from buffer
		{
			if(executeBufferedCmd(executedSetpointCommand)==false)//returns false if buffer underrun
			{
				//now executeBufferedCmd aborts buffered cmd mode in case of buffer underrun

				//old way
				//bufferedCmdStatus|=SM_BUFCMD_STAT_UNDERRUN;//doesnt stop run mode. its stopped only if diff>30k so 3 secs data break allowed and it continues automatically
			}
			if(executedSetpointCommand)
				cmdClock += busBufferedCmdPeriod;//execute non setpoint commands fast
		}
	}

	//get return data if any available
	while(parentSystem->SM_BUFFERED_CMD_QUEUE.numberOfReturnPacketsWaiting())
	{

#ifdef refraktorointi2
		bool success;
		SMPayloadCommandRet32 retFromGC,finalRet;
		SMPayloadCommandForQueue newcmd;

		//execute same command locally (get from queue)
		xQueueReceive(localBufferedCmdDelayQueue, &newcmd, 0);//get from queue
		localBufferedCmdInterpreter.executeHostSideGlobalSetParamCommand(newcmd);//interpret local commands

		//get smpret from GC
		success=parentSystem->GCCmdStream1_HighPriority.receiveReturnPacket(retFromGC);//receive return packet from GC

		//get local smpret. this call will either place retFromGC to ret or its own return value (overriding GC value)
		localBufferedCmdInterpreter.executeHostSideGlobalGetParamCommand(finalRet,retFromGC);

		if(success==true)
			insertSMPayloadRetToBuffer(userCmdRets,finalRet);//put into smpret buffer
#else
		bool success;
		SMPayloadCommandRet32 retFromGC;
//		SMPayloadCommandForQueue newcmd;

		//execute same command locally (get from queue)
//		xQueueReceive(localBufferedCmdDelayQueue, &newcmd, 0);//get from queue
//		localBufferedCmdInterpreter.executeHostSideGlobalSetParamCommand(newcmd);//interpret local commands

		//get smpret from GC
		success=parentSystem->SM_BUFFERED_CMD_QUEUE.receiveReturnPacket(retFromGC);//receive return packet from GC

		//get local smpret. this call will either place retFromGC to ret or its own return value (overriding GC value)
//		localBufferedCmdInterpreter.executeHostSideGlobalGetParamCommand(finalRet,retFromGC);

		if(success==true)
			insertSMPayloadRetToBuffer(userCmdRets,retFromGC);//put into smpret buffer
		else
			parentSystem->setFault(FLT_FIRMWARE,480201);

#endif

	}
}

//returns number of bytes used and stores SMP command to newcmd
int SimpleMotionComm::extractSMPayloadCommandFromBuffer( RingBuffer &buffer, SMPayloadCommandForQueue &newcmd, bool &notEnoughBytes )
{
	bool readfail=false;
	u8 readbyte=buffer.peek(readfail);
	u8 cmdID = readbyte >> 6; //extract packet header 2 bits

	if(readfail)
	{
		notEnoughBytes=true;
		return 0;
	}

	if (cmdID == SMPCMD_24B)
	{
		if(buffer.bytesAvailable()<3)
		{
			notEnoughBytes=true;
			return 0;
		}
		//extract return packet and convert to 32 bit and return
		SMPayloadCommand24 read;
		u8 *readBuf = (u8*) &read;
		readBuf[2] = buffer.get();
		readBuf[1] = buffer.get();
		readBuf[0] = buffer.get();
		newcmd.param = read.param;
		newcmd.ID = read.ID;
		newcmd.discardRetData = false;
		return 3;
	}
	else if (cmdID == SMPCMD_32B)
	{
		if(buffer.bytesAvailable()<4)
		{
			notEnoughBytes=true;
			return 0;
		}
		//extract return packet and convert to 32 bit and return
		SMPayloadCommand32 read;
		u8 *readBuf = (u8*) &read;
		readBuf[3] = buffer.get();
		readBuf[2] = buffer.get();
		readBuf[1] = buffer.get();
		readBuf[0] = buffer.get();
		newcmd.param = read.param;
		newcmd.ID = read.ID;
		newcmd.discardRetData = false;
		return 4;
	}
	else if (cmdID == SMPCMD_SET_PARAM_ADDR) //16bit
	{
		if(buffer.bytesAvailable()<2)
		{
			notEnoughBytes=true;
			return 0;
		}
		//extract return packet and convert to 32 bit and return
		SMPayloadCommand16 read;
		u8 *readBuf = (u8*) &read;
		readBuf[1] = buffer.get();
		readBuf[0] = buffer.get();

		newcmd.param = read.param;
		newcmd.ID = read.ID;
		newcmd.discardRetData = false;
		return 2;
	}
	else
	{
		SMfault( FLT_SM485_ERROR, 480301 );
		return 0;
	}
}

//return number of bytes written
int SimpleMotionComm::insertSMPayloadRetToBuffer( RingBuffer &buffer,
		SMPayloadCommandRet32 ret )
{
	if (ret.ID == SMPRET_32B)
	{
		if (buffer.bytesFree() < 4)
		{
			SMfault( FLT_SM485_ERROR, 480401 );
			//tx payload buffer full
			return 0;
		}
		SMPayloadCommandRet32 send;
		send.ID = ret.ID;
		send.retData = ret.retData;
		u8 *sendBuf = (u8*) &send;
		buffer.put(sendBuf[3]);
		buffer.put(sendBuf[2]);
		buffer.put(sendBuf[1]);
		buffer.put(sendBuf[0]);
		return 4;
	}
	if (ret.ID == SMPRET_24B)
	{
		if (buffer.bytesFree() < 3)
		{
			SMfault( FLT_SM485_ERROR, 480402 );
			//tx payload buffer full
			return 0;
		}
		SMPayloadCommandRet24 send;
		send.ID = ret.ID;
		send.retData = ret.retData;
		u8 *sendBuf = (u8*) &send;
		buffer.put(sendBuf[2]);
		buffer.put(sendBuf[1]);
		buffer.put(sendBuf[0]);
		return 3;
	}
	if (ret.ID == SMPRET_16B)
	{
		if (buffer.bytesFree() < 2)
		{
			SMfault( FLT_SM485_ERROR, 480403 );
			//tx payload buffer full
			return 0;
		}
		SMPayloadCommandRet16 send;
		send.ID = ret.ID;
		send.retData = ret.retData;
		u8 *sendBuf = (u8*) &send;
		buffer.put(sendBuf[1]);
		buffer.put(sendBuf[0]);
		return 2;
	}
	if (ret.ID == SMPRET_CMD_STATUS)
	{
		if (buffer.bytesFree() < 1)
		{
			SMfault( FLT_SM485_ERROR, 480404 );
			//tx payload buffer full
			return 0;
		}
		SMPayloadCommandRet8 send;
		send.ID = ret.ID;
		send.retData = ret.retData;
		u8 *sendBuf = (u8*) &send;
		buffer.put(sendBuf[0]);
		return 1;
	}

	SMfault( FLT_SM485_ERROR, 480405 );
	//cant reach this ever
	return 0;
}

void SimpleMotionComm::startProcessingBufferedCommands()
{
	//activate
	if(!(bufferedCmdStatus&SM_BUFCMD_STAT_RUN))
	{
		 cmdClock=smBusClock; //sync clocks
		 bufferedCmdStatus=SM_BUFCMD_STAT_RUN;//clears errors too
	}
}

void SimpleMotionComm::executeSMcmd()
{
	int rxPos, txPos;

	//s16 temp16;
	switch (receivedCMDID)
	{
#if PROCIMG
		case SMCMD_PROCESS_IMAGE:
		{
			u8 spicmd;
			u16 spiparam;

			debuggi=1;
			//if offset is too far, variable not found to be found in payload data
			if( smRegister[SMREGISTER_PROC_VARIABLE_OFFSET]+3 > receivedPayloadSize )//in this device process variable is 3 bytes and return data 4 bytes
			{
				setFault(FLT_SM485_ERROR);
				makeReturnPacket2B(SMCMD_ERROR_RET,0,SMERR_INVALID_PARAMETER);
			}
			else // execute cmd
			{
				spicmd=receivedPayloadBuf[smRegister[SMREGISTER_PROC_VARIABLE_OFFSET]];
				spiparam=(u16)(receivedPayloadBuf[smRegister[SMREGISTER_PROC_VARIABLE_OFFSET]+2]<<8)|receivedPayloadBuf[smRegister[SMREGISTER_PROC_VARIABLE_OFFSET]+1];

				//execute and store result data
				recv_proc_returndata=executeCmd_w_return_CRC(spicmd,spiparam);
				//recv_proc_returndata_sent=0;

				//if we are first to make reply packet, then begin replying
				if(smRegister[SMREGISTER_PROC_RETURN_OFFSET]==0)
				makeProcImageReturnPacket(First);
			}
		}
		break;
#endif

	case SMCMD_BUFFERED_CMD:
	{
		xSemaphoreTake(bufferMutex,portMAX_DELAY);

		if (userCmds.bytesFree() < receivedPayloadSize)
		{
			SMfault( FLT_SM485_ERROR, 480501 );
			makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
		}
		else
		{
			//copy payload buffer to to cmd buffer
			while(payloadIn.bytesAvailable())
			{
				if( userCmds.put( payloadIn.get() ))
				{
					bufferedCmdStatus|=SM_BUFCMD_STAT_OVERRUN;
				}
			}

			s16 retbytes = userCmdRets.bytesAvailable();
			if (retbytes > MAX_PAYLOAD_BYTES)
				retbytes = MAX_PAYLOAD_BYTES;

			//copy SMP cmd return data to returning payloadbuf
			for(txPos=0;txPos<retbytes;txPos++)
			{
				payloadOut.put(userCmdRets.get());
			}

			//activate
			//now clock sync activates
			/*if(!(bufferedCmdStatus&SM_BUFCMD_STAT_RUN))
			{
				 cmdClock=smBusClock; //sync clocks
				 bufferedCmdStatus=SM_BUFCMD_STAT_RUN;//clears errors too
			}*/

			makeReturnPacket( SMCMD_BUFFERED_CMD_RET, payloadOut );
		}
		xSemaphoreGive(bufferMutex);
	}
	break;

	case SMCMD_INSTANT_CMD:
	{
#if 0
		rxPos = 0;
		txPos = 0;
		int rxDone=0; //numb of cmds executed
		int txDone=0; //cmdrets
		bool getRetValuesNow=false;

		/* TODO
		 * stream2:lle 3 prioriteettia ja quea: smbuffered, sys, sminstant
		 * hyv� paikka stm smp interpreterille quessa, ohittaa gc:n jos oma parsku
		 * joku erroribitti paluupakettiin jos sendbuf overrun tms
		 */
		while (rxPos < receivedPayloadSize || rxDone>txDone ) //loop until all send & received
		{
			SMPayloadCommandForQueue newcmd;
			SMPayloadCommandRet32 retFromGC;
			SMPayloadCommandRet32 ret;
			bool notenoughdata=false;

			if( rxPos < receivedPayloadSize ) //execute commands
			{
				rxPos += extractSMPayloadCommandFromBuffer( payloadIn, newcmd, notenoughdata );

				if(rxPos>=PAYLOAD_BUFSIZE)//command corrupted because it has overflown payload buf size
				{
					SMfault( FLT_SM485_ERROR, 48520 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
					return;
				}
				else//SMP cmd received ok, execute
				{
					xQueueSend(localInstantCmdDelayQueue, &newcmd, portMAX_DELAY);//queue local commands to have same delay as GC commands (get reply same time)
					SM_INSTANT_CMD_QUEUE.sendCommand(newcmd); //send command to GC side
					rxDone++;
					if(rxDone>=4)
						getRetValuesNow=true;
				}
			}
			else
				getRetValuesNow=true; //needed here if theres less than 4 packets in payloadIn


			//driveSM485CmdQueue.numberOfReturnPacketsWaiting() method doesnt work because it some times return 0 because this loops faster than GD sends replies
			if(getRetValuesNow) //receive return packets after 4 cycles delay
			{
				xQueueReceive(localInstantCmdDelayQueue, &newcmd, 0);//get from queue
				localInstantCmdInterpreter.executeHostSideGlobalSetParamCommand(newcmd);//interpret local commands
				SM_INSTANT_CMD_QUEUE.receiveReturnPacket(retFromGC); //receive return packet from GC

				//this call will either place retFromGC to ret or its own return value (overriding GC value)
				localInstantCmdInterpreter.executeHostSideGlobalGetParamCommand(ret,retFromGC);
				//txPos += insertSMPayloadRetToPayloadBuffer( transmittedPayloadBuf, ret, txPos, PAYLOAD_BUFSIZE );
				int bytesStored=insertSMPayloadRetToBuffer( payloadOut, ret );
				txPos += bytesStored;
				if(bytesStored==0)//buffer full
				{
					SMfault( FLT_SM485_ERROR, 48526 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
				}
				txDone++;
			}
		}
#endif
#ifdef refraktorointi2
		rxPos = 0;
		txPos = 0;
		int rxDone=0; //numb of cmds executed
		int txDone=0; //cmdrets
		bool getRetValuesNow=false;

		/* TODO
		 * stream2:lle 3 prioriteettia ja quea: smbuffered, sys, sminstant
		 * hyv� paikka stm smp interpreterille quessa, ohittaa gc:n jos oma parsku
		 * joku erroribitti paluupakettiin jos sendbuf overrun tms
		 */
		while (rxPos < receivedPayloadSize || rxDone>txDone ) //loop until all send & received
		{
			SMPayloadCommandForQueue newcmd;
			SMPayloadCommandRet32 retFromGC;
			SMPayloadCommandRet32 ret;
			bool notenoughdata=false;

			if( rxPos < receivedPayloadSize ) //execute commands
			{
				rxPos += extractSMPayloadCommandFromBuffer( payloadIn, newcmd, notenoughdata );

				if(rxPos>=PAYLOAD_BUFSIZE)//command corrupted because it has overflown payload buf size
				{
					SMfault( FLT_SM485_ERROR, 48520 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
					return;
				}
				else//SMP cmd received ok, execute
				{
					xQueueSend(localInstantCmdDelayQueue, &newcmd, portMAX_DELAY);//queue local commands to have same delay as GC commands (get reply same time)
					SM_INSTANT_CMD_QUEUE.sendCommand(newcmd); //send command to GC side
					rxDone++;
					if(rxDone>=4)
						getRetValuesNow=true;
				}
			}
			else
				getRetValuesNow=true; //needed here if theres less than 4 packets in payloadIn


			//driveSM485CmdQueue.numberOfReturnPacketsWaiting() method doesnt work because it some times return 0 because this loops faster than GD sends replies
			if(getRetValuesNow) //receive return packets after 4 cycles delay
			{
				xQueueReceive(localInstantCmdDelayQueue, &newcmd, 0);//get from queue
				localInstantCmdInterpreter.executeHostSideGlobalSetParamCommand(newcmd);//interpret local commands
				SM_INSTANT_CMD_QUEUE.receiveReturnPacket(retFromGC); //receive return packet from GC

				//this call will either place retFromGC to ret or its own return value (overriding GC value)
				localInstantCmdInterpreter.executeHostSideGlobalGetParamCommand(ret,retFromGC);
				//txPos += insertSMPayloadRetToPayloadBuffer( transmittedPayloadBuf, ret, txPos, PAYLOAD_BUFSIZE );
				int bytesStored=insertSMPayloadRetToBuffer( payloadOut, ret );
				txPos += bytesStored;
				if(bytesStored==0)//buffer full
				{
					SMfault( FLT_SM485_ERROR, 48526 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
				}
				txDone++;
			}
		}
#else
		rxPos = 0;
		txPos = 0;
		int rxDone=0; //numb of cmds executed
		int txDone=0; //cmdrets
		bool getRetValuesNow=false;

		/* TODO
		 * stream2:lle 3 prioriteettia ja quea: smbuffered, sys, sminstant
		 * hyv� paikka stm smp interpreterille quessa, ohittaa gc:n jos oma parsku
		 * joku erroribitti paluupakettiin jos sendbuf overrun tms
		 */
		while (rxPos < receivedPayloadSize || rxDone>txDone ) //loop until all send & received
		{
			SMPayloadCommandForQueue newcmd;
			SMPayloadCommandRet32 retFromGC;
			bool notenoughdata=false;

			if( rxPos < receivedPayloadSize ) //execute commands
			{
				rxPos += extractSMPayloadCommandFromBuffer( payloadIn, newcmd, notenoughdata );

				if(rxPos>=PAYLOAD_BUFSIZE)//command corrupted because it has overflown payload buf size
				{
					SMfault( FLT_SM485_ERROR, 480502 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
					return;
				}
				else//SMP cmd received ok, execute
				{
	//				xQueueSend(localInstantCmdDelayQueue, &newcmd, portMAX_DELAY);//queue local commands to have same delay as GC commands (get reply same time)
					parentSystem->SM_INSTANT_CMD_QUEUE.sendCommand(newcmd); //send command to GC side
					rxDone++;
					if(rxDone>=4)
						getRetValuesNow=true;
				}
			}
			else
				getRetValuesNow=true; //needed here if theres less than 4 packets in payloadIn


			//driveSM485CmdQueue.numberOfReturnPacketsWaiting() method doesnt work because it some times return 0 because this loops faster than GD sends replies
			if(getRetValuesNow) //receive return packets after 4 cycles delay
			{
//				xQueueReceive(localInstantCmdDelayQueue, &newcmd, 0);//get from queue
				//localInstantCmdInterpreter.executeHostSideGlobalSetParamCommand(newcmd);//interpret local commands
				parentSystem->SM_INSTANT_CMD_QUEUE.receiveReturnPacket(retFromGC); //receive return packet from GC

				//this call will either place retFromGC to ret or its own return value (overriding GC value)
//				localInstantCmdInterpreter.executeHostSideGlobalGetParamCommand(ret,retFromGC);
				//txPos += insertSMPayloadRetToPayloadBuffer( transmittedPayloadBuf, ret, txPos, PAYLOAD_BUFSIZE );
				int bytesStored=insertSMPayloadRetToBuffer( payloadOut, retFromGC );
				txPos += bytesStored;
				if(bytesStored==0)//buffer full
				{
					SMfault( FLT_SM485_ERROR, 480503 );
					makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_BUF_OVERFLOW );
				}
				txDone++;
			}
		}
#endif
		makeReturnPacket( SMCMD_INSTANT_CMD_RET, payloadOut );

	}
	break;

	case SMCMD_ECHO:
	{
		u32 i;
		//copy to cmd buffer
		for( i = 0; i < payloadIn.bytesAvailable(); i++ )
		{
			txbyte( payloadIn.get() );
		}
	}
		break;

	case SMCMD_GET_CLOCK:
	{
		u16 clock=getSmBusClock(0);
		makeReturnPacket( SMCMD_GET_CLOCK_RET, 2, (u8*) &clock );
		startProcessingBufferedCommands();
	}
		break;

	case SMCMD_GET_CLOCK_RET: //this is interpreted by every node (sync clocks), not answered
	{
		u16 newclock=int(payloadIn.get()) | (int(payloadIn.get())<<8);
		setSmBusClock( newclock );
		startProcessingBufferedCommands();
	}
		break;

	default: //error
		SMfault( FLT_SM485_ERROR, 480601 );
		makeReturnPacket( SMCMD_ERROR_RET, receivedCMDID, SMERR_INVALID_CMD );
		break;
	}
}

void SimpleMotionComm::resetReceiverState()
{
	payloadIn.clear();
	payloadOut.clear();
	receivedCRC = SM_CRCINIT;
	receiverNextState = WaitCmdId;

	receivedPayloadSize = -1;
	receivedCMDID = 0; // cmdid=0 kun ed komento suoritettu
	receivedAddress = 255;

	receiverState = WaitCmdId;
	receiverNextState = WaitCmdId;
	receptionBeginTime=0;
}


bool SimpleMotionComm::receptionTimeouted()//return true if reception time has passed
{
	u32 currentTime=parentSystem->getTimeMicrosecs();

	//handles rollover correctly
	s32 timeDiff=s32(currentTime-receptionBeginTime);

	//timeout possible only when not waiting the first byte (cmdid)
	if(timeDiff>s32(busTimeout)*100 && receiverState!=WaitCmdId )
	{
		SMfault(FLT_SM485_ERROR,481201);
		resetReceiverState();
		return true;
	}
	else
		return false;
}

void SimpleMotionComm::watchdogUpdate()
{
	//set fault if watchdog enabled t>=100 and timeout happened
	u32 timenow=parentSystem->getTimeMicrosecs();
	u32 diff=timenow-lastReceivedValidPacketTimestamp;//rollover safe
	u32 watchDogTimeout=(faultBehavior>>8)&1023;//in 10ms steps

	if(watchDogTimeout!=0  && diff>(watchDogTimeout*10000))
	{
		SMfault(FLT_SM485_ERROR,481001);
		NOP;
	}
}

//reset watchdog timer if command received OK
void SimpleMotionComm::watchdogReset()
{
	lastReceivedValidPacketTimestamp=parentSystem->getTimeMicrosecs();
}

//can be called at any frequency
u8 SimpleMotionComm::poll()
{
	u8 data;
	bool empty = false;

	//check for timeout & handle it
	receptionTimeouted();
	watchdogUpdate();

	//read byte from uart
	data = comm->getByte( empty );
	if (empty == true)
		return 0; //no rx data

	//buffered variable allows placing if's in any order
	receiverState = receiverNextState;

	//state machine begins:
	if (receiverState == WaitPayload)
	{
		receivedCRC = calcCRC16( data, receivedCRC );

#if PROCIMG
		//process image return packet special handling because this is sent by multiple nodes together
		if( receivedCMDID==SMCMD_PROCESS_IMAGE_RET )
		{
			if( recvStorePosition==(smRegister[SMREGISTER_PROC_RETURN_DATA_LEN]-1) )
			{
				//its this node's turn to broadcast return data. this is the last node on process image
				makeProcImageReturnPacket(Last);//send just crc
			}
			else if( recvStorePosition==(smRegister[SMREGISTER_PROC_RETURN_OFFSET]-1) )
			{
				//its this node's turn to broadcast return data
				makeProcImageReturnPacket(Middle);
			}
		}
#endif

		//normal handling for all payload data
//		if (recvStorePosition < PAYLOAD_BUFSIZE)
			//receivedPayloadBuf[recvStorePosition++] = data;
			if( payloadIn.put(data) ==false )//if full
//		else
		{
				SMfault( FLT_SM485_ERROR, 480701 );
			if (receivedAddress == myAddress)
			{
				makeReturnPacket( SMCMD_ERROR_RET, 0, SMERR_PAYLOAD_SIZE ); //if this device, report buffer overflow
				resetReceiverState();
			}
		}

		//all received
		if (receivedPayloadSize <= s16(payloadIn.bytesAvailable()))
			receiverNextState = WaitCrcHi;

		return 1;
	}

	if (receiverState == WaitCmdId)//the first byte of packet
	{
		receptionBeginTime=parentSystem->getTimeMicrosecs();//"start" timeout counter

		receivedCRC = calcCRC16( data, receivedCRC );
		receivedCMDID = data;
		switch (data & SMCMD_MASK_PARAMS_BITS)
		//commands with fixed payload size
		{
		case SMCMD_MASK_2_PARAMS:
			receivedPayloadSize = 2;
			receiverNextState = WaitAddr;
			break;
		case SMCMD_MASK_0_PARAMS:
			receivedPayloadSize = 0;
			receiverNextState = WaitAddr;
			break;
		case SMCMD_MASK_N_PARAMS:
			receivedPayloadSize = -1;
			receiverNextState = WaitPayloadSize;
			break; //-1 = N databytes
		default:
			SMfault( FLT_SM485_ERROR, 480801 );
			resetReceiverState();
			//can't answer because all clients would answer same time makeReturnPacket2B(SMCMD_ERROR_RET,0,SMERR_INVALID_CMD);
			break; //error, unsupported command id
		}

		return 1;
	}

	//no data payload size known yet
	if (receiverState == WaitPayloadSize)
	{
		receivedCRC = calcCRC16( data, receivedCRC );
		receivedPayloadSize = data;
		receiverNextState = WaitAddr;
		return 1;
	}

	if (receiverState == WaitAddr)
	{
		receivedCRC = calcCRC16( data, receivedCRC );
		receivedAddress = data; //can be receiver or sender addr depending on cmd
		if (receivedPayloadSize > payloadIn.bytesAvailable())
			receiverNextState = WaitPayload;
		else
			receiverNextState = WaitCrcHi;
		return 1;
	}

	if (receiverState == WaitCrcHi)
	{
		receivedCRCHiByte = data; //crc_msb
		receiverNextState = WaitCrcLo;
		return 1;
	}

	//get crc_lsb, check crc and execute the SM cmd
	if (receiverState == WaitCrcLo)
	{
		if (((receivedCRCHiByte << 8) | data) != receivedCRC)
		{
			//CRC error, no reply because all clients may answer simultaneously
			//makeReturnPacket2B(SMCMD_ERROR_RET,0,SMERR_CRC);
			SMfault( FLT_SM485_ERROR, 480901 );
			resetReceiverState();
		}
		else
		{
			//CRC ok
			if (receivedAddress
					== myAddress || receivedAddress==SM_BROADCAST_ADDR || receivedCMDID==SMCMD_GET_CLOCK_RET || receivedCMDID==SMCMD_PROCESS_IMAGE)
			{
				executeSMcmd();
				watchdogReset();
			}

		}

		//re-init all statics to receive next smcmd
		resetReceiverState();
		return 1;
	}

	return 1;
}

//sets fault and causes device fault stop if fault behavior is set accordingly
void SimpleMotionComm::SMfault(u32 faultbits, u32 faultLocation)
{
	parentSystem->setFault(faultbits,faultLocation);
	//if(faultBehavior&1)
//		parentSys->setStatus(STAT_FAULTSTOP);//cant write it
}


void SimpleMotionComm::loopBackComm()
{
	 //loopback test
	 bool empty = false;
	 u8 c = comm->getByte( empty );
	 if (empty == false)
	 {
		 comm->putByte( c );
		 //if(c=='\r')
		 comm->sendBuffer();
	 }
}




int SimpleMotionComm::getMyAddress(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 1;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 64;
	else
		return myAddress;
}

u16 SimpleMotionComm::getSmBusClock(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 0;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 0xFFFF;
	else
		return smBusClock;
}

void SimpleMotionComm::setMyAddress(int myAddress)
{
    this->myAddress = myAddress;
}

void SimpleMotionComm::setSmBusClock(u16 smBusClock)
{
//	setLed(2,1);
	xSemaphoreTake( mutex,portMAX_DELAY );
    this->smBusClock = smBusClock;
    xSemaphoreGive( mutex );
//    setLed(2,0);
}

s32 SimpleMotionComm::getBusBaudRate(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 1000;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 2000000;
	else
		return busBaudRate;
}

u16 SimpleMotionComm::getBusMode(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 1;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 1;
	else
		return busMode;
}

bool SimpleMotionComm::setBusBaudRate(s32 busBaudRate)
{
	if( comm->setBaudRate(busBaudRate) )//if success
	{
		this->busBaudRate = busBaudRate;
		return true;
	}
	return false;
}

bool SimpleMotionComm::setBusMode(u16 busMode)
{
	if(busMode!=1)//only existng mode
		return false;
	else
		this->busMode = busMode;
	return true;
}

u16 SimpleMotionComm::getBusBufferedCmdPeriod(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 8;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 10000;
	else
		return busBufferedCmdPeriod;
}

u16 SimpleMotionComm::getBusTimeout(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)
		return 1;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 10000;
	else
		return busTimeout;
}

bool SimpleMotionComm::setBusBufferedCmdPeriod(u16 busBufferedCmdPeriod)
{
/*	if(busBufferedCmdPeriod==0)//0 aborts buffered commands
	{
		abortBufferedCmdExecution();
		return true;
	}*/

	if(busBufferedCmdPeriod<8 || busBufferedCmdPeriod>10000) return false;//out of range

	//max buffered command rate is 1250/s to avoid blocking other communications to GC

    this->busBufferedCmdPeriod = busBufferedCmdPeriod;
    return true;
}

bool SimpleMotionComm::setBusTimeout(u16 busTimeout)
{
    this->busTimeout = busTimeout;
    return true;
}

s16 SimpleMotionComm::getBufferedCmdStatus(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK)//when returning min=2 max=1, it means parameter is read-only
		return 2;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 1;
	else
		return bufferedCmdStatus;
}

u32 SimpleMotionComm::getSMBusBufferFreeBytes(u16 attribute) const
{

	if(attribute&SMP_MIN_VALUE_MASK)//when returning min=2 max=1, it means parameter is read-only
		return 2;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 1;
	else
	{
		/* mutex lock not needed because size is atomic integer. value is ok even if updated middle of read */
//		xSemaphoreTake(bufferMutex,portMAX_DELAY);//FIXME jumii tahan jos buffered cmdssa on taa parsku returndatana
		int free=userCmds.bytesFree();
//		xSemaphoreGive(bufferMutex);
		return free;
	}
}


s16 SimpleMotionComm::getSMBusVersion(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK) //when returning min=2 max=1, it means parameter is read-only
		return 2;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 1;
	else
		return SM_VERSION;
}

s16 SimpleMotionComm::getSMBusCompatVersion(u16 attribute) const
{
	if(attribute&SMP_MIN_VALUE_MASK) //when returning min=2 max=1, it means parameter is read-only
		return 2;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return 1;
	else
		return SM_VERSION_COMPAT;
}

u32 SimpleMotionComm::getSMBusFaultBehavior( u16 attribute ) const
{
	if(attribute&SMP_MIN_VALUE_MASK) //when returning min=2 max=1, it means parameter is read-only
		return 0;
	else if(attribute&SMP_MAX_VALUE_MASK)
		return ((1023<<8)|1);//max timout sets the max +1 enable bit //max timout sets the max +1 enable bit //according to spec, bits 18-32 are alwas zero in current SM vesrion
	else
		return faultBehavior;
}

bool SimpleMotionComm::setSMBusFaultBehavior( u32 behabiorbits )
{
	faultBehavior=behabiorbits;
	return true;
}
