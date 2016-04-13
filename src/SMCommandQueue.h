/*
 * SMCommandQueue.h
 *
 *  Created on: Jan 4, 2012
 *      Author: tero
 */

#ifndef SMCOMMANDQUEUE_H_
#define SMCOMMANDQUEUE_H_


#include "FreeRTOS.h"
#include "queue.h"
#include "types.h"
#include "simplemotion_defs.h"
#include "SMCommandInterpreter.h"
#include "semphr.h"


class System;

class SMCommandQueue
{
public:
	SMCommandQueue( System *parent, int size, const char *name );
	virtual ~SMCommandQueue();

	enum CmdLength {Len24=SMPCMD_24B, Len32=SMPCMD_32B};

	//for internal paramter get/set
	bool setParamCommand( u16 paramID, u32 paramValue, CmdLength len );
	s32 getReturnValue( bool &fail );

	//return false if queue full
	bool sendCommand( SMPayloadCommandForQueue cmd );
	bool receiveReturnPacket( SMPayloadCommandRet32 &ret);

	//SM485 byte stream command handling
	//bool interpretSingleSMCommandByte( u8 inbyte );
	//int getReturnPacket( u8 *outbuf, int *writtenbytes );

	//used only in GC communications task
	SMPayloadCommandForQueue popNextPendingCommand( bool &empty );
	//SMPayloadCommand32 popNextPendingCommand(bool & empty);

	/* GC comm task calls this on each SM queue to store answers from GC to STM side.
	 * This method also executes the given command locally on STM side so it must be called even
	 * when results will be discarded.
	 * discardResults=true will not store results to result queue
	 */
	bool pushAnswer(SMPayloadCommandRet32 ret, bool discardResult );

	//bool pushAnswer( SMPayloadCommandRet32 ret );

	int numberOfReturnPacketsWaiting();
	int numberOfCommandPacketsWaiting();

	void lockMutex();
	void unlockMutex();

	//set ingoring true on high prio stream 1 where other stream setpoints are forwarded. note GC will ignore all setpoints from other streams except from the one where this is set true.
    void setIgnoreSetpointCommands(bool on)
    {
    	localInterpreter.setIgnoreSetpointCommands(on);
    }

private:
	u16 currentParamID;
	xQueueHandle cmdQueue, retQueue;

	xQueueHandle localCmdQueue,localRetQueue;//4 deep queue to match GC side delay on local commands

	//SM cmd interpterer
	u8 inBuf[4];
	int inBytes;
	System *parentSystem;
	SMCommandInterpreter localInterpreter;

	//mutex to prevent multiple sources writing commands simutaneously to avoid param addr confusion
	xSemaphoreHandle mutex;
};

#endif /* SMCOMMANDQUEUE_H_ */
