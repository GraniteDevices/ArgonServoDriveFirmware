/*
 * SMCommandQueue.cpp
 *
 *  Created on: Jan 4, 2012
 *      Author: tero
 */

#include "SMCommandQueue.h"
#include "globals.h"
#include <stdlib.h>
#include "System.h"
#include "SMCommandInterpreter.h"


SMCommandQueue::SMCommandQueue( System *parent, int size, const char *name):
localInterpreter(parent)
{
	char *namebase=(char*)malloc(20);//FreeRTOS only stores a pointer so must allocate memory for string
	char *namemod=(char*)malloc(20);//FreeRTOS only stores a pointer so must allocate memory for string
	int i;

	parentSystem=parent;

	//copy string
	for(i=0;i<20-4;i++)
	{
		namemod[i]=name[i];
		namebase[i]=name[i];

		if(name[i]==0)
			break;
	}
	//append "Ret" to freertos queue name for return channel
	namemod[i]='R';
	namemod[i+1]='e';
	namemod[i+2]='t';
	namemod[i+3]=0;

	//allocate queues
	cmdQueue = xQueueCreate( size, sizeof( SMPayloadCommandForQueue ) );

	if(cmdQueue==NULL)
		parentSystem->setFault(FLT_ALLOC|FLT_FIRMWARE,710101);

	retQueue = xQueueCreate( size, sizeof( SMPayloadCommandRet32 ) );

	if(retQueue==NULL)
		parentSystem->setFault(FLT_ALLOC|FLT_FIRMWARE,710102);

	//allocate queues
	localCmdQueue = xQueueCreate( 6, sizeof( SMPayloadCommandForQueue ) );

	if(localCmdQueue==NULL)
		parentSystem->setFault(FLT_ALLOC|FLT_FIRMWARE,710103);

	localRetQueue = xQueueCreate( 6, sizeof( SMPayloadCommandRet32 ) );

	if(localRetQueue==NULL)
		parentSystem->setFault(FLT_ALLOC|FLT_FIRMWARE,710104);

    mutex = xSemaphoreCreateMutex();
    if( mutex == NULL )
    {
    	parentSystem->setFault(FLT_SM485_ERROR|FLT_FIRMWARE|FLT_ALLOC,480101);
    }

	//set Q naming for kernel aware debugging, otherwise useless:
	vQueueAddToRegistry(cmdQueue,(signed char*)namebase);
	vQueueAddToRegistry(retQueue,(signed char*)namemod);

	vQueueAddToRegistry(localCmdQueue,(signed char*)"localcmd");
	vQueueAddToRegistry(localRetQueue,(signed char*)"localret");
	currentParamID=0;//no id yet
}

SMCommandQueue::~SMCommandQueue()
{
	vQueueDelete(cmdQueue);
	vQueueDelete(retQueue);
	vSemaphoreDelete(mutex);//danger: dont delete mutex if currently locked

/*	vQueueDelete(localCmdQueue);
	vQueueDelete(localRetQueue);*/
}

int SMCommandQueue::numberOfReturnPacketsWaiting()
{
	return uxQueueMessagesWaiting( retQueue );
}
int SMCommandQueue::numberOfCommandPacketsWaiting()
{
	return uxQueueMessagesWaiting( cmdQueue );
}


bool SMCommandQueue::setParamCommand(u16 paramID, u32 paramValue, CmdLength len )
{
	SMPayloadCommandForQueue newcmd;

	//waits infinitely if q is full

	if(paramID!=currentParamID) //must insert a command to change store param id
	{
		newcmd.ID=SMPCMD_SET_PARAM_ADDR;
		newcmd.param=paramID;
		newcmd.discardRetData=true;
		if( sendCommand(newcmd)==false ) return false;
		currentParamID=paramID;
	}

	newcmd.ID=len;
	newcmd.param=paramValue;
	newcmd.discardRetData=false;
	return sendCommand(newcmd);
}



s32 SMCommandQueue::getReturnValue( bool &fail )
{
	SMPayloadCommandRet32 ret;
	//wait infintely
	if(xQueueReceive(retQueue, &ret, portMAX_DELAY) == pdTRUE )
	{
		return (ret.retData);
	}
	//queue was empty
	fail=true;
	return 0;
}



void setLed(int,int);

SMPayloadCommandForQueue SMCommandQueue::popNextPendingCommand(bool & empty)
{
	SMPayloadCommandForQueue cmd;
	//wait infintely
	if(xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE )
	{
		//execute also locally before giving it to GC comm taks:
		//localInterpreter.executeHostSideGlobalSetParamCommand(cmd);
		//store to queue, this is executed second time when GC comm task pushes answer

		if(xQueueSend(localCmdQueue, &cmd, 0) != pdTRUE )
			parentSystem->setFault(FLT_QUEUE_FULL|FLT_FIRMWARE,710201);

		return (cmd);
	}
	empty=true;
	return cmd;
}
/*
SMPayloadCommand32 SMCommandQueue::popNextPendingCommand(bool & empty)
{
	SMPayloadCommand32 retconv;
	SMPayloadCommandForQueue ret;
	ret=popNextPendingCommand(empty);
	retconv.ID=ret.ID;
	retconv.param=ret.param;
	return retconv;
}
*/
bool SMCommandQueue::sendCommand(SMPayloadCommandForQueue cmd)
{
	//setLed(3,1);
	if(xQueueSend(cmdQueue, &cmd, portMAX_DELAY) == pdFALSE ) //if full
	{
		parentSystem->setFault(FLT_QUEUE_FULL, 710301);
		//setLed(3,0);
		return false;
	}
	else
	{
	//	setLed(3,0);
		return true;
	}
}

bool SMCommandQueue::receiveReturnPacket(SMPayloadCommandRet32 & ret)
{
	//wait infintely
	if(xQueueReceive(retQueue, &ret, portMAX_DELAY) == pdTRUE )
	{
		return true;
	}

	return false;//Q was empty
}

bool SMCommandQueue::pushAnswer(SMPayloadCommandRet32 ret, bool discardResult )
{
	SMPayloadCommandForQueue cmd;
	if(xQueueReceive(localCmdQueue, &cmd, 0) != pdTRUE )
		parentSystem->setFault(FLT_FIRMWARE,710401);

	SMPayloadCommandRet32 finalret;
	localInterpreter.executeHostSideGlobalSetParamCommand(cmd);
	localInterpreter.executeHostSideGlobalGetParamCommand(finalret,ret);

	if( discardResult==false )//GC comm task wants to discard result so dont put in queue
	{
		if(xQueueSend(retQueue, &finalret, 0) == pdTRUE )
	//	if(xQueueSend(retQueue, &ret, 0) == pdTRUE )
			return true;
		else
			return false;
	}
	else
		return true;
}

void SMCommandQueue::lockMutex()
{
	xSemaphoreTake( mutex,portMAX_DELAY );
}

void SMCommandQueue::unlockMutex()
{
	xSemaphoreGive(mutex);
}






