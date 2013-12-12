/*
 * SMCommandInterpreter.h
 *
 *  Created on: 25.2.2012
 *      Author: Tero
 */

#ifndef SMCOMMANDINTERPRETER_H_
#define SMCOMMANDINTERPRETER_H_
#include "types.h"

class System;

class SMCommandInterpreter
{
public:
	//SMCommandInterpreter( SimpleMotionComm *parent );
	SMCommandInterpreter( System *parent );
	~SMCommandInterpreter();

	//returns true if this command should be passed to GC side, false if param handled completely here
	bool executeCommand( SMPayloadCommandForQueue cmd, SMPayloadCommandRet32 &ret );

	/*
	 * Execute a command that has global effect on host side MCU (STM32) of VSD-R.
	 * Return true if command shall be passed forward to GC side or false if command should be filtered out.
	 */
	/*new spec: all commands & return values must pass to GC side! use resered address space to implement own parameters in stm32 side
	 *
	 */
	bool executeHostSideGlobalSetParamCommand( SMPayloadCommandForQueue cmd );
	bool executeHostSideGlobalGetParamCommand(  SMPayloadCommandRet32 & ret, SMPayloadCommandRet32 & retFromGC );
	//bool executeHostSideGlobalGetParamCommand(  SMPayloadCommandRet32 & ret );
    u16 getSetParamAddr() const;

private:
	//interpreter specific params
	u16 setParamAddr, returnParamAddr, returnParamLength;
	u32 lastReturnStatus;//ACK, NACK etc
	u32 cumulativeStatus;
	//SimpleMotionComm *parentComm;
	System *parentSystem;
};




#endif /* SMCOMMANDINTERPRETER_H_ */
