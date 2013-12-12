/*
 * SMCommandInterpreter.cpp
 *
 *  Created on: 25.2.2012
 *      Author: Tero
 */

#include "SMCommandInterpreter.h"
#include "simplemotion_defs.h"
#include "sm485.h"
#include "globals.h"
#include "System.h"


SMCommandInterpreter::SMCommandInterpreter( System *parent)
{
	setParamAddr = returnParamAddr = 0;
	cumulativeStatus = SMP_CMD_STATUS_ACK;
	returnParamLength = SMPRET_CMD_STATUS;
	lastReturnStatus = SMP_CMD_STATUS_ACK;
	parentSystem=parent;
}

SMCommandInterpreter::~SMCommandInterpreter()
{
	// TODO Auto-generated destructor stub
}

bool SMCommandInterpreter::executeCommand( SMPayloadCommandForQueue cmd,
		SMPayloadCommandRet32 & ret )
{
	bool passCommandToGC = true;

	/*
	 * This interpreter does not check whether address or param value is valid.
	 * If command is passed to GC then it checks the correctness and returns error if necessary.
	 * Problem: this interpreter may end up having incorrect address that shoud not be
	 * accepted, however it should not cause errors.
	 */

	if (cmd.ID == SMPCMD_SET_PARAM_ADDR)
	{
		setParamAddr = cmd.param;
	}
	else //set param commands
	{
		if (setParamAddr == SMP_RETURN_PARAM_ADDR)
			returnParamAddr = cmd.param;
		else if (setParamAddr == SMP_RETURN_PARAM_LEN)
			returnParamLength = cmd.param;
		else
		{
			passCommandToGC = executeHostSideGlobalSetParamCommand( cmd );
		}
	}
	return passCommandToGC;
	//FIXME GClta tarvii anyway kysyy paluudata silta varalta etta paluudata ei tule taalta
	//ratkasu? kaikki komennot menee my�s gclle tarvittiin tai ei niit� siell�
}


/*
 * Execute a command that has global effect on I/O side MCU (STM32) of drive
 * Return true if command shall be passed forward to GC side or false if command should be filtered out.
 */
bool SMCommandInterpreter::executeHostSideGlobalSetParamCommand(
		SMPayloadCommandForQueue cmd )
{
	bool passCommandToGC = true; //let GC to see the same command, change to false if adding custom params
	//that GC is not aware of to avoid error in GC interpreter.

	/*
	 * List of that params should be handled in this function:
	 *
	 * 1 node address (maybe read only)
	 * 2 SM bus mode, 16b
	 * 3 node's SM version, 16b
	 * 4 node's SM version backwards compatible with, 16b
	 * 5 SM bus speed, 16b
	 * 6 SM node buffer free amount (bytes), 16b
	 * 7 SM buffered cmd active now, 16b
	 * 8 SM buffered CMD cycle period (100us/tick), 16b
	 * 12 timeout
	 *
	 * 128-131 digital input actual values, 16 channels each, 16b
	 * 132-135 digital input value changed since last read (register resets to 0 on read). 16 channels each, 16b
	 * 136-139 digital set output values, 16 channels each, 16b
	 * 140-148 digital I/O direction register, 0=input, 1=output, 16 chans each, 16b (may not be supported by HW)
	 *
	 * 168-183 analog input values, signed 16 bits, inputs 1-16, 16b
	 * 184-200 analog output values, signed 16 bits, outputs 1-16, 16b
	 * */
	lastReturnStatus = SMP_CMD_STATUS_ACK;

/*	if (setParamAddr >= 200 && cmd.ID != SMPCMD_SET_PARAM_ADDR)
		return true; //if not handled here
*/
	if (cmd.ID == SMPCMD_SET_PARAM_ADDR)
		setParamAddr = cmd.param;
	else
		switch (setParamAddr)
		{
		/*SM bus common params */
		case SMP_RETURN_PARAM_ADDR:
			returnParamAddr = cmd.param;
			break;
		case SMP_RETURN_PARAM_LEN:
			returnParamLength = cmd.param;
			break;
		case SMP_BUS_MODE:
			if (parentSystem->SMComm.setBusMode( cmd.param ) == false)
				lastReturnStatus = SMP_CMD_STATUS_INVALID_VALUE;
			break;
		case SMP_BUS_SPEED:
			if (parentSystem->SMComm.setBusBaudRate( cmd.param ) == false)
				lastReturnStatus = SMP_CMD_STATUS_INVALID_VALUE;
			break;
		case SMP_BUFFERED_CMD_PERIOD:
			if (parentSystem->SMComm.setBusBufferedCmdPeriod( cmd.param )
					== false)
				lastReturnStatus = SMP_CMD_STATUS_INVALID_VALUE;
			break;
		case SMP_TIMEOUT:
			if (parentSystem->SMComm.setBusTimeout( cmd.param ) == false)
				lastReturnStatus = SMP_CMD_STATUS_INVALID_VALUE;
			break;
		case SMP_CUMULATIVE_STATUS:
			cumulativeStatus=cmd.param;
			break;

		/* Device specific params */
		case SMP_DIGITAL_OUT_VALUE_1:
			//TODO
			break;
		case SMP_INPUT_REFERENCE_MODE:
			parentSystem->setInputReferenceMode((System::InputReferenceMode)cmd.param);
			break;
		case SMP_CONTROL_MODE:
			parentSystem->setControlMode( (System::ControlMode)cmd.param);
			break;

		case SMP_FAULTS:
			parentSystem->clearFaults();
			break;
		case SMP_DEBUGPARAM4:
			parentSystem->setDebugParam(4,cmd.param);
			break;
		case SMP_DEBUGPARAM5:
			parentSystem->setDebugParam(5,cmd.param);
			break;
		case SMP_DEBUGPARAM6:
			parentSystem->setDebugParam(6,cmd.param);
			break;
		case SMP_SYSTEM_CONTROL:
			if(cmd.param==SMP_SYSTEM_CONTROL_RESTART)
				//parentSystem->deviceResetRequested=true;
				parentSystem->setSignal(System::DeviceReset);
			else if(cmd.param==SMP_SYSTEM_CONTROL_ABORTBUFFERED)
				parentSystem->SMComm.abortBufferedCmdExecution();
			else if(cmd.param==SMP_SYSTEM_START_PRODUCTION_TEST )
			{
				parentSystem->clrSignal(System::ProductionTestOver);
				parentSystem->setSignal(System::RunProductionTest);
			}
			else if(cmd.param==SMP_SYSTEM_STOP_PRODUCTION_TEST )
				parentSystem->clrSignal(System::RunProductionTest);
			break;
		case SMP_BRAKE_POWERON_RELEASE_DELAY:
			parentSystem->setBrakePoweronReleaseDelayMs(cmd.param*1000);
			break;
		case SMP_BRAKE_STOP_ENGAGE_DELAY:
			parentSystem->setBrakeEngageDelayMs(cmd.param*1000);
			break;
		default:
			if (setParamAddr < 200 )
				//attempt to modify read only or unsupported param causes error status
				lastReturnStatus = SMP_CMD_STATUS_NACK;

			break;
		}

	cumulativeStatus|=lastReturnStatus;
	return passCommandToGC;
}


bool SMCommandInterpreter::executeHostSideGlobalGetParamCommand(
		SMPayloadCommandRet32 & ret,SMPayloadCommandRet32 & retFromGC  )
{
	u16 returnParamMaskedAddr=returnParamAddr&SMP_ADDRESS_BITS_MASK;
	u16 returnParamAttribute=returnParamAddr&SMP_ATTRIBUTE_BITS_MASK;
	bool overrideGCreturnPacket = false;
	s32 retValue = 0;

	//by default, let return data come from GC. overridden later if necessary
	ret.ID=retFromGC.ID;
	ret.retData=retFromGC.retData;

//	if (returnParamMaskedAddr >= 200)
		//return false; //not handled here

	switch (returnParamMaskedAddr)
	{
		//these are the only params addresses below 200 that are also handled at GC
		case SMP_RETURN_PARAM_ADDR:
		case SMP_RETURN_PARAM_LEN:
			break;
		case SMP_BUS_MODE:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getBusMode(returnParamAttribute);
			break;
		case SMP_BUS_SPEED:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getBusBaudRate(returnParamAttribute);
			break;
		case SMP_BUFFERED_CMD_PERIOD:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getBusBufferedCmdPeriod(returnParamAttribute);
			break;
		case SMP_TIMEOUT:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getBusTimeout(returnParamAttribute);
			break;
		case SMP_SM_VERSION:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getSMBusVersion(returnParamAttribute);
			break;
		case SMP_SM_VERSION_COMPAT:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getSMBusCompatVersion(returnParamAttribute);
			break;
		case SMP_BUFFER_FREE_BYTES:
			overrideGCreturnPacket = true;
			retValue = parentSystem->SMComm.getSMBusBufferFreeBytes(returnParamAttribute);
			break;

		case SMP_DIGITAL_IN_VALUES_1:
			overrideGCreturnPacket = true;
			retValue = parentSystem->physIO.getDigitalInputs();
			break;
		case SMP_ANALOG_IN_VALUE_1:
			overrideGCreturnPacket = true;
			retValue = parentSystem->physIO.getAnalogInput1();
			break;
		case SMP_ANALOG_IN_VALUE_2:
			overrideGCreturnPacket = true;
			retValue = parentSystem->physIO.getAnalogInput2();
			break;
		case SMP_ANALOG_IN_VALUE_3:
			overrideGCreturnPacket = true;
			retValue = parentSystem->physIO.getAnalogInputEncA();
			break;
		case SMP_ANALOG_IN_VALUE_4:
			overrideGCreturnPacket = true;
			retValue = parentSystem->physIO.getAnalogInputEncB();
			break;

		case SMP_DEBUGPARAM4:
			overrideGCreturnPacket=true;
			retValue=parentSystem->getDebugParam(4);
			break;
		case SMP_DEBUGPARAM5:
			overrideGCreturnPacket=true;
			retValue=parentSystem->getDebugParam(5);
			break;
		case SMP_DEBUGPARAM6:
			overrideGCreturnPacket=true;
			retValue=parentSystem->getDebugParam(6);
			break;

		case SMP_DIGITAL_OUT_VALUE_1:
			overrideGCreturnPacket = true;
			//TODO
			break;
		case SMP_FAULTS:
			overrideGCreturnPacket = true;
			retValue=retFromGC.retData|parentSystem->getFaultBitsReg(); //special case, OR the bits with GD and STM
			break;
		case SMP_FAULT_LOCATION2:
			overrideGCreturnPacket = true;
			retValue=parentSystem->getFirstFaultLocation();
			break;
		case SMP_CUMULATIVE_STATUS:
			overrideGCreturnPacket = true;
			retValue=retFromGC.retData|cumulativeStatus; //special case, OR the bits with GD and STM
			//retValue=cumulativeStatus; //special case, OR the bits with GD and STM
			break;
		case SMP_FIRMWARE_VERSION:
			overrideGCreturnPacket=true;
			retValue=FW_VERSION;
			break;
		case SMP_FIRMWARE_BACKWARDS_COMP_VERSION:
			overrideGCreturnPacket=true;
			retValue=FW_BACKWARDS_COMPATITBLE_VERSION;
			break;

		default:
			if (returnParamMaskedAddr < 200)
				return false; //not handled here
			break;
	}

	cumulativeStatus|=lastReturnStatus;

	if(overrideGCreturnPacket)
	{
		if(returnParamMaskedAddr==SMP_DIGITAL_IN_VALUES_1)
			asm("nop");

		//write to output
		ret.ID = returnParamLength;
		if (returnParamLength == SMPRET_CMD_STATUS)
			ret.retData = lastReturnStatus;
		else
			ret.retData = retValue;
	}

	return overrideGCreturnPacket;
}

u16 SMCommandInterpreter::getSetParamAddr() const
{
	return setParamAddr;
}

