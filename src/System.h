/*
 * System.h
 *
 *  Created on: Apr 28, 2012
 *      Author: tero
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Device.h"

#include "Serial.h"
#include "SMCommandQueue.h"
#include "EncoderIn.h"
#include "sm485.h"
#include "DigitalCounterInput.h"
#include "ResolverIn.h"

#define VSDR_049_HW_ID 3999 /* must change this BL on prototypes if in use! now its 4000*/
#define VSDR_HW_ID 4000

/*changelog
 * V    changes
 * 1000 initial release
 * 1001 -analog in setpoint now monitor anain2 and inverts polarity if anain2>3V
 *      -fix hall sensor feed to GC (bit shift 12->13)
 * 1002 -brake delay changes etc
 * 1003 -was sent to matjaz slovenia, with BRAKE_RELEASE_DELAY_MOD 1
 * 1004 -no longer avoid resetting setpoint in quad & step dir modes when setpoint source SMP is set
 * 1005 -resolver development, disabled atm. production testing analog tolerances converted to the new test PCB
 *      -velocity poll time jitter compensation implemented
 * 1006 -production testing fast response test of GPIO removed
 * 1007 -brake released delay changed from 0 to 0.8s
 * 1008 -brake release delay is now parameterizable
 *      -DelayedConditional cleanup & new features
 * 1009 -ADC (anain and encoder adc readouts) offset software compensated
 *      -Gain of encoder ADC input corrected to mach HW better
 *      -production test voltage limits updated accordingly
 *      -sending clear faults cmd now clears also fault location 2 register
 *      -initial resolver support ready
 * 1010 -fix pulse&dir problem with noisy direction signal causing errorenous direction states
 * 1011(WIP, merged development & master branch):
 *      -implement second PWM setpoint input, uses analog input pins, see digitalcounterinput.cpp for usage
 * 		-change dscpowertask to use TIM8 instead of TIM1 (TODO: verify by measuring)
 *      -makefile now calls mkfirmware utily to generate .gdf file
 * 1090 -ported IONI features: new setpoint calculation (Ioni style simplified setpoint handling: drive setpoint is a sum of phyiscal setpoint (step/dir, pwm, analog etc) and the setpoint from SM host (instant and buffered commands). If SM host sets absolute setpoint, then phyiscal counter (incremental types onle) are reset to zero.)
 * 		-support PWM+dir and Analog+dir setpoints with on/off option in Granity
 * 		-Increase ADC sampling time (in hope that it reduces ADC channel crosstalk)
 * 		-requires GraniteCore FW version 1090 or later
 * 		-reduce fault sensitivity of resolver input
 * 		-added simulated encoder output feature
 * 1095 -changed communication protocol between GraniteCore chip and I/O side chip (GC-IO protocol):
 *       1) Now instead of sending/receiving two simultaneous streams of SimpleMotion subpackets (SMPs),
 *       we send only one and hard coded 32 bit setpoint value. Originally stream1 was used
 *       to send setpoints. Also return data stream of sream1 is removed
 *       2) Added clearController bit to GC-IO protocol. When GC returns value 1 on it, IO side resets setpoint to zero.
 *       Note: this breaks compatibility with older GC side firmwares, to use new this version, GC FW version 1095+ or racing simulator version 9195+ is required
 * 1096  -add motor commutation sensor (Hall simulation) support for resolver.
 */

/*
 * TODO & bugs
 * -serial comm fails sometimes after FW upgrade and app launch from granity. perhaps address goes wrong or it gets disturbed by serial comm rx too early?
 *
 */
#define FW_VERSION 1096
#define FW_BACKWARDS_COMPATITBLE_VERSION 1090

#define COMMAND_QUEUE1_SIZE 256
#define COMMAND_QUEUE2_SIZE 256
#define SYS_COMMAND_QUEUE_SIZE 64


//common faults for GC and STM
#define FLT_FOLLOWERROR	BV(1)
#define FLT_OVERCURRENT BV(2)
#define FLT_COMMUNICATION BV(3)
#define FLT_ENCODER	BV(4)
#define FLT_OVERTEMP BV(5)
#define FLT_UNDERVOLTAGE BV(6)
#define FLT_OVERVOLTAGE BV(7)
#define FLT_PROGRAM_OR_MEM BV(8)
#define FLT_HARDWARE BV(9)
#define FLT_OVERVELOCITY BV(10)
#define FLT_INIT BV(11)
#define FLT_MOTION BV(12)
#define FLT_RANGE BV(13)
#define FLT_PSTAGE_FORCED_OFF BV(14)
#define FLT_HOST_COMM_ERROR BV(15)

//stm32 side macros
#define FLT_GC_COMM BV(15)
#define FLT_QUEUE_FULL FLT_PROGRAM_OR_MEM
#define FLT_SM485_ERROR FLT_COMMUNICATION
#define FLT_FIRMWARE FLT_PROGRAM_OR_MEM //non-recoverable program error
#define FLT_ALLOC FLT_PROGRAM_OR_MEM //memory etc allocation failed

//list of faults that cause emergencency actions (brake etc)
#define FATAL_STM32_FW_FAULTS (FLT_GC_COMM|FLT_FIRMWARE|FLT_ALLOC|FLT_QUEUE_FULL)

//assign command streams to GC to different tasks
#define SM_BUFFERED_CMD_QUEUE GCCmdStream2_HighPriority
#define SM_INSTANT_CMD_QUEUE GCCmdStream2_LowPriority
#define SYS_CMD_QUEUE GCCmdStream2_LowPriority

#define STAT_TARGET_REACHED BV(1)
#define STAT_FERROR_RECOVERY BV(2)
//run is true only if motor is being actually driven. run=0 clears integrators etc
#define STAT_RUN BV(3)
#define STAT_ENABLED BV(4)
#define STAT_FAULTSTOP BV(5)
//follow error warning, recovering or disabled
#define STAT_FERROR_WARNING BV(6)
//get bit using CFG_STAT_USER_BIT_SOURCE
#define STAT_USER_BIT BV(7)
//ready for user command: initialized, running (no fault), not recovering, not homing & no homing aborted, not running sequence
#define STAT_SERVO_READY BV(8)

//writing flash or init for example
//#define STAT_BUSY BV(6)
//for example limit switch status or analog level triggered
//#define STAT_EXT_INPUT BV(7)
//writing 1 to this initiates config write
//////////////////#define STAT_WRITEFLASH BV(8)
//phasesearch complete
////////////////#define STAT_ROTORALIGNED BV(9)
//power stage disable. this must be cleared only by enablePowerStage
//#define STAT_POWERSTAGE_DISABLED BV(10)
//external disable interrupt occurred, cleared when PID loop sees ext disable release

//running sequence
#define STAT_RUN_SEQUENCE BV(9)

//for information only
//#define STAT_EXT_DISABLE_EVENT BV(10)
#define STAT_BRAKING BV(10)
//writing 1 to this initiates homing
#define STAT_HOMING BV(11)
#define STAT_INITIALIZED BV(12)
#define STAT_VOLTAGES_OK BV(13)
//this is 1 when opto out should indicate error, same as STAT_FAULTSTOP
//#define STAT_FAULT_INDICATOR_OUT BV(14)
//outputs disabled until reset
#define STAT_PERMANENT_STOP BV(15)





/*
 * STM32 perpheral usage:
 *
 * TIM1 PWM generator for DSCPowerTask
 * TIM2 DigitalCounterInput (step/dir)
 * TIM3 EncoderIn
 * TIM4 System high freq task
 * TIM5 32 bit system time counter
 *
 * DMA2_Stream0 Channel 0 AnalogIn
 * DMA2_Stream2 USART RS485
 * DMA1_Stream5 USART DSC
 *
 * ADC1 AnalogIn
 * ADC2 AnalogIn
 *
 * USART1 Serial RS485
 * USART2 Serial DSC
 *
 * GPIOs (uncomplete):
 * PA9 Serial TX RS485
 * PA10 Serial RX RS485
 * PC10 Serial TXEN RS485
 * PA2 Serial TX DSC
 * PA3 Serial RX DSC
 * Most used by DigitalInputPin & OutputPins
 *
 *
 *
 */
/*
 * STM32 interrupt priority usage:
 *
 * 0=highest 15=lowest, no subpriorities are used
 *
 * 0=DigitalCounterInput (dir swtich)
 * 1=ResolverIn
 *  uncomplete...
 *
 */

class System
{
	friend class SMCommandInterpreter;

private:
	const BLHeader *BootloaderInfo; //placed here so gets initialized first, needed by some other initializations
public:
	System();
	virtual ~System();

	/* faultlocation coding: 6 numbers: ffFFll
	 * ff=unique per file
	 * FF=unique number per function
	 * ll=unique inside function
	 *
	 * used ff numbers:
	 * 10 main
	 * 20 stm32f2xx_it
	 * 71 SMCommandQueue
	 * 48 sm485
	 * 60 mccommunication
	 * 15 System
	 * 89 ResolverIn
	 */
	void setFault( u32 faultbits, u32 faultlocation );
	void clearFaults(); //clear all faults and fault location
	void setStatus( u32 statusbits );//set status bits active


	//calling this either enables or disables STM32 sread spectrum clock depending on feedback & reference signal modes selected
	void updateSpreadSpectrumClockMode();

	//physical IO of system
	PhysicalIO physIO;

	//simple motion instance
	SimpleMotionComm SMComm;

	//rs485 port for simple motion
	Serial serialPortRS485;

	//GC communication serial link
	Serial serialPortGC;


	/* command streams between STM and GC CPU's. Data exchange
	 * handled at mccommunication task. there are two streams of commands
	 * and stream 1 is dedicated for reference signal while stream 2 is
	 * shared between 3 "users" with different priorities. highes priority
	 * queue will be served first. stream 1 and 2 operate simultaneously. */


	//for instant commands from SM485 bus
	SMCommandQueue GCCmdStream2_LowPriority;

	//for buffered commands from SM485 bus
	SMCommandQueue GCCmdStream2_HighPriority;

	//for internal commands, get/set params & control etc
	SMCommandQueue GCCmdStream2_MediumPriority;

	//read parameter GC side, currently quite slow implementation so use only for init
	s32 getParameter( u16 paramID, bool &fail );
	//write parameter GC side, currently quite slow implementation so use only for init. return true on success
	bool setParameter( u16 paramID, s32 value );//sends to medium priority queue
	bool setParameter( u16 paramID, s32 value, SMCommandQueue &queue);

	/*Feedback drivers*/
	EncoderIn encoder;
	ResolverIn resolver;
	enum FeedbackDevice { None,Encoder,Resolver };


	/* Drive input reference singal methods */
	DigitalCounterInput digitalCounterInput;
	enum InputReferenceMode { Serialonly=0,Pulsetrain=1,Quadrature=2, PWM=3, Analog=4, Reserved1=5, Reserved2=6 };
	enum ControlMode { Position=1,Velocity=2,Torque=3 };

	enum Signal { DeviceReset=0, RunProductionTest, ProductionTestOver };
	bool isSignal(Signal s)
	{
		return ( (SignalReg>>int(s))&0x1 );
	}
	void setSignal(Signal s)
	{
		SignalReg|=1<<int(s);
	}
	void clrSignal(Signal s)
	{
		SignalReg&=~(1<<int(s));
	}

	void setInputReferenceMode(InputReferenceMode mode);
	InputReferenceMode getInputReferenceMode();
	//get reference value from currently selected input
	s32 getInputReferenceValue();
	u32 getFaultBitsReg() const;
	u32 getFirstFaultLocation() const;
	u32 getStatusBitsReg() const;
	u32 getFirstFaultBitsReg() const;

	//these should be called only by MCcomm task once per 400us cycle, use getLastPositionFeedbackValue() from elsewhere
	s16 getVelocityFeedbackValue();
	u16 getPositionFeedbackValue();
	//get Hall sensor bits, or equivalent from a feedback device (simulated Halls on resolver)
	u8 getCommutationSensorState();

	u16 getLastPositionFeedbackValue(){ return lastPositionFBValue; }
	//for emulated encoder output. outputs true one time after feedback device index has been passed
	//flaw: this method is inaccurate if speed is above 2500 counts/s
	bool simulatedIndexPulseOutNow()
	{
		bool ret=indexHasOccurred;
		indexHasOccurred=false;
		return ret;
	}
	

	//this method has very high priority and is called from isr at 40kHz
	void highFrequencyISRTask();

	//get current time, a rolling hardware counter value. 1 tick=microsecond
	u32 getTimeMicrosecs();
	//time counter in herz
	static const int timeCounterFrequency=1000000;


	//GC status & faults fetched here at low rate for led blink task
	u16 GCStatusBits, GCFaultBits, GCFirstFault;

	//set or release mech brake according to motor status
	void updateMechBrakeState();

	//update GPI/GPO pins
	void updatePhysGPIOs_obsolete();

	/*
	 * all commands going to GC side will pass thru this function which has option to modify the command
	 * and its parameters. used for overriding/modifying some values such as control bits param to
	 * contain bits from phyiscal inputs.
	 */
	//bool filterSetParamCommandToGC( SMPayloadCommandForQueue &cmd );
	//bool deviceResetRequested;//set at SM interpreter and executed with delay to allow reply packet to SM bus

	//return currently underlying hardware ID: http://granitedevices.com/wiki/Hardware_version_ID
	u32 getHardwareID();

	//channel = 4-6 which equals SMP_DEBUGPARAM_4..6
	void setDebugParam(s32 channel,s32 value)
	{
		if(channel<4 || channel>6) return;
		debugParams[channel-4]=value;
	}
	//channel = 4-6 which equals SMP_DEBUGPARAM_4..6
	s32 getDebugParam(s32 channel)
	{
		if(channel<4 || channel>6) return 0;
		return debugParams[channel-4];
	}

	ControlMode getControlMode() const
	{
		return presentControlMode;
	}


	bool readInitStateFromGC();

	void setBrakeEngageDelayMs( int brakeEngageDelayMs )
		{
		this->brakeEngageDelayMs = brakeEngageDelayMs;
	}

	void setBrakePoweronReleaseDelayMs( int brakePoweronReleaseDelayMs )
	{
		this->brakePoweronReleaseDelayMs = brakePoweronReleaseDelayMs;
	}

	int getBrakeEngageDelayMs() const
	{
		return brakeEngageDelayMs;
	}

	int getBrakePoweronReleaseDelayMs() const
	{
		return brakePoweronReleaseDelayMs;
	}
	

	FeedbackDevice getCurrentPositionFeedbackDevice()
	{
		return positionFeedbackDevice;
	}

	bool indexHasOccurred;

	//return true if flag bit is on (drive config param flags). See FLAG_DISABLED_AT_STARTUP etc defines
	bool isFlagBit(u32 bit)
	{
		return DriveFlagBits&bit;
	}

	void setSerialSetpoint(s32 setp)
	{
		serialSetpoint=setp;
		resetPhyiscalSetpoint();
	}
	void incrementSerialSetpoint(s32 setp)
	{
		serialSetpoint+=setp;
	}

	void resetPhyiscalSetpoint()
	{
		digitalCounterInput.setCounter(0);
	}

private:
	//these registers are for local STM side status&faults. for GC side registers, see GCStatusBits etc
	u32 FaultBitsReg;
	u32 FirstFaultBitsReg;
	u32 FirstFaultLocation;
	u32 LastFaultLocation;
	u32 StatusBitsReg;

	//write certain bits in this register to asynchronously start functions elsewhere in the program.
	//code elsewhere polls bits in this register and act if 1.
	//see enum Signal
	u32 SignalReg;
	u32 DriveFlagBits;
	s32 setpointOffset;

	s32 debugParams[3];

	//last setpoint value from SM host
	s32 serialSetpoint;

	ControlMode presentControlMode;
	FeedbackDevice positionFeedbackDevice,velocityFeedbackDevice;

	//called from SMCommandInterpreter when SMV2 command tells to change control mode
	void setControlMode(ControlMode mode)
	{
		presentControlMode=mode;
	}

	//enalbe or disable STM32 spread spectrum clocking. disabling may be useful for PWM reference input accuracy.
	void enableSpreadSpectrumClock( bool enable );

	//start running highFrequencyISRTask
	void enableHighFrequencyTask( bool on );

	//init timer that may be used for timeout counting etc, reading raw timer value etc
	void initGeneralTimer();

	InputReferenceMode inputReferenceMode;//choose between drive input references. value must be one of SMP_INPUT_REFERENCE_MODE_ constants

    xSemaphoreHandle SysCmdMutex;//avoid nesting GC set/getparam calls in different threads

    int brakePoweronReleaseDelayMs;
    int brakeEngageDelayMs;
	
	u16 lastPositionFBValue;
};

#endif /* SYSTEM_H_ */


