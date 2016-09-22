/*
 * System.cpp
 *
 *  Created on: Apr 28, 2012
 *      Author: tero
 */

#include "System.h"
#include "utils.h"

#include "sm485.h"
#include "LedBlinkTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "globals.h"

//timer ISR
#include "stm32f2xx_tim.h"
#include "stm32f2xx_rcc.h"
#include "misc.h"//==NVIC
#include "utils.h"

#define FAULTLOCATION_BASE 100000

System::System() :
		//SMComm( &serialPortRS485, this, 5 ),
		BootloaderInfo((BLHeader*)BLHEADER_ADDRESS),
		physIO(this),
		SMComm( &serialPortRS485, this, physIO.getDIPSwitchAddress() ),
		serialPortRS485( Serial::RS485,
				this ), serialPortGC( Serial::DSC, this ), GCCmdStream2_LowPriority(
				this, COMMAND_QUEUE2_SIZE, "cmdQ2Med" ), GCCmdStream2_HighPriority(
				this, COMMAND_QUEUE2_SIZE, "cmdQS2High" ), GCCmdStream2_MediumPriority(
				this, SYS_COMMAND_QUEUE_SIZE, "cmdQ2Low" ),
				resolver(this)

{
	lastPositionFBValue=0;
	FaultBitsReg = 0;
	FirstFaultBitsReg = 0;
	FirstFaultLocation = 0;
	LastFaultLocation = 0;
	StatusBitsReg = 0;
	GCStatusBits=STAT_BRAKING;//set braking on initially to prevent brake clitch at boot
	GCFaultBits=0;
	GCFirstFault=0;
	SignalReg=0;
	DriveFlagBits=0;
	setpointOffset=0;
	serialSetpoint=0;
	//deviceResetRequested=false;

	//encoder is the default setting, changed later if configured so
	positionFeedbackDevice=None;
	velocityFeedbackDevice=None;


	//digitalCounterInput.setCountMode( DigitalCounterInput::PWM );

	setDebugParam(4,-1);
	setDebugParam(5,-1);
	setDebugParam(6,-1);

	updateSpreadSpectrumClockMode();
	initGeneralTimer();
	enableHighFrequencyTask( true );



	//s32 testPulseAmpl=0;
	//s32 testPulsePause=0;
	vSemaphoreCreateBinary( SysCmdMutex );
    if(SysCmdMutex==NULL)
    {
    	setFault( FLT_FIRMWARE|FLT_ALLOC,FAULTLOCATION_BASE+01);
    }

}

System::~System()
{
}

u32 System::getHardwareID()
{
	return BootloaderInfo->HardwareID;
}

void System::enableSpreadSpectrumClock( bool enable )
{
	if (enable)
	{
		//spectrum spread 1kHz 1% depth @ PLL_M=8 CPU=120MHz, center freq mode
		//RCC->SSCGR= 250 | (63<<13) | RCC_SSCGR_SSCGEN;

		//spectrum spread 5kHz 1% depth @ PLL_M=8 CPU=120MHz, center freq mode
		RCC->SSCGR = 50 | (315 << 13) | RCC_SSCGR_SSCGEN;
	}
	else
	{
		RCC->SSCGR = 0;
	}
}

s32 System::getParameter( u16 paramID, bool &fail )
{
	SMPayloadCommandForQueue tx;

	xSemaphoreTake( SysCmdMutex, portMAX_DELAY );

	//set return data len to 32b
	tx.ID = SMPCMD_SET_PARAM_ADDR;
	tx.discardRetData = true;
	tx.param = SMP_RETURN_PARAM_LEN;
	if (GCCmdStream2_MediumPriority.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150101 );
		xSemaphoreGive( SysCmdMutex );
		return -1;
	}
	tx.ID = SMPCMD_24B;
	tx.param = SMPRET_32B;
	if (GCCmdStream2_MediumPriority.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150102 );
		xSemaphoreGive( SysCmdMutex );
		return -1;
	}

	//set param address to read
	tx.ID = SMPCMD_SET_PARAM_ADDR;
	tx.discardRetData = true;
	tx.param = SMP_RETURN_PARAM_ADDR;
	if (GCCmdStream2_MediumPriority.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150103 );
		xSemaphoreGive( SysCmdMutex );
		return -1;
	}
	tx.ID = SMPCMD_24B;
	tx.param = paramID;
	tx.discardRetData = false;
	if (GCCmdStream2_MediumPriority.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150104 );
		xSemaphoreGive( SysCmdMutex );
		return -1;
	}

	s32 ret = GCCmdStream2_MediumPriority.getReturnValue( fail );
	if (fail)
		setFault( FLT_GC_COMM, 150105 );

	xSemaphoreGive( SysCmdMutex );
	return ret;
}

bool System::setParameter( u16 paramID, s32 value )
{
	bool ret;
	xSemaphoreTake( SysCmdMutex, portMAX_DELAY );
	ret=setParameter( paramID, value, GCCmdStream2_MediumPriority );
    xSemaphoreGive( SysCmdMutex );
    return ret;
}

bool System::setParameter( u16 paramID, s32 value, SMCommandQueue &queue )
{
	bool fail;
	SMPayloadCommandForQueue tx;

	queue.lockMutex();

	//set param address to write
	tx.ID = SMPCMD_SET_PARAM_ADDR;
	tx.discardRetData = true;
	tx.param = paramID;
	if (queue.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150201 );
		queue.unlockMutex();
		return false;
	}
	tx.ID = SMPCMD_32B;
	tx.param = value;
	tx.discardRetData = true;
	if (queue.sendCommand( tx ) == false)
	{
		fail = true;
		setFault( FLT_GC_COMM, 150202 );
		queue.unlockMutex();
		return false;
	}

	queue.unlockMutex();
	return true;
}

void System::setFault( u32 faultbits, u32 faultlocation)
{
	//encoder fault must be sent to GC side too to have desired effect (fault stop)
	//do this only if encoder fault is not yet set to avoid flooding queue
	if(((FaultBitsReg&FLT_ENCODER)==0) && (faultbits&FLT_ENCODER))
	{
		setParameter(SMP_FAULTS, FLT_ENCODER, SYS_CMD_QUEUE);
	}


	if (FaultBitsReg == 0)
	{
		FirstFaultBitsReg = faultbits;
		FirstFaultLocation = faultlocation;
	}
	FaultBitsReg |= faultbits;
	LastFaultLocation = faultlocation;


	//infinite loop for debugging. TODO FIXME remove infinite loop from official code
	if (faultbits & (FLT_FIRMWARE | FLT_ALLOC))
	{
		/*TODO decide what to do here. FreeRTOS may still run even if HW faults happened
		 * so would it be best to attempt send some disable command to GC side..
		 * or just suspend FreeRTOS so GC timeouts and disables.
		 * but it may be useful to have systems operational...
		 */
		vTaskSuspend(&ledBlinkTaskHandle);//suspend led task
		LedBlinkTask(this);//any non-null argument value will do to select mode of led blink task

		while (1)
			NOP;
	}
}

void System::clearFaults()
{
	FirstFaultBitsReg=FaultBitsReg=0;
	FirstFaultLocation=LastFaultLocation=0;
}

void System::setInputReferenceMode( InputReferenceMode mode )
{
	inputReferenceMode = mode;
	switch (mode)
	{
	case Serialonly:
		digitalCounterInput.setCountMode( DigitalCounterInput::None );
		break;
	case Pulsetrain:
		digitalCounterInput.setCountMode( DigitalCounterInput::PulseTrain );
		break;
	case Quadrature:
		digitalCounterInput.setCountMode( DigitalCounterInput::Quadrature );
		break;
	case PWM:
		digitalCounterInput.setCountMode( DigitalCounterInput::PWM );
		break;
	case Analog:
		digitalCounterInput.setCountMode( DigitalCounterInput::None );
		break;
	case Reserved1:
		digitalCounterInput.setCountMode( DigitalCounterInput::None );
		break;
	case Reserved2:
		digitalCounterInput.setCountMode( DigitalCounterInput::None );
		break;
	default:
		digitalCounterInput.setCountMode( DigitalCounterInput::None );
		break;
	}

	updateSpreadSpectrumClockMode();
}

u32 System::getFaultBitsReg() const
{
	return FaultBitsReg;
}

u32 System::getFirstFaultLocation() const
{
	return FirstFaultLocation;
}

u32 System::getFirstFaultBitsReg() const
{
	return FirstFaultBitsReg;
}

u32 System::getStatusBitsReg() const
{
	return StatusBitsReg;
}

s32 System::getInputReferenceValue()
{
	s32 setpoint=0;

	switch (inputReferenceMode)
	{
	case Serialonly:
		setpoint=0;
		break;
	case Pulsetrain:
		setpoint=digitalCounterInput.getCounter();
		break;
	case Quadrature:
		setpoint=digitalCounterInput.getCounter();
		break;
	case PWM:
		if(isFlagBit(FLAG_ENABLE_DIR_INPUT_ON_ABS_SETPOINT))//PWM+DIR mode
		{
			if(physIO.dinHSIN1.inputState()) //non inverted if 0
			{
				setpoint=(digitalCounterInput.getCounter(0,false))+setpointOffset;
				if(setpoint<0) setpoint=0;//dont allow wrong polarity in dir input mode
			}
			else//inverted
			{
				setpoint=-(digitalCounterInput.getCounter(0,false)+setpointOffset);
				if(setpoint>0) setpoint=0;//dont allow wrong polarity in dir input mode
			}
		}
		else//PWM only input
		{
			setpoint=digitalCounterInput.getCounter(0,true);
			if(!digitalCounterInput.isInvalidPWMSignal())//good only for this mode, dir mode no pwm is valid too..
				setpoint+=setpointOffset;
		}
		break;
	case Analog:
		if( isFlagBit(FLAG_ENABLE_DIR_INPUT_ON_ABS_SETPOINT))//DIR input active
		{
			if(physIO.getAnalogInput2()>4915)//inverted analog1 if anain2>3.0V
			{
				setpoint = -(physIO.getAnalogInput1()+setpointOffset);//inverted
				if(setpoint>0) setpoint=0;//dont allow wrong polarity in dir input mode
			}
			//non-inverted analog if anain2 is below 3V
			else
			{
				setpoint = physIO.getAnalogInput1()+setpointOffset;//non inverted
				if(setpoint<0) setpoint=0;//dont allow wrong polarity in dir input mode
			}
		}
		else
		{
			setpoint=physIO.getAnalogInput1()+setpointOffset;
		}
		break;
	case Reserved1:
		setpoint=0;
		break;
	case Reserved2:
		setpoint=0;
		break;
	default:
		return 0;
		break;
	}

	return setpoint+serialSetpoint;
}

s16 System::getVelocityFeedbackValue()
{
	static u32 timeOfLastCall=0;
	u32 timeNow=getTimeMicrosecs();
	s32 uncompensatedVel;

	switch(velocityFeedbackDevice)
	{
	case Encoder:
		uncompensatedVel=encoder.getVelocity();
		break;
	case Resolver:
		uncompensatedVel=resolver.getVelocity();
		break;
	case None:
	default:
		uncompensatedVel=0;
		break;
	}

	//compensate velocity value to counter time jitter of caller (GC side communication jitters due to CPU load)
	//value is normalized to equal velocity with constant 400us cycle time.
	//uncompensatedVel is actually position difference between two calls

	s32 timeDiff=s32(timeNow-timeOfLastCall);//time in us since the last cycle

	//filter out huge variations (such as init moment)
	if(timeDiff<300)
		timeDiff=300;
	else if(timeDiff>500)
		timeDiff=500;

	s32 vel=uncompensatedVel*400/timeDiff;

	timeOfLastCall=timeNow;

	return s16(vel);
}

u16 System::getPositionFeedbackValue()
{
	switch(velocityFeedbackDevice)
	{
	case Encoder:
		lastPositionFBValue=encoder.getCounter();
		break;
	case Resolver:
		lastPositionFBValue=resolver.getAngle();
		break;
	case None:
	default:
		lastPositionFBValue=0;
	}
	return lastPositionFBValue;
}


//volatile int adcrunover=0;
volatile bool adcread=true;

//this method has very high priority and is called from isr at 40kHz
//execution time of function must be less than 8Âµs
void OPTIMIZE_FUNCTION System::highFrequencyISRTask()
{
	static int resolverCycle=0;
//	sys.physIO.doutDebug1.setState(1);


/*	test if samplig is still ongoing
  if(adcread==false)
	{
		NOP;
		adcrunover++;
	}
	adcread=false;*/
	//physIO.ADin.storeSamples();

//	physIO.ADin.collectNoiseStatistics(); //debug only

	//physIO.ADin.startSampling();


	if( positionFeedbackDevice==Resolver || velocityFeedbackDevice==Resolver)
	{
		//drive / sample resolver
		if(resolverCycle>=4)
				resolverCycle=0;
		if(resolverCycle==0)
		{
			resolverCycle++;
			sys.physIO.doutCHE.setState(1);
		}
		else if( resolverCycle==1)
		{
			resolverCycle++;
			physIO.ADin.storeSamples();
			physIO.ADin.startSampling();
			resolver.addSamples(physIO.ADin.getVoltageVolts(AnalogIn::EncA),physIO.ADin.getVoltageVolts(AnalogIn::EncB),true);
		}
		else if( resolverCycle==2)
		{
			sys.physIO.doutCHE.setState(0);
			resolverCycle++;
		}
		else if( resolverCycle==3)
		{
			resolverCycle=0;
			physIO.ADin.storeSamples();
			physIO.ADin.startSampling();
			resolver.addSamples(physIO.ADin.getVoltageVolts(AnalogIn::EncA),physIO.ADin.getVoltageVolts(AnalogIn::EncB),false);
		}
	}
	else if(isSinCosEncoder)
	{
		physIO.ADin.storeSamples();
		physIO.ADin.startSampling();
		encoder.update();//adjust delay/order between startsampling and update
		NOPdelay(3);//2.2us or more to finish first ADC conversion
		tee samplejen talletus tänne resolver.addSamples(physIO.ADin.getVoltageVolts(AnalogIn::EncA),physIO.ADin.getVoltageVolts(AnalogIn::EncB),false);
	}
	else
	{
		physIO.ADin.storeSamples();

	//	physIO.ADin.collectNoiseStatistics(); //debug only

		physIO.ADin.startSampling();

	}
//	sys.physIO.doutDebug1.setState(0);

}

System::InputReferenceMode System::getInputReferenceMode()
{
	return inputReferenceMode;
}


void System::updateSpreadSpectrumClockMode()
{
	//FIXME now just disable it in all cases, must take account resolver read mode too
	//also affects velocity FB calculation (but not significant?)
	//besides not sure if this is useful anyways..
	enableSpreadSpectrumClock(false);
	return;

	switch (inputReferenceMode)
	{
	case Serialonly:
		enableSpreadSpectrumClock(true);
		break;
	case Pulsetrain:
		enableSpreadSpectrumClock(true);
		break;
	case Quadrature:
		enableSpreadSpectrumClock(true);
		break;
	case PWM:
		enableSpreadSpectrumClock(false); //disable spread spectrum because it causes jitter to PWM readout
		break;
	case Analog:
		enableSpreadSpectrumClock(true);
		break;
	case Reserved1:
		enableSpreadSpectrumClock(true);
		break;
	case Reserved2:
		enableSpreadSpectrumClock(true);
		break;
	default:
		break;
	}

}

void System::setStatus( u32 statusbits )
{
	StatusBitsReg|=statusbits;
}

void System::enableHighFrequencyTask( bool on )
{
	if (on == true)
	{
		//setup timer interrupt
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

		/* Enable TIM clock */
		RCC_APB1PeriphClockCmd( HIFREQ_TASK_TIMER_RCC, ENABLE );

		/* get CPU freq*/
		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq( &RCC_Clocks );

		/* Init PMW timer */
		TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
		TIM_TimeBaseStructure.TIM_Period = RCC_Clocks.PCLK1_Frequency*2 / HIFREQ_TASK_FREQ; // *2 because counter runs 2x bus speed
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; //reload freq
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit( HIFREQ_TASK_TIMER, &TIM_TimeBaseStructure );
		TIM_SetCompare1(HIFREQ_TASK_TIMER, 0);

		/* TIM counter enable */
		TIM_Cmd( HIFREQ_TASK_TIMER, ENABLE );

		/* TIM Interrupts enable */
		TIM_ITConfig(HIFREQ_TASK_TIMER, TIM_IT_CC1, ENABLE);

		/* Enable the TIM gloabal Interrupt */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = HIFREQ_TASK_TIMER_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIFREQ_TASK_ISR_PRIORITY; //higher value=lower priority. priority 0 used by step/dir
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init_GD( &NVIC_InitStructure );

	}
	else
	{
		sys.physIO.doutCHDDir.setState(false);//set CHD as output for r
		TIM_Cmd( HIFREQ_TASK_TIMER, DISABLE );
	}
}

u32 System::getTimeMicrosecs()
{
	return TIM_GetCounter(TIM5);
}

void System::initGeneralTimer()
{
	//setup timer interrupt
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Enable TIM clock */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

	/* get CPU freq*/
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq( &RCC_Clocks );//PCLK1=APB1 PCLK2=APB2 HCLK=CPU

	/* Init PMW timer */
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_Prescaler = RCC_Clocks.PCLK1_Frequency*2 / timeCounterFrequency ; // *2 because counter runs 2x bus speed
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; //reload freq
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );
	TIM_SetCompare1(TIM5, 0);

	/* TIM counter enable */
	TIM_Cmd( TIM5, ENABLE );
}

#define BRAKE_RELEASE_DELAY_MOD 1
void System::updateMechBrakeState()
{
	//this function is called at low frequency (~10Hz)

	//0.3sec delay for activating mech brake to allow motor to spin down before locking brake
	//0.3 is compromise between safety and brake endurance
#ifdef oldway
	static DelayedConditional brakeEngageDelay(this,float(getBrakeEngageDelayMs())/1000.0,true);

	//read status from GC side, also any fault in STM32 side will activate brake (such as lost connection to GC)
	if( brakeEngageDelay.delayedTrue(GCStatusBits & STAT_BRAKING)==true || (FaultBitsReg&FATAL_STM32_FW_FAULTS) )
		physIO.mechBrakeRelease.setState(false);//brake on
	else
		physIO.mechBrakeRelease.setState(true);//brake off

#elif BRAKE_RELEASE_DELAY_MOD

	static DelayedConditional brakeReleaseDelay(this,0.8,true);
	brakeReleaseDelay.setDelay((float)getBrakePoweronReleaseDelayMs()/1000.0);
	bool brakerelease;

	if( (GCStatusBits & STAT_BRAKING) || (FaultBitsReg&FATAL_STM32_FW_FAULTS) )
		brakerelease=false;
	else
		brakerelease=true;

	if(brakeReleaseDelay.delayedTrue(brakerelease))
		physIO.mechBrakeRelease.setState(true);//brake off
	else
		physIO.mechBrakeRelease.setState(false);//brake on

#else
	if( (GCStatusBits & STAT_BRAKING) || (FaultBitsReg&FATAL_STM32_FW_FAULTS) )
		physIO.mechBrakeRelease.setState(false);//brake on
	else
		physIO.mechBrakeRelease.setState(true);//brake off
#endif
}


//update GPI/GPO pins
void System::updatePhysGPIOs_obsolete()
{
	//obsolete
/*
	//GPO1=fault stop
	physIO.doutGPO1.setState(GCStatusBits&STAT_FAULTSTOP || (FaultBitsReg&FATAL_STM32_FW_FAULTS));

	//GPO2=target reached
	physIO.doutGPO2.setState(GCStatusBits&STAT_TARGET_REACHED);

	//GPO3=follow error warning
	physIO.doutGPO3.setState(GCStatusBits&STAT_FERROR_WARNING);

	//GPO3=homing active
	physIO.doutGPO4.setState(GCStatusBits&STAT_HOMING);*/
}

bool System::readInitStateFromGC()
{
	bool fail = false;

	//get input reference mode
	setInputReferenceMode(
			(InputReferenceMode) sys.getParameter(
					SMP_INPUT_REFERENCE_MODE, fail ) );

	//get control mode
	setControlMode((ControlMode)sys.getParameter(SMP_CONTROL_MODE, fail ));

	//read brake control registers
	setBrakePoweronReleaseDelayMs(sys.getParameter(SMP_MECH_BRAKE_RELEASE_DELAY, fail ));
	setBrakeEngageDelayMs(sys.getParameter(SMP_MECH_BRAKE_ENGAGE_DELAY, fail ));

	//set FB1 feedback device source
	int fb1device=sys.getParameter(SMP_FB1_DEVICE_SELECTION, fail );
	int fb2device=sys.getParameter(SMP_FB2_DEVICE_SELECTION, fail );
	switch(fb1device)
	{
	case 1:
		positionFeedbackDevice=Encoder;
		break;
	case 3:
		positionFeedbackDevice=Resolver;
		resolver.enableResolverRead(true);
		break;
	default:
		setFault(FLT_ENCODER,FAULTLOCATION_BASE+201);//unsupported fb1 device choice
		positionFeedbackDevice=None;//even when it failed, set to encoder just to ensure a valid value is present in variable
		break;
	}

	switch(fb2device)
	{
	case 0://same as FB1
		velocityFeedbackDevice=positionFeedbackDevice;
		break;
	default://at the moment no dual loop FB supported so cause fault if FB2 is different from FB1
		setFault(FLT_ENCODER,FAULTLOCATION_BASE+202);//unsupported fb2 device choice
		velocityFeedbackDevice=None;//even when it failed, set to encoder just to ensure a valid value is present in variable
		break;
	}

	DriveFlagBits=sys.getParameter(SMP_DRIVE_FLAGS, fail );

	setpointOffset=sys.getParameter(SMP_ABS_IN_OFFSET, fail );

	//if any GC command failed
	if (fail)
	{
		sys.setFault( FLT_GC_COMM, FAULTLOCATION_BASE+101 );
		return false;
	}
	else
	{
		sys.setStatus( STAT_INITIALIZED );
		return true;
	}
}




