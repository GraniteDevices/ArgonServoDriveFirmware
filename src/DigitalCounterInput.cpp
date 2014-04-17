/*
 * DigitalCounterInput.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: tero
 */

#include "DigitalCounterInput.h"
#include "misc.h"
#include "stm32f2xx_exti.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_syscfg.h"
#include "stm32f2xx_tim.h"
#include "globals.h"
#include "utils.h"



DigitalCounterInput::DigitalCounterInput():
	PWMIn1(MAX_PWM_PERIOD_TIME),
	PWMIn2(MAX_PWM2_PERIOD_TIME)
{
	countMode=None;
	setCountMode( countMode );
}

DigitalCounterInput::~DigitalCounterInput()
{
	// TODO Auto-generated destructor stub
}

void DigitalCounterInput::setCountingDirection( bool up )
{
	//caution: if change config, check that this still writes correct values
	//for optimization reason we don't change one bit but write all bits with fixed value
	if (up)
		TIM2->CR1 = 0x01;
	else
		TIM2->CR1 = 0x11;
}

void DigitalCounterInput::setCountMode( CountMode mode )
{
	bool resetCounter=false;
	//don't require counter reset if switching between quadrature and pulsetrain modes
	/* always reset now because granity sets divider on apply so any value in setpoint gets rotated always on apply
	if( (countMode!=mode && countMode!=PulseTrain && mode==Quadrature)
			|| (countMode!=mode && countMode!=Quadrature && mode==PulseTrain) )
		resetCounter=true;*/
	resetCounter=true;

	countMode = mode;

	//TIM2:pwm1, step dir, quadrature sources
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource0, GPIO_AF_TIM2 );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource1, GPIO_AF_TIM2 );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	//TIM1:connected to analog inputs (anain2=TIM1 CH4 anain1=TIM1 ETR), used for second pwm input
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource11, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource12, GPIO_AF_TIM1 );
	RCC_APB1PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE);


	if (countMode == PulseTrain)
	{
		EXTI_InitTypeDef EXTI_InitStructure;

		/*DIR pin init*/
		//skip initializing IO clock and direction as theyre done DigitalInputPin class
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE );

		/* Connect Button EXTI (Ext inq) Line to GPIO Pin */
		SYSCFG_EXTILineConfig( PULSETRAIN_DIR_EXTI_PORT_SOURCE,
				PULSETRAIN_DIR_EXTI_PIN_SOURCE );

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = PULSETRAIN_DIR_EXTI_LINE;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init( &EXTI_InitStructure );

		/* Enable the PULSETRAIN_DIR_EXTI_IRQ Interrupt */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = PULSETRAIN_DIR_EXTI_IRQ; //dir connected to BP11 so its EXTI 10-15 (11)
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init_GD( &NVIC_InitStructure );

		//set counting direction according to DIR (HSIN2) pin state
		sys.digitalCounterInput.setCountingDirection( sys.physIO.getDirectionPinState() );

		/*end of DIR pin init*/
		/*STEP pin init*/
		/* Init pulse counter on PA0-
		 * TODO: it may be possible to use fully HW based step/dir with two timers:
		 *   use gated mode with exterlan clock.
		 *   one timer with gate low active and second with gate high active
		 *   then get two counter values and totalcount=cnt1-cnt2
		 *
		 *   Now direction of counting is switched in interrupt.
		 *   Problem may arise if noisy or fast toggling direction input generating high rate of ISRs.
		 */
		if(resetCounter)
			setCounter(0);

#ifdef MODE2
		/*mode2 seems to use odd filtering on ETR input.
		 * Filter after edge detection
		 * and prescaler, so avoid that.
		 */
		TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF,
				TIM_ExtTRGPolarity_NonInverted, 8);
#else
		/*mode1 uses TI1 input as clock source and filtering
		 * is done right after
		 * pin sampling.
		 */
		TIM_ETRClockMode1Config( TIM2, TIM_ExtTRGPSC_OFF,
				TIM_ExtTRGPolarity_NonInverted, 8 ); //filter value here has no effect!
		TIM_SelectInputTrigger( TIM2, TIM_TS_TI1FP1 );

		/*configure input filtering
		 * fDTS=60MHz (configured in TIM3_CR1), verify! Applies only when CR1.CKD=00
		 * Filtering times (minimum pulse length to cause state change).
		 * physically verified with pulse gen on TIM2 which has same clock freq
		 * 1=33ns
		 * 2=66ns
		 * 3=133ns
		 * 4=200ns
		 * 5=266ns
		 * 6=400ns
		 * 7=533ns
		 * 8=800ns
		 * 9=1.066us
		 * 10=1.33us
		 * 11=1.60us
		 * 12=2.13us
		 * 13=2.66us
		 * 14=3.2us
		 * 15=4.2us
		 */
		TIM_ICInitTypeDef TIM_ICInitStruct;
		//3 first values doesnt matter in this app
		TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; /*!< Specifies the active edge of the input signal. This parameter can be a value of @ref TIM_Input_Capture_Polarity */
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*!< Specifies the input. This parameter can be a value of @ref TIM_Input_Capture_Selection */
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; /*!< Specifies the Input Capture Prescaler. This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
		//sets step input filtering
		TIM_ICInitStruct.TIM_ICFilter = 3; /*!< Specifies the input capture filter. This parameter can be a number between 0x0 and 0xF */
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
		TIM_ICInit( TIM2, &TIM_ICInitStruct );

#endif
		TIM_Cmd( TIM2, ENABLE );
		/*end of STEP pin init*/


	}
	else if (countMode == PWM) //capture PWM duty cycle in HSIN2
	{
		/* FIRST PWM INPUT ON TIM1 */
		/* Disable DIR pin interrupt (just in case pulsetrain mode was active previously) */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = PULSETRAIN_DIR_EXTI_IRQ; //dir connected to BP11 so its EXTI 10-15 (11)
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init_GD( &NVIC_InitStructure );

		/* init PWM capture */
		TIM_ICInitTypeDef TIM_ICInitStructure;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 8;
		TIM_PWMIConfig( TIM2, &TIM_ICInitStructure );

		/* Select the TIM2 Input Trigger: TI2FP2 */
		TIM_SelectInputTrigger( TIM2, TIM_TS_TI2FP2 );

		/* Select the slave Mode: Reset Mode */
		TIM_SelectSlaveMode( TIM2, TIM_SlaveMode_Reset );
		TIM_SelectMasterSlaveMode( TIM2, TIM_MasterSlaveMode_Enable );

		/* TIM enable counter */
		TIM_Cmd( TIM2, ENABLE );

		setCounter(0);


		/* SECOND PWM ON TIM1  (done on lower level than TIM2 because ST perpih library doesn't implement it for TI3-4) */

		/* Usage of second pwm input:
		 *
		 * Tie analog inputs 1&2 together and feed PWM to them.
		 * Usage of 3.3V source barely stable when connected to pos inputs and neg inputs biased to DC 2.3V vs GND,
		 * Recommend at least 5V signals with 3-3.5V bias.
		 */


		TIM_PrescalerConfig(TIM1, 1, TIM_PSCReloadMode_Immediate);//set clock divided by 2 (prescaler value 1) as TIM1 runs 120MHz as TIM2 only 60MHz
	    /* TI4 Configuration */
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;//invert signal as anain op-amp is inverting
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM1, &TIM_ICInitStructure);
	    /* TI3 Configuration */
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//invert signal as anain op-amp is inverting
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
		TIM_ICInit(TIM1, &TIM_ICInitStructure);

		/* Select the TIM4 Input Trigger: ETR */
		TIM_ETRConfig(TIM1,TIM_ExtTRGPSC_OFF ,TIM_ExtTRGPolarity_NonInverted,
				TIM_ICInitStructure.TIM_ICFilter);//invert signal as anain op-amp is inverting
		TIM_SelectInputTrigger( TIM1, TIM_TS_ETRF );

		/* Select the slave Mode: Reset Mode */
		TIM_SelectSlaveMode( TIM1, TIM_SlaveMode_Reset );
		TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable );


		/* TIM enable counter */
		TIM_Cmd( TIM1, ENABLE );


#if 0
	    TI4_Config(TIM4, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 8);
	    /* Set the Input Capture Prescaler value */
	    TIM_SetIC4Prescaler(TIM4, TIM_ICPSC_DIV1);
	    /* TI3 Configuration */
	    TI3_Config(TIM4, TIM_ICPolarity_Falling, TIM_ICSelection_IndirectTI, 8);
	    /* Set the Input Capture Prescaler value */
	    TIM_SetIC3Prescaler(TIM4, TIM_ICPSC_DIV1);
#endif



	}
	else if (countMode == Quadrature)
	{
		/* Disable DIR pin interrupt (just in case pulsetrain mode was active previously) */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = PULSETRAIN_DIR_EXTI_IRQ; //dir connected to BP11 so its EXTI 10-15 (11)
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init_GD( &NVIC_InitStructure );

		/*set quadrature capture*/
		TIM_ICInitTypeDef TIM_ICInitStruct;
		TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; /*!< Specifies the active edge of the input signal. This parameter can be a value of @ref TIM_Input_Capture_Polarity */
		TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*!< Specifies the input. This parameter can be a value of @ref TIM_Input_Capture_Selection */
		TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; /*!< Specifies the Input Capture Prescaler. This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
		TIM_ICInitStruct.TIM_ICFilter = 6; /*!< Specifies the input capture filter. This parameter can be a number between 0x0 and 0xF */

		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
		TIM_ICInit( TIM2, &TIM_ICInitStruct );
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_2; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
		TIM_ICInit( TIM2, &TIM_ICInitStruct );

		//set in encoder mode
		//TIM_SetAutoreload( TIM2, 0xffff );
		TIM_EncoderInterfaceConfig( TIM2, TIM_EncoderMode_TI12,
				TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
		TIM_Cmd( TIM2, ENABLE );

		if(resetCounter)
			setCounter(0);

	}
	else// countMode==None or unknown
	{
		setCounter(0);
	}

	//setCounter(0); error:here causes motor to run to 0 pos when granity apply button pressed

}

s32 DigitalCounterInput::getCounter( int sourceNr )
{
	if (countMode == PulseTrain || countMode == Quadrature)
	{
		return TIM_GetCounter( TIM2 );
	}
	else if (countMode == PWM)
	{
		if(sourceNr==0)
			return PWMIn1.computePWMInput(TIM_GetCapture2( TIM2 ),TIM_GetCapture1( TIM2 ),TIM_GetCounter( TIM2 ));
		else if(sourceNr==1)
			return PWMIn2.computePWMInput(TIM_GetCapture3( TIM1 ),TIM_GetCapture4( TIM1 ),TIM_GetCounter( TIM1 ));
		else//invalid sourcenr
			return 0;
	}
	else if(countMode==None)
	{
		return 0;
	}
	else
	{
		return -1; //error, FW bug
	}
}

void DigitalCounterInput::setCounter( s32 newvalue )
{
	if (countMode == PulseTrain || countMode == Quadrature)
	{
		TIM_SetCounter( TIM2, newvalue );
	}
	else if (countMode == PWM)
	{

	}
}


PWMInputComputing::PWMInputComputing( u32 maxPulselength )
{
	this->maxCounterValue=maxPulselength;
	noPWMsignal=true;
}

s32 PWMInputComputing::computePWMInput( u32 period, u32 pulselength, u32 timerCounter )
{
	//if PWM in is stuck high or no edges present, then timer counter keeps counting and capture regs may have old values
	//so set PWM signal status bad
	if (timerCounter > maxCounterValue || period <= 0
			|| period > maxCounterValue || pulselength > period)
	{
		//clearing capture registers seems impossible on HW so workaround by software
		noPWMsignal = true; //disable pwm readout on this cycle
	}
	else //check if edges now available
	{
		//if signal edges detected, assume counter to be reset and edge times changed due to noise at least
		if ((period != prevPeriod || prevPulselength != pulselength)
				&& timerCounter < maxCounterValue)
		{
			noPWMsignal = false; //assume PWM signal good
		}
	}

	prevPeriod = period;
	prevPulselength = pulselength;

	if (noPWMsignal == false)
	{
		/* Duty cycle computation */
		//s32 DutyCycle = s32((uint64_t(pulselength) * 100000ULL) / (period)); //0-100000 scale
		s32 DutyCycle = s32(
				(uint64_t( pulselength ) * 32786ULL) / (period) ) - 16384; //-16k..16k scale
		return DutyCycle;
	}
	else
		//cant div by zero, probably no input edges present
		return 0;
}



