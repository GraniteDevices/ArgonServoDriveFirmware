/*
 * EncoderIn.cpp
 *
 *  Created on: Dec 1, 2011
 *      Author: tero
 */

#include "EncoderIn.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_tim.h"

EncoderIn::EncoderIn()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );

	//setup pins
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource6, GPIO_AF_TIM3 );
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource7, GPIO_AF_TIM3 );
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource8, GPIO_AF_TIM3 );
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource9, GPIO_AF_TIM3 );

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	/*TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	 TIM_TimeBaseStructure.TIM_Period = 0xffff;
	 TIM_TimeBaseStructure.TIM_Prescaler = 0;
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);*/

	/*configure input filtering
	 * fDTS=60MHz (configured in TIM3_CR1), verify!
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
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; /*!< Specifies the active edge of the input signal. This parameter can be a value of @ref TIM_Input_Capture_Polarity */
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; /*!< Specifies the input. This parameter can be a value of @ref TIM_Input_Capture_Selection */
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; /*!< Specifies the Input Capture Prescaler. This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
	TIM_ICInitStruct.TIM_ICFilter = 8; /*!< Specifies the input capture filter. This parameter can be a number between 0x0 and 0xF */

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
	TIM_ICInit( TIM3, &TIM_ICInitStruct );
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
	TIM_ICInit( TIM3, &TIM_ICInitStruct );
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
	TIM_ICInit( TIM3, &TIM_ICInitStruct );
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4; /*!< Specifies the TIM channel.This parameter can be a value of @ref TIM_Channel */
	TIM_ICInit( TIM3, &TIM_ICInitStruct );


	//set in encoder mode
	TIM_SetAutoreload( TIM3, 0xffff );
	TIM_EncoderInterfaceConfig( TIM3, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_Cmd( TIM3, ENABLE );
}

EncoderIn::~EncoderIn()
{
}

void EncoderIn::update()
{
	prevCount=currCount;
	currCount=TIM_GetCounter( TIM3 );
	velocity=S16(currCount-prevCount);
}

u16 EncoderIn::getCounter()
{
	return currCount;
}

s16 EncoderIn::getVelocity()
{
	return velocity;
}


/* counter value when index pulse was seen last time */
u16 EncoderIn::getCounterAtIndex()
{
	TIM_ClearFlag(TIM3,TIM_FLAG_CC3);
	return TIM_GetCapture3( TIM3 );
}

/* return true if index value has been updated and can be read with getCounterAtIndex()
 * note: calling getCounterAtIndex() will reset this status */
bool EncoderIn::hasIndexUpdated()
{
	if( TIM_GetFlagStatus(TIM3, TIM_FLAG_CC3) == SET )
		return true;
	else
		return false;
}

