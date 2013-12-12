/*
 * DSCPowerTask.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: tero
 */

#include "DSCPowerTask.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_rcc.h"
#include "queue.h"
#include "types.h"

xQueueHandle PSUCommandQueue=0;

#define PSU_OFF 1
#define PSU_ON 2

//100kHz switching
#define GCPSU_FREQ 120000
int TimerPeriod;

void GCPSUInitHW()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	/* Enable TIM1 clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

	/* get CPU freq*/
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq( &RCC_Clocks );

	/* Configure TIM1_CH2N (PB14) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	/* Connect TIM pins to AF1 */
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource14, GPIO_AF_TIM1 );

	/* Init PMW timer */
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Period = RCC_Clocks.HCLK_Frequency / GCPSU_FREQ; //set to 100kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; //reload freq
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM1, &TIM_TimeBaseStructure );
	TimerPeriod=TIM_TimeBaseStructure.TIM_Period;

	/* TIM1 channel2 configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; //0% duty at start
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC2Init( TIM1, &TIM_OCInitStructure );

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 1;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig( TIM1, &TIM_BDTRInitStructure );

	//for ADC sync
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);//center align
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1);//center align

	/* TIM1 counter enable */
	TIM_Cmd( TIM1, ENABLE );

	/* Main Output Enable */
	TIM_CtrlPWMOutputs( TIM1, ENABLE );

}

void GCPSUSetState(bool enabled)
{
	u8 newstate;
	if(enabled)
		newstate=PSU_ON;
	else
	{
		TIM_SetCompare2( TIM1, 0 );
		newstate=PSU_OFF;
	}

	xQueueSend(PSUCommandQueue,&newstate,0);
}

void GCPSUInit()
{
	PSUCommandQueue = xQueueCreate( 5, sizeof( u8 ) );
}

void GCPSUTask( void *pvParameters )
{
	int i;
	u8 currentstate=PSU_OFF;
	u8 newstate=PSU_OFF;

	GCPSUInitHW();

	for(;;)
	{
		//wait for new PSU command, wait infinitely
		if( xQueueReceive( PSUCommandQueue, &newstate, portMAX_DELAY ) )
		{
			if(newstate!=currentstate )
			{
				if(newstate==PSU_ON)
				{
					portTickType xLastWakeTime;
					xLastWakeTime = xTaskGetTickCount();
					/*ramp duty cycle from 1 to 45% slowly to prevent current rush, with this power rise time about 100ms */
					for( i = 1; i <= 180; i++ )
					{
						//delay about 2ms
						vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/500 );
						//vTaskDelayUntil( &xLastWakeTime, 5 );
						TIM_SetCompare2( TIM1, TimerPeriod * i / 400 );
					}
				}
/*				else if(newstate==PSU_OFF)
				{
					TIM_SetCompare2( TIM1, 0 );//moved to DSCPSUSetSate for instant execution
				}*/

				currentstate=newstate;
			}
		}
	}
}

