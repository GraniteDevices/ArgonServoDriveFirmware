
/*
 * AnalogIn.cpp
 *
 *  Created on: Oct 29, 2011
 *      Author: tero
 */

#include "AnalogIn.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_adc.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "stm32f2xx_tim.h"

#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)

#include <stdlib.h>


AnalogIn::AnalogIn()
{

	/* Noise observations:
	 * adc clk div 8 = lowest noise on rank 2 channels but no effect on rank 1
	 * rank 2 has significantly higher noise than rank 1
	 * supply filtering -> no effect
	 * signal filtering -> no effect
	 * sample time 15 cycles = good, 3= worse
	 *
	 * see stm32f205_noise.png
	 *
	 * LogicPSU pwm has large effect on noise. PSU-on noise 17count p-p, off 3c p-p
	 * Johtuu enimmakseen kayttojannitesuodatuksista, 3v3 konkka auttaa
	 * TODO: sync ADC to PSU PWM
	 */

	/* sampling rates (per sample, total sampling cycle is 2x longer when 2x2 chans sampled * number of averaging cycles) measured:
	 *
	 * 280kSPS when ADC_Prescaler_Div8 (7.5MHz) ADC_Resolution_12b ADC_TwoSamplingDelay_5Cycles
	 * 368kSPS when ADC_Prescaler_Div6 ADC_Resolution_12b ADC_TwoSamplingDelay_5Cycles
	 * with 4x oversampling and 4 chans total output is 46kSPS = good for 10kHz resolver sampling
	 *
	 * Sample time calculation
	 * t_onesample=SampleTime+12/ADCclock
	 * i.e.
	 * (15+12)/15MHz=1.8µs
	 * and same in 6x oversampling and 4 channels = 21.6µs total
	 *
	 * or 8x OS, 15cycle sampling, 30MHz ADC = 14.4µs (FW V1000 setting)
	 */

	//must not use static table if ADin is located in main() because RTOS start resets stack pointer
	//-> DMA overwrites wrong areas
	ADCDMASampleBuffer=(u16*)malloc(2*ADC_OVERSAMPLING*ADC_CHANS);
	if(ADCDMASampleBuffer==NULL) while(1);//cannot allocate


	/* Enable peripheral clocks *************************************************/
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC,
			ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE );

	/*options:
	 #define ADC_SampleTime_3Cycles                    ((uint8_t)0x00)
	 #define ADC_SampleTime_15Cycles                   ((uint8_t)0x01)
	 #define ADC_SampleTime_28Cycles                   ((uint8_t)0x02)
	 #define ADC_SampleTime_56Cycles                   ((uint8_t)0x03)
	 #define ADC_SampleTime_84Cycles                   ((uint8_t)0x04)
	 #define ADC_SampleTime_112Cycles                  ((uint8_t)0x05)
	 #define ADC_SampleTime_144Cycles                  ((uint8_t)0x06)
	 #define ADC_SampleTime_480Cycles                  ((uint8_t)0x07)
	 */
	u8 sampleTime = ADC_SampleTime_144Cycles; //15 has better SNR than 3, but no further improvement at 28
	//u8 sampleTime = ADC_SampleTime_3Cycles; //15 has better SNR than 3, but no further improvement at 28

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC0-3 as analog inputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
			| GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit( &DMA_InitStructure );
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &ADCsamples;
	//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) ADCsamples;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) ADCDMASampleBuffer;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC_CCR_ADDRESS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//DMA_InitStructure.DMA_BufferSize = ADC_OVERSAMPLING*ADC_CHANS/2;
	//DMA_InitStructure.DMA_BufferSize = 16;
	DMA_InitStructure.DMA_BufferSize = ADC_OVERSAMPLING*ADC_CHANS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init( DMA2_Stream0, &DMA_InitStructure );

	/* DMA2_Stream0 enable */
	DMA_Cmd( DMA2_Stream0, ENABLE );

	/* ADC Common Init */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	//no difference in noise found between ADC clock settings
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; //Div4: ADC clock 60/4=15MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//no effect in sumiltanous sampleing, only interleaved mode
	ADC_CommonInit( &ADC_CommonInitStructure );

	//enable temp sensor and internal 1.25v reference. note: sampling time of both should be 10µs or more
//	ADC_TempSensorVrefintCmd(ENABLE);

	/* Init ADC1 */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit( &ADC_InitStructure );
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

	/*  timer sync doesnt work for some reason
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv  =ADC_ExternalTrigConv_T1_CC1;
	 */

	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_CHANS/2*ADC_OVERSAMPLING;
	ADC_Init( ADC1, &ADC_InitStructure );
	ADC_Init( ADC2, &ADC_InitStructure );

	/* ADC1 regular channels 10, 11 configuration */
	for(int i=0;i<ADC_OVERSAMPLING;i++)
	{
		//ADC_RegularChannelConfig( ADC1, ADC_Channel_TempSensor, 1+i*2, sampleTime );
//		ADC_RegularChannelConfig( ADC1, ADC_Channel_Vrefint, 2+i*2, sampleTime );
		ADC_RegularChannelConfig( ADC1, ADC_Channel_10, 1+i*2, sampleTime );//AIN1
		ADC_RegularChannelConfig( ADC1, ADC_Channel_12, 2+i*2, sampleTime );//CHB
	}

	/* Enable ADC1 DMA since ADC1 is the Master*/
	ADC_DMACmd( ADC1, ENABLE );


	/* ADC2 regular channels 11, 12 configuration */
	for(int i=0;i<ADC_OVERSAMPLING;i++)
	{
	//	ADC_RegularChannelConfig( ADC2, ADC_Channel_Vrefint, 1+i*2, sampleTime );
//		ADC_RegularChannelConfig( ADC2, ADC_Channel_TempSensor, 2+i*2, sampleTime );
		ADC_RegularChannelConfig( ADC2, ADC_Channel_11, 1+i*2, sampleTime );//AIN2
		ADC_RegularChannelConfig( ADC2, ADC_Channel_13, 2+i*2, sampleTime );//CHA
	}

	/* Enable DMA request after last transfer (Multi-ADC mode)  */
	ADC_MultiModeDMARequestAfterLastTransferCmd( ENABLE );

	/*test isr*/
	/* Enable the ADC gloabal Interrupt */
/*	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //higher value=lower priority. priority 0 used by step/dir
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init_GD( &NVIC_InitStructure );
*/

	/* Enable ADC1 */
	ADC_Cmd( ADC1, ENABLE );

	/* Enable ADC2 */
	ADC_Cmd( ADC2, ENABLE );

	//now must start conversion once or frequently depending on continuous conv mode
	startSampling();
}

AnalogIn::~AnalogIn()
{
}



u16 AnalogIn::getSample( Channel channel )
{

	if (channel < 0 || channel >= ADC_CHANS)
		return 0;
	return ADCsamples[channel];
}

s16 AnalogIn::getVoltage(Channel channel)
{
	//return scale=+16384=+10V -16384=-10V
	//getSample returns 0-4095

//	return (s32(getSample(channel))*4);

	switch(channel)
	{
		case AnaIn1:
		case AnaIn2:
			//124=-10V=-16384
			//3909=10V=16384
			//0V=2048
			return (s32(getSample(channel))-ADC_OFFSET_VALUE_ANAIN)*(144250/8)/16384;
			break;
		case EncA:
		case EncB:
			//analog gain 1.80333x with 10+2.2k & 22k feedback resistors
			//24425=10V (out of range, input amp V gain 1.80333X)
			//-20329=-10V
			//0V=2048

			//in reality this gain seems to be about 10% too much compared to reality due to RS422 receiver
			//input internal resistances that also load adc inputs and cause reduction of gain
			//however we let it be like this as enc analog inputs are not needed to be measured accurately anyways
			return (s32(getSample(channel))-ADC_OFFSET_VALUE_ENC)*(11996/8)/16384;
			break;
	}
}

float AnalogIn::getVoltageVolts(Channel channel)
{
	//return scale=+16384=+10V -16384=-10V
	//getSample returns 0-32768

	switch(channel)
	{
		case AnaIn1:
		case AnaIn2:
			//124*16=-10V=-16384
			//3909*16=10V=16384
			//0V=2048*16
			return float(s32(getSample(channel))-ADC_OFFSET_VALUE_ANAIN)*(0.0053735/8.0);
			break;
		case EncA:
		case EncB:
			//analog gain 1.80333x with 10+2.2k & 22k feedback resistors
			//24425=10V (out of range, input amp V gain 1.80333X)
			//-20329=-10V
			//0V=2048

			//in reality this gain seems to be about 10% too much compared to reality due to RS422 receiver
			//input internal resistances that also load adc inputs and cause reduction of gain
			//however we let it be like this as enc analog inputs are not needed to be measured accurately anyways
			return float(s32(getSample(channel))-ADC_OFFSET_VALUE_ENC)*(4.4688e-4/8.0);
			break;
	}
	return -1.0;//error if go here
}

void AnalogIn::startSampling()
{
	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv( ADC1 );
}

//optimization mandotary to be able to run at 40kHz and 8 sample averaging
//at O0 it otherwise takes 50% of cpu time
//with O3 reduced to ̃~10%
void OPTIMIZE_FUNCTION AnalogIn::storeSamples()
{
	int i, sum, channel;

	//calculate average of samples in DMA buffer and store averages in ADCsamples
	for(channel=0;channel<ADC_CHANS;channel++)
	{
		sum=0;
		for(i=0;i<ADC_OVERSAMPLING;i++)
			sum+=ADCDMASampleBuffer[channel+i*ADC_CHANS];
		ADCsamples[channel]=8*sum/ADC_OVERSAMPLING; //sampes are now 0-32767 instead of 0-4095

		//no avg:
		//i=0;ADCsamples[channel]=8*ADCDMASampleBuffer[channel+i*ADC_CHANS];
	}
}

//for analyzing/debugging adc noise
void AnalogIn::collectNoiseStatistics()
{
	const int span=20, ignoreNruns=100000, stopAt=200000;
	static int runs=0;
	static int cumsum[ADC_CHANS][span];
	static int middle[ADC_CHANS];
	static bool initialized=false;
	int i,s;

	runs++;

	if(runs>ignoreNruns && initialized==false)
	{
		for(i=0;i<ADC_CHANS;i++)
			for(s=0;s<span;s++)
			{
				cumsum[i][s]=0;
				middle[i]=ADCsamples[i]/8;
			}
		initialized=true;

	}

	if(initialized && runs<stopAt)
	{
		//collect
		for(i=0;i<ADC_CHANS;i++)
		{
			s=ADCsamples[i]/8-middle[i]+span/2;
			if(s<0)s=0;
			if(s>=span)s=span-1;

			cumsum[i][s]++;
		}
	}
	if(runs>stopAt)
	{
		//volatile int acr=FLASH->ACR;//verify prefetch setting
		NOP;//place breakpoint here and inspect cumsum table
	}

	/* some results:
	 *
	 * no avg prefetch on ch0: {4, 35, 45, 229, 621, 6542, 2848, 13928, 24912, 33221, 10920, 4377, 1732, 454, 97, 28, 4, 1, 0, 1}
	 * ch0 again {5, 7, 40, 46, 299, 613, 2494, 3239, 12293, 18691, 26801, 11106, 20551, 2087, 1387, 209, 117, 7, 2, 5}
	 * others chans similar
	 *
	 * prefetch on:
	 * ch0 w avg8:{0, 0, 0, 0, 0, 0, 2, 4, 232, 9324, 56708, 32450, 1267, 11, 1, 0, 0, 0, 0, 0}

	 * ch0 noavg but prefetch off: {5, 15, 153, 47, 244, 2044, 5174, 4461, 11377, 30092, 14381, 24017, 5190, 1922, 608, 236, 27, 4, 0, 2}
	 *
	 * same as above but 100khz DSCpsu off {5, 0, 1, 22, 9, 84, 277, 3094, 1071, 7696, 13510, 23785, 22027, 22908, 2677, 2294, 358, 153, 18, 10}
	 *
	 */
}




