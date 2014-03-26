/*
 * ResolverIn.cpp
 *
 *  Created on: Sep 15, 2012
 *      Author: tero
 */

/* noise notes:
 *
 * noise is minimum when x & y are equal value and max when they're opposite because
 * common mode noise spikes increasing both same amount causes larger displacement in opposite values.
 * equal values change only vector length but no amplitude.
 *
 * -motor cable shield and ferrite on phases has significant effect on noise
 * -ferrites on resolver cable - no effect
 * -2x ferrites on resolver stator pairs - small effect maybe
 *
 * Try:
 * -better shieldin and also shielded cable for resolver
 * -use ST AN4073 to pick out noise spikes from averaged samples as not all samples are noisy (no pwm edges hit)
 *
 */

#include "ResolverIn.h"
#include "stm32f2xx.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_rcc.h"
#include "misc.h"//==NVIC
#include "globals.h"
#include "utils.h"

//10kHz switching
#define RESOLVER_FREQ 10000

ResolverIn::~ResolverIn()
{

}

ResolverIn::ResolverIn( System *parent ) :
		parentSys( parent ), countsPerRev( 8192 ),
				lastAngle( 0 ), currAngle( 0 ), outputCount( 0 ), unrolledCount( 0 ), lastOutputCount(
						0 )
{
	xacc = yacc = 0;
	samples = 0;
}

void ResolverIn::enableResolverRead( bool on )
{
	if (on == true)
	{

		sys.physIO.doutCHEDir.setState( true ); //set CHD as output for resolver
		return;
#if 0

		//	testTIM5();

		//setup timer interrupt to generate square wave to resolver primary coil

		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

		/* Enable TIM clock */
		RCC_APB1PeriphClockCmd( RESOLVER_TIMER_RCC, ENABLE );

		/* get CPU freq*/
		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq( &RCC_Clocks );

		/* Init PMW timer */
		TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
		TIM_TimeBaseStructure.TIM_Period = RCC_Clocks.HCLK_Frequency / RESOLVER_FREQ /4; //set to 10kHz. div by 4 because TIM4 runs at 1/4 of core speed
		//TIM_TimeBaseStructure.TIM_Period = 65535;
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;//reload freq
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit( RESOLVER_TIMER, &TIM_TimeBaseStructure );
		TIM_SetCompare1(RESOLVER_TIMER, 0);
		/*TIM_OCInitTypeDef  TIM_OCInitStructure;
		 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
		 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		 TIM_OCInitStructure.TIM_Pulse = 3000;
		 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		 TIM_OC1Init(TIM3, &TIM_OCInitStructure);

		 TIM_OC1PreloadConfig(RESOLVER_TIMER, TIM_OCPreload_Disable);*/

		/* TIM counter enable */
		TIM_Cmd( RESOLVER_TIMER, ENABLE );

		/* TIM Interrupts enable */
		TIM_ITConfig(RESOLVER_TIMER, TIM_IT_CC1, ENABLE);

		/* Enable the TIM gloabal Interrupt */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = RESOLVER_TIMER_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //higher value=lower priority. priority 0 used by step/dir
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init_GD( &NVIC_InitStructure );

#endif
	}
	else
	{
		sys.physIO.doutCHEDir.setState( false ); //set CHD as input
#if 0
				TIM_Cmd( RESOLVER_TIMER, DISABLE );
#endif
	}
}

#include <math.h>
u16 ResolverIn::getAngle()
{
	//TODO

	/* status 9.2012:
	 * pinnia heilutetaan systemin hf taskista
	 *
	 *
	 * update:kytkis loytyy psvn skemana, natti RC ulostulo resosta
	 *
	 * luenta:
	 * säädetään keskeytys 20kHz:iin ja luetaan adc arvot talteen pulssin keskeltä
	 *
	 * skooppi:
	 * transcend muistitikulla skooppikuvia resolverista kun lähdön kanssa sarjassa 6.8 ohm ja
	 * kelan rinnan 0.33uf -> melkein siniaalto mutta LC oskillaatiota reunoissa
	 *
	 * sin ja kelt: käämin päät maahan nähden
	 * pink: erotus käämin yli
	 * vih: toision ulostulo
	 *
	 * Tarvinnee myös isomman sarjaresistanssin koska driveri lämpenee 70Chen 25Cssä ja ulostuloksi tulee
	 * yli +-2v
	 *
	 */

	/* TODO add fault detection if voltages of both channels less than 100mV or so
	 * this detects disconnected resolver and prevents angle going crazy*/

	lastAngle = currAngle;
	float x, y; //parentSys->physIO.ADin.getVoltageVolts(AnalogIn::EncA),parentSys->physIO.ADin.getVoltageVolts(AnalogIn::EncB)

	getSamples( x, y );
	currAngle = u16( s32( 65536.0 * atan2f( x, y ) * (0.5 / M_PI) ) );

	lastOutputCount = outputCount;
	unrolledCount += s16( currAngle - lastAngle );
	outputCount = unrolledCount / (65536 / countsPerRev);

	return outputCount;
}

s16 ResolverIn::getVelocity()
{
	return s16( outputCount - lastOutputCount );
}

void ResolverIn::addSamples( float x, float y, bool upcycle )
{
	if (samples < 6) //limit amount to get always same number of pos/neg samples
	{
		samples++;
		float polarity = 1;
		if (upcycle == false)
			polarity = -1;
		xacc += x * polarity;
		yacc += y * polarity;
	}
}

void ResolverIn::getSamples( float &x, float &y )
{
	x = xacc / float( samples );
	y = yacc / float( samples );

	xacc = yacc = 0;
	samples = 0;
}

float ResolverIn::customAtan2( float y, float x )
{
	if (x > 0)
		return atanf( y / x );
	else if (y >= 0 && x < 0)
		return atanf( y / x ) + M_PI;
	else if (y < 0 && x < 0)
		return atanf( y / x ) - M_PI;
	else if (y > 0 && x == 0)
		return M_PI / 2;
	else if (y < 0 && x == 0)
		return -M_PI / 2;

	return 0; //error
}


#if ST_APPNOTE_CODE
/** @brief Sort the N ADC samples
* @param ADC samples to be sorted
* @param Numbre of ADC samples to be sorted
* @retval None
*/
void Sort_tab(uint16_t tab[], uint8_t lenght)
{
	uint8_t l=0x00, exchange =0x01;
	uint16_t
	tmp=0x00;
	/* Sort tab */
	while(exchange==1)
	{
		exchange=0;
		for(l=0; l<lenght-1; l++)
		{
			if( tab[l] > tab[l+1] )
			{
				tmp = tab[l];
				tab[l] = tab[l+1];
				tab[l+1] = tmp;
				exchange=1;
			}
		}
	}
}

/**
 * @brief Get the average of N-X ADC samples
 * @param Numbre of ADC samples to be averaged
 * @param Numbre of ADC samples to be averaged
 * @retval The average value
 */
uint16_t ADC_GetSampleAvgNDeleteX( uint8_t N, uint8_t X )
{
	uint32_t avg_sample = 0x00;
	uint16_t adc_sample[8] =
	{ 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t index = 0x00;
	for( index = 0x00; index < N; index++ )
	{
		/* ADC start conv */
		ADC_SoftwareStartConv( ADC1 );
		/* Wait end of conversion */
		while (ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) == RESET)
			;
		/* Store ADC samples */
		adc_sample[index] = ADC_GetConversionValue( ADC1 );
	}
	/* Sort the N-X ADC samples */
	Sort_tab( adc_sample, N );
	/* Add the N ADC samples */
	for( index = X / 2; index < N - X / 2; index++ )
	{
		avg_sample += adc_sample[index];
	}
	/* Compute the average of N-X ADC sample */
	avg_sample /= N - X;
	/* Return average value */
	return avg_sample;
}


#endif

