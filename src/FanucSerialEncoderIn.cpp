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

#include "FanucSerialEncoderIn.h"
#include "stm32f2xx.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_rcc.h"
#include "misc.h"//==NVIC
#include "globals.h"
#include "utils.h"

//10kHz switching
//#define RESOLVER_FREQ 10000

#define FAULTLOCATION_BASE 930000

FanucSerialEncoderIn::~FanucSerialEncoderIn()
{
	/*fanuc serial encoder ouputs data like UART at 1024kbps and word length is 76 or 80 bits. 
	need special serial receiver to read that long words. can be difficult with software timing*/
}

FanucSerialEncoderIn::FanucSerialEncoderIn( System *parent ) :
		parentSys( parent ), countsPerRev( 32768 ),
				lastAngle( 0 ), currAngle( 0 ), outputCount( 0 ), unrolledCount( 0 ), lastOutputCount(
						0 )
{
}

void FanucSerialEncoderIn::enableRead( bool on )
{
	if (on == true)
	{
		sys.physIO.doutCHDDir.setState( true ); //set CHD as output for fanuc REQ signal
		return;
	}
	else
	{
		sys.physIO.doutCHDDir.setState( false ); //set CHD as input
	}
}

void FanucSerialEncoderIn::customNOPdelay( unsigned long dly )
{
	unsigned long i;
	for( i = 0; i < dly; i++ )
	{
		//attempt to get same delay regardless of compiler optimizations
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
		asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n" );
	}
}


u16 FanucSerialEncoderIn::getAngle()
{
	static u16 cnt=0;
	cnt++;
	sys.physIO.doutCHD.setState( true ); 
//	Delay_1us(6); //FIXME need more consistent timing The REQ pulse must be between 6.5 and 9.1 us wide, positive-going. Outside this range, the encoder gives no response.
	customNOPdelay(9);//this too, varies due to some other code interrupting this delay
	sys.physIO.doutCHD.setState( false );
	
	return cnt;
}

s16 FanucSerialEncoderIn::getVelocity()
{
	return s16( outputCount - lastOutputCount );
}


