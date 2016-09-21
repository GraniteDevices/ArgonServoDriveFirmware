/*
 * SinCosEncoder.cpp
 *
 *  Created on: 23.4.2015
 *      Author: Tero
 */
#include <math.h>
#include "System.h"
#include "SinCosEncoder.h"

SinCosEncoder::SinCosEncoder( System *parent ):parentSys(parent)
{
	interpolationFactor=64;

	a=1;
	b=0;
	incr=0;
	initialized=false;
	prevInterpolatedPos=0;
	interpolatedPos=0;
	analogAngle=0;
	incrAdder=0;
	velocity=0;
	resetAdder=0;
	//offset_a=offset_b=-0.64;//LinMot
	offset_a=offset_b=0;
	gain_a=gain_b=defaultGainCoeff;
	calibrationSamplesToGo=calibrationAngles*calibrationSamples;
	calibrationComplete=false;
}

SinCosEncoder::~SinCosEncoder() {

}

//rotate slowly and call this frequently to have sincos intialized. after init it provides interpolation
void SinCosEncoder::initialize()
{
	//const float threshold=0.15;

	if(initialized==false)//in initialize, find a major quadrature position (not in middle of transition)
	{
		//produces triangle wave where abs(a)=abs(b) when angl = 0.25*32768 (0.25 = float atan2(1,1)/pi)
		//u16 angl=fxpt_atan2(abs(s16(a*defaultGainCoeff)),abs(s16(b*defaultGainCoeff)));

		//parentSys->setDebugParam(3,angl);
		//initialize when we are at certain angles (where a and b are near max values so digital angle is defined well)

		/*TODO optimize. NOTE testing it now
		 * as (angl > u16(0.25*32768.0*(1.0-threshold)) && angl < u16(0.25*32768.0*(1.0+threshold))
		 * selects angle range of 0.25*.85 - 0.25*1.15 rad angle range, it equals sine&cos values between 0.619-0.785
		 * -> enough to ensure that a & b are near each other. (10*a)>(8*b) && (10*b)>(8*a) NOT TESTED
		 */
		//if( angl > u16(0.25*32768.0*(1.0-threshold)) && angl < u16(0.25*32768.0*(1.0+threshold))
		if( 10*a>8*b && 10*b>8*a
				&& fabsf(a)>minimumAmplForDigitalReadout && fabsf(b)>minimumAmplForDigitalReadout)
		{
			//atan2 wraps around when a=0 and b<0
			int quadrant;
			if(a>0)//2301
				if(b>0) //ab=++
					quadrant=0;
				else //ab=+-
					quadrant=1;
			else
				if(b>0)//ab=-+
					quadrant=3;
				else//ab=--
					quadrant=2;

			initialized=true;

			incrAdder=(-incr+quadrant)&3;

			//parentSys->setDebugParam(3,quadrant);
			//parentSys->setDebugParam(4,initIncr);
			//parentSys->setDebugParam(5,initAnalogAngle);
			//parentSys->setDebugParam(6,incrAdder);
		}
	}
	else//else because doing colect and init same time overloads cpu and watchdog triggers
		if(calibrationDataCollected()==false)
			collectCalibrationData();
		else
			useCalibrationData();//call few times to fade in gains/offfs
}

OPTIMIZE_FUNCTION bool SinCosEncoder::hasIndexUpdated() {
	return parentSys->encoder.hasIndexUpdated();
}

OPTIMIZE_FUNCTION u16 SinCosEncoder::getCounterAtIndex() {
	return ((0xffff-parentSys->encoder.getCounterAtIndex())+incrAdder)*interpolationFactor/4+resetAdder;
}


void SinCosEncoder::encoderResetAt(u16 count)
{
	incrAdder+=count;
	incrAdder&=3;
	incr=0;
	update();
	resetAdder=0xffff-interpolatedPos;
}

void SinCosEncoder::collectCalibrationData()
{
	static u16 prevIncr;

	if( s16(incr-prevIncr)<=1 )//only collect data at low speeds
	{
		u16 aangle=calcAngle();
		aangle=(u32(aangle)*calibrationAngles)>>16;//scale angle to 0..calibrationSteps
		if(calibrationData[aangle].count<calibrationSamples)
		{
			calibrationData[aangle].count++;
			calibrationData[aangle].a+=a;
			calibrationData[aangle].b+=b;
			calibrationSamplesToGo--;
		}
	}

	prevIncr=incr;
}

void SinCosEncoder::useCalibrationData()
{
	static bool calculated=false;
	static float fadeInFactor=0;//from 0 to 1.0

	if(calculated==false)
	{
		//calc averages of samples
		int maxa=-99999, maxb=-99999, minb=99999, mina=99999;
		int i;
		for(i=0;i<calibrationAngles;i++)
		{
			int avga,avgb;
			avga=calibrationData[i].a/calibrationSamples;
			avgb=calibrationData[i].b/calibrationSamples;

			//find mins and maxs
			if(avga>maxa)
				maxa=avga;
			if(avgb>maxb)
				maxb=avgb;
			if(avga<mina)
				mina=avga;
			if(avgb<minb)
				minb=avgb;
		}

		//calc gains and offsets
		target_offset_a=-float(mina+maxa)/2.0/defaultGainCoeff;
		target_offset_b=-float(minb+maxb)/2.0/defaultGainCoeff;
		target_gain_a=defaultGainCoeff/(float(maxa-mina)/2.0/defaultGainCoeff);
		target_gain_b=defaultGainCoeff/(float(maxb-minb)/2.0/defaultGainCoeff);

		/*parentSys->setDebugParam(1,target_offset_a*1000);
		parentSys->setDebugParam(2,target_offset_b*1000);
		parentSys->setDebugParam(3,target_gain_a);
		parentSys->setDebugParam(4,target_gain_b);*/
		calculated=true;
	}
	else//fade in the resulted gains & offets to avoid jump
	{
		fadeInFactor+=0.001;//fades in 1000 cycles
		float fadeInFactorInv=1.0-fadeInFactor;
		gain_a=defaultGainCoeff*fadeInFactorInv + target_gain_a*fadeInFactor;
		gain_b=defaultGainCoeff*fadeInFactorInv + target_gain_b*fadeInFactor;
		offset_a=target_offset_a*fadeInFactor;
		offset_b=target_offset_b*fadeInFactor;

		if(fadeInFactor>=1.0)
			calibrationComplete=true;
	}
}



OPTIMIZE_FUNCTION void SinCosEncoder::update()
{
	if(initialized)
	{
		analogAngle=calcAngle();

		u16 adder=((interpolationFactor*(u32)analogAngle)>>16);
		u16 base=incr+incrAdder;

		//correct clitches
		if((base&3)==3 && analogAngle<16384)
			base++;
		else if((base&3)==0 && analogAngle>3*16384)
			base--;

		interpolatedPos=(base/4)*interpolationFactor + adder;
	}
	else
		interpolatedPos=(incr+incrAdder)*interpolationFactor/4+resetAdder;

	//parentSys->setDebugParam(1,interpolatedPos);
	//parentSys->setDebugParam(2,alt);
	//parentSys->setDebugParam(3,s32(interpolatedPos-prevInterpolatedPos));
	//parentSys->scopeCapture.captureSample(ScopeCapture::Channel::DEBUG1,(base/4)*4);
	//parentSys->scopeCapture.captureSample(ScopeCapture::Channel::DEBUG2,adder);
}

