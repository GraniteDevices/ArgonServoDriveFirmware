/*
 * SinCosEncoder.h
 *
 *  Created on: 23.4.2015
 *      Author: Tero
 */

#ifndef SINCOSENCODER_H_
#define SINCOSENCODER_H_

#include "types.h"
#include "utils.h"
#include <math.h>

class System;

class SinCosCalibrationData
{
public:
	SinCosCalibrationData():a(0),b(0),count(0) {}
	int a,b;
	s8 count;
};

class SinCosEncoder {
public:
	SinCosEncoder( System *parent );
	~SinCosEncoder();

	//rotate slowly and call this frequently to have sincos intialized. after init it provides interpolation
	void initialize();

	void setInterpolationFactor(int factor){
		interpolationFactor=4*factor;
	}

	//called after adc complete, these are used for calculation of angles
	void setInputs(float cha,float chb, u16 counter)
	{
		a=(cha+offset_a)*gain_a;
		b=(chb+offset_b)*gain_b;
		incr=counter;
	}
#if 0
	void setInputs(float cha,float chb)
	{
		a=(cha+offset_a)*gain_a;
		b=(chb+offset_b)*gain_b;
	}
	void setInputs(u16 counter)
	{
		incr=counter;
	}
#endif

	//called when encoder is setup, to avoid need re-init
	void encoderResetAt(u16 count);

	//call before getCounter and getVelocity
	void update();

	//interpolated count
	u16 getCounter()
	{
		return interpolatedPos+resetAdder;
	}

	//interpolated velocity
	s32 getVelocity(){return velocity;}

	bool hasIndexUpdated();
	u16 getCounterAtIndex();

	void updateVelocity()
	{
		velocity=s32(s16(interpolatedPos-prevInterpolatedPos));
		prevInterpolatedPos=interpolatedPos;
	}

	bool isFullyInitialized() { return calibrationComplete; }

	bool initialized;


private:
	void collectCalibrationData();
	void useCalibrationData();
	bool inline calibrationDataCollected(){return (calibrationSamplesToGo<=0); }


	System *const parentSys;

	u16 interpolatedPos, prevInterpolatedPos;

	float offset_a,offset_b,gain_a,gain_b;
	float target_offset_a,target_offset_b,target_gain_a,target_gain_b;//used for interpolation of gains to avoid jump
	float a,b;
	u16 analogAngle;//0 to 65536 is full rev
	u16 incr;
	u16 incrAdder;
	u16 resetAdder;//0 by default and some value after reset to get 0
	s32 velocity;
	int interpolationFactor;

	//both a and b must be above this voltage to have initialize position (digital reading is not in hysteresis)
	static constexpr float defaultGainCoeff=8192.0;
	static constexpr float minimumAmplForDigitalReadout=0.34*defaultGainCoeff;//0.34volts

	static constexpr int calibrationAngles=32;
	static constexpr int calibrationSamples=16;
	int calibrationSamplesToGo;
	SinCosCalibrationData calibrationData[calibrationAngles];
	bool calibrationComplete;

	inline u16 calcAngle()
	{
		//return fxpt_atan2(s16(a),s16(b));//cpu load 74.0%
		return u16(s16(atan2f(a,b)*1.0430378e4));//cpu load 73.0%, no int conversions 72.4%
		//return u16(s16(arctan2(a,b)*1.0430378e4));//cpu load 71.8% with int conversions but shows velocity ripple due to error
	}

};

#endif /* SINCOSENCODER_H_ */
