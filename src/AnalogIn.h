
/*
 * AnalogIn.h
 *
 *  Created on: Oct 29, 2011
 *      Author: tero
 */

#ifndef ANALOGIN_H_
#define ANALOGIN_H_

#include "types.h"

#define ADC_CHANS 4
#define ADC_OVERSAMPLING 8

//in ideal case offset of raw value (when input is 0V) is 16384 which is half way the raw value scale
//however about some typical offset is present in practical HW and it's compensated here
#define ADC_OFFSET_VALUE_ENC (16384+650)
#define ADC_OFFSET_VALUE_ANAIN (16384+47)

class AnalogIn {
public:
	AnalogIn();
	~AnalogIn();

	enum Channel {EncB=0, EncA=1,AnaIn1=2, AnaIn2=3};



	u16 getSample(Channel channel);
	s16 getVoltage(Channel channel); //scale=+16384=+10V -16384=-10V
	float getVoltageVolts(Channel channel);//in volts

	//trigger converesion to start. it takes 22µs to have valid data readable with getSample etc
	//this is called from general purpose high frequency ISR
	void startSampling();
	//called >20us later of startSamplig to store sapmple values to memory that may be later at any time read with getSample etc
	void storeSamples();

	//for analyzing/debugging adc noise
	void collectNoiseStatistics();

	//method to read encoder analog samples as soon as first two are converted (needed for sincos because quadature encoder must be sampled simultaneously)
	//should return fresh valus values ~2.2us after startSampling() (based on 30MHz freq and 28clock sampling time)
	void getFirstAnalogEncSample( float &cha, float &chb );

private:
	//volatile u16 ADCsamples[ADC_CHANS*ADC_OVERSAMPLING];
	u16 *ADCDMASampleBuffer;//DMA buffer
	u16 ADCsamples[ADC_CHANS];

};

#endif /* ANALOGIN_H_ */
