
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

class AnalogIn {
public:
	AnalogIn();
	~AnalogIn();

	enum Channel {AnaIn1=0, AnaIn2=1, EncB=2, EncA=3};



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

private:
	//volatile u16 ADCsamples[ADC_CHANS*ADC_OVERSAMPLING];
	u16 *ADCDMASampleBuffer;//DMA buffer
	u16 ADCsamples[ADC_CHANS];

};

#endif /* ANALOGIN_H_ */
