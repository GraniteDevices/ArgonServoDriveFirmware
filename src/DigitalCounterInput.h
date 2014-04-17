/*
 * DigitalCounterInput.h
 *
 *  Created on: Jun 7, 2012
 *      Author: tero
 *
 *  This class implements following reference input modes:
 *   - step/dir
 *   - PWM (todo)
 *   - Quadrature (todo)
 */

#ifndef DIGITALCOUNTERINPUT_H_
#define DIGITALCOUNTERINPUT_H_

#include "types.h"

/*
 * HW Pins:
 * HSIN1=PA0 & PB10
 * HSIN2=PA1 & PB11
 *
 * Pulsetrain:
 * PA0=step
 * PA1=dir
 */

#define PULSETRAIN_DIR_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOA
#define PULSETRAIN_DIR_EXTI_PIN_SOURCE       EXTI_PinSource1
#define PULSETRAIN_DIR_EXTI_LINE EXTI_Line1
#define PULSETRAIN_DIR_EXTI_IRQ EXTI1_IRQn


//TIM2 runs at 60MHz clock (ABP bus clock x2)
//following value sets minimum freq for valid PWM input in timer cycles @ 60MHz. 100k=600Hz min
#define MAX_PWM_PERIOD_TIME 100000
//PWM in 2 uses TIM1 with 16 bit counters, so can't use that high value. 24000=2.5kHz minimum pwm freq
#define MAX_PWM2_PERIOD_TIME 24000

class PWMInputComputing
{
public:
	PWMInputComputing(u32 maxPulselength);
	s32 computePWMInput( u32 period, u32 pulselength, u32 timerCounter );

private:
	bool noPWMsignal;
	u32 maxCounterValue;//for loss of pwm signal detection
	u32 prevPulselength, prevPeriod;
};


class DigitalCounterInput
{
public:
	DigitalCounterInput();
	virtual ~DigitalCounterInput();

	enum CountMode { None, PulseTrain, PWM, Quadrature };

	//set counting direction of pulse input. called from direction pin changed ISR
	void setCountingDirection(bool up);
	void setCountMode( CountMode mode );
	//read counter value (step/dir, quadrature, pwm) sourceNr defines which phyiscal input to sample.
	//default 0 but pwm has two inputs so it can be 0 or 1
	s32 getCounter( int sourceNr=0 );
	void setCounter(s32 newvalue);


private:
	CountMode countMode;

	bool noPWM1signal,noPWM2signal;

	PWMInputComputing PWMIn1, PWMIn2;

};


#endif /* DIGITALCOUNTERINPUT_H_ */
