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


//in timer cycles @ 60MHz. 100k=600Hz
#define MAX_PWM_PERIOD_TIME 100000


class DigitalCounterInput
{
public:
	DigitalCounterInput();
	virtual ~DigitalCounterInput();

	enum CountMode { None, PulseTrain, PWM, Quadrature };

	//set counting direction of pulse input. called from direction pin changed ISR
	void setCountingDirection(bool up);
	void setCountMode( CountMode mode );
	s32 getCounter();
	void setCounter(s32 newvalue);


private:
	CountMode countMode;
};

#endif /* DIGITALCOUNTERINPUT_H_ */
