/*
 * ResolverIn.h
 *
 *  Created on: Sep 15, 2012
 *      Author: tero
 */

#ifndef FANUCSERIAL_H_
#define FANUCSERIAL_H_

#include "types.h"

//if changed, must also udpate ISRs and RCC function
#define RESOLVER_TIMER TIM4
#define RESOLVER_TIMER_RCC RCC_APB1Periph_TIM4
#define RESOLVER_TIMER_IRQ TIM4_IRQn
#define RESOLVER_TIMER_IRQ_HANDLER TIM4_IRQHandler


class System;

class FanucSerialEncoderIn
{
public:
	virtual ~FanucSerialEncoderIn();
	FanucSerialEncoderIn(System *parent);

	/** starts or stops readout code. must be set true before getAngle() */
	void enableRead(bool on);

	/** read actual resolver angle */
	u16 getAngle();

	/** read actual velocity */
	s16 getVelocity();
private:
	void customNOPdelay( unsigned long dly );
	System *parentSys;
	const u16 countsPerRev;
	u16 lastAngle, currAngle;
	s32 outputCount, unrolledCount, lastOutputCount;

};

#endif /* FANUCSERIAL_H_ */
