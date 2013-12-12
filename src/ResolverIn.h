/*
 * ResolverIn.h
 *
 *  Created on: Sep 15, 2012
 *      Author: tero
 */

#ifndef RESOLVERIN_H_
#define RESOLVERIN_H_

#include "types.h"

//if changed, must also udpate ISRs and RCC function
#define RESOLVER_TIMER TIM4
#define RESOLVER_TIMER_RCC RCC_APB1Periph_TIM4
#define RESOLVER_TIMER_IRQ TIM4_IRQn
#define RESOLVER_TIMER_IRQ_HANDLER TIM4_IRQHandler


class System;

class ResolverIn
{
public:
	virtual ~ResolverIn();
	ResolverIn(System *parent);

	/** starts or stops interrupt based resolver readout code. must be set true before getAngle() */
	void enableResolverRead(bool on);

	/** read actual resolver angle */
	u16 getAngle();

	/** read actual velocity */
	s16 getVelocity();

	/** add adc samples fo raveraging. up=true if value is taken at positive cycle and false if negative cycle */
	void addSamples(float x, float y, bool upcycle);

	void getSamples(float &x, float &y);
private:

	float xacc,yacc;
	int samples;

	float customAtan2(float y,float x);

	System *parentSys;
	const u16 countsPerRev;
	u16 lastAngle, currAngle;
	s32 outputCount, unrolledCount, lastOutputCount;

};

#endif /* RESOLVERIN_H_ */
