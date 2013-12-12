/*
 * EncoderIn.h
 *
 *  Created on: Dec 1, 2011
 *      Author: tero
 */

#ifndef ENCODERIN_H_
#define ENCODERIN_H_

#include "types.h"

class EncoderIn
{
public:
	EncoderIn();
	virtual ~EncoderIn();
	u16 getCounter();
	/* counter value when index pulse was seen last time */
	u16 getCounterAtIndex();
	bool hasIndexUpdated();
	void update();
	s16 getVelocity();

private:
	u16 currCount, prevCount;
	s16 velocity;
};

#endif /* ENCODERIN_H_ */
