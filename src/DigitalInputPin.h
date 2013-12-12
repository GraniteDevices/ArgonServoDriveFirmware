/*
 * DigitalInputPin.h
 *
 *  Created on: Mar 25, 2012
 *      Author: tero
 */

#ifndef DIGITALINPUTPIN_H_
#define DIGITALINPUTPIN_H_

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "types.h"


class System;

class DigitalInputPin
{
public:
	enum InputPin { HallU,HallV,HallW,
		GPI2_EnablePosFeed,GPI3_EnableNegFeed,GPI4_Clearfaults,GPI1_HomeSwitch, GPI5_Enable,
		DIP1,DIP2,DIP3,DIP4 /*dip switch*/,
		ENCA,ENCB,ENCC,ENCD,ENCE,Ana1,Ana2,HSIN1,HSIN2
	};

	DigitalInputPin( InputPin inPin, System *parentSys );
	virtual ~DigitalInputPin();

	bool inputState();
	int inputStateInt(){
		bool state=inputState();
		if(state)
			return 1;
		else
			return 0;
	}
private:
	GPIO_TypeDef *port; //pointer to GPIOx
	u16 pin; //from 0 to n
	bool invert;
};


#endif /* DIGITALINPUTPIN_H_ */
