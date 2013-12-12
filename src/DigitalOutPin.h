/*
 * DigitalOutPin.h
 *
 *  Created on: Jun 17, 2012
 *      Author: tero
 */

#ifndef DIGITALOUTPIN_H_
#define DIGITALOUTPIN_H_

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "types.h"

class DigitalOutPin
{
public:
	enum OutputPin { CHD, CHDDir, CHE, CHEDir, GPO1, GPO2, GPO3, GPO4, LED1, LED2, Debug1, Debug2, MechBrakeRelease };

	DigitalOutPin( OutputPin outpin );
	virtual ~DigitalOutPin();

	//set on true and clear on false
	void setState(bool on);
	//set on non-zero and clear on zero
	void setState( int on );

	bool getState();
private:
	GPIO_TypeDef *port; //pointer to GPIOx
	u16 pin; //from 0 to n
	bool invert;
	bool state;
};

#endif /* DIGITALOUTPIN_H_ */
