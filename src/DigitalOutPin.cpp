/*
 * DigitalOutPin.cpp
 *
 *  Created on: Jun 17, 2012
 *      Author: tero
 */

#include "DigitalOutPin.h"
#include "globals.h"

DigitalOutPin::DigitalOutPin( OutputPin outpin )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	invert=false;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;


	switch (outpin)
	{
	case CHD:
#ifdef VSDHV_033
		port = FIXME;
		pin = GPIO_Pin_9;
#else
		port = GPIOB;
		pin = GPIO_Pin_8;
#endif
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case CHDDir: //setting this on makes CHC output driver active
#ifdef VSDHV_033
		pin = GPIO_Pin_7;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
#else
		pin = GPIO_Pin_8;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#endif
		invert=false;
		break;
	case CHE:
		pin = GPIO_Pin_9;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case CHEDir: //setting this on makes CHD output driver active
		pin = GPIO_Pin_7;
		port = GPIOB;
		invert=false;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case GPO1:
		pin = GPIO_Pin_5;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
	case GPO2:
		pin = GPIO_Pin_4;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
	case GPO3:
		pin = GPIO_Pin_0;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case GPO4:
		pin = GPIO_Pin_1;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case LED1:
		pin = GPIO_Pin_13;
		port = GPIOC;
		invert=true;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	case LED2:
		pin = GPIO_Pin_14;
		port = GPIOC;
		invert=true;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	case MechBrakeRelease:
		pin = GPIO_Pin_15;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	case Debug1:
		pin = GPIO_Pin_11;// extu uart pin 3
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	case Debug2:
		pin = GPIO_Pin_10; //EXT uart pin 4
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;

	default:
		break;
	}
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( port, &GPIO_InitStructure );
	setState( false );
}

DigitalOutPin::~DigitalOutPin()
{
}

void DigitalOutPin::setState( int on )
{
	if(on!=0)
		setState(true);
	else
		setState(false);
}

bool DigitalOutPin::getState()
{
	return state;
}

void DigitalOutPin::setState( bool on )
{
	state=on;

	if(on)
		if(invert==false)
			GPIO_SetBits(port,pin);
		else
			GPIO_ResetBits(port,pin);
	else//turn output off
		if(invert==true)
			GPIO_SetBits(port,pin);
		else
			GPIO_ResetBits(port,pin);
}


