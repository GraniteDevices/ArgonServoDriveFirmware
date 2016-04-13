/*
 * DigitalInputPin.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: tero
 */

#include "DigitalInputPin.h"

#include "System.h"

DigitalInputPin::DigitalInputPin( InputPin inPin, System *parentSys )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	invert=false;
	bool pullup=true;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;


	switch (inPin)
	{
	case HallU:
		pin = GPIO_Pin_2;
		port = GPIOD;
		pullup=false;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		break;
	case HallV:
		pin = GPIO_Pin_12;
		port = GPIOC;
		pullup=false;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	case HallW:
		pin = GPIO_Pin_11;
		port = GPIOC;
		pullup=false;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	case GPI1_HomeSwitch:
		pin = GPIO_Pin_7;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		pullup=false;
		break;
	case GPI2_EnablePosFeed:
		pin = GPIO_Pin_6;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		pullup=false;
		break;
	case GPI3_EnableNegFeed:
		pin = GPIO_Pin_4;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		pullup=false;
		break;
	case GPI4_Clearfaults:
		pin = GPIO_Pin_5;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		pullup=false;
		break;
	case GPI5_Enable:
		if( parentSys->getHardwareID() == VSDR_049_HW_ID) //prototype 049
		{
			pin = GPIO_Pin_5;
			port = GPIOC;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		}
		else
		{
			pin = GPIO_Pin_3;
			port = GPIOB;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		}
		pullup=false;
		break;
	case DIP4://SELECTOR1 on schematics
		pin = GPIO_Pin_15;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		invert=true;
		break;
	case DIP3://SELECTOR2 on schematics
		pin = GPIO_Pin_13;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		invert=true;
		break;
	case DIP2://SELECTOR3 on schematics
		pin = GPIO_Pin_12;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		invert=true;
		break;
	case DIP1://SELECTOR4 on schematics
		pin = GPIO_Pin_2;
		port = GPIOB;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		invert=true;
		break;
	case ENCA:
		pin = GPIO_Pin_6;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		pullup=false;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//overwritten to AF by encoder in
		break;
	case ENCB:
		pin = GPIO_Pin_7;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		pullup=false;
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//overwritten to AF by encoder in
		break;
	case ENCC:
		pin = GPIO_Pin_8;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//overwritten to Mode_AF by encoder in
		pullup=false;
		break;
	case ENCD:
		pin = GPIO_Pin_9;
		port = GPIOC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		pullup=false;
		break;
	case ENCE:/*NOTE: this reads always 0 when CHE is set as output due to HW wiring*/
		pin = GPIO_Pin_15;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		pullup=false;
		break;
	case Ana1:
		/*
		 * TODO better use analog value and threshold instead of digital value when reading with inputState
		 * Anains are also routed to TIM1 inputs, use them to read digital values. Threshold voltage for 0->1 seems to be about 0V
		 * and 1->0 about -0.95V. 3.3V CMOS output wired between GND and AIN+ (- floating) generates swing of -1.35 to +1.35 so
		 * it barely works as proper source. Better connect - input to about +2 VDC though.
		 */
		pin = GPIO_Pin_12;
		port = GPIOA; //was originally GPIOC of anain, didnd't seem to work
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		pullup=false;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//AF so TIM can use this input still
		break;
	case Ana2: //TODO better use analog value and threshold instead of digital value when reading with inputState
		pin = GPIO_Pin_11;
		port = GPIOA;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		pullup=false;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//AF so TIM can use this input still
		break;
	case HSIN1:
		pin = GPIO_Pin_0;
		port = GPIOA;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//needed so TIM can read step pulse input. doesn't break GPI mode
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
	case HSIN2:
		pin = GPIO_Pin_1;
		port = GPIOA;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//needed so TIM can read pulse input. doesn't break GPI mode
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;

	default:
		break;
	}
	GPIO_InitStructure.GPIO_Pin = pin;
	if(pullup==true)
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( port, &GPIO_InitStructure );
}

DigitalInputPin::~DigitalInputPin()
{
}

bool DigitalInputPin::inputState()
{
	if( GPIO_ReadInputDataBit(port, pin) )
	{
		if(invert==false)
			return true;
		else
			return false;
	}
	else
	{
		if(invert==false)
			return false;
		else
			return true;
	}
}

