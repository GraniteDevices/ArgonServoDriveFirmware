/*
 * Device.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: tero
 */

#include "Device.h"
#include "System.h"

PhysicalIO::PhysicalIO( System *parentSys ):
	//init member variables:
	doutCHD(DigitalOutPin::CHD),
	doutCHDDir(DigitalOutPin::CHDDir),
	#ifndef VSDHV_033
	doutCHE(DigitalOutPin::CHE),
	doutCHEDir(DigitalOutPin::CHEDir),
	#endif
	doutGPO1(DigitalOutPin::GPO1),
	doutGPO2(DigitalOutPin::GPO2),
	doutGPO3(DigitalOutPin::GPO3),
	doutGPO4(DigitalOutPin::GPO4),
	doutLED1(DigitalOutPin::LED1),
	doutLED2(DigitalOutPin::LED2),
	mechBrakeRelease(DigitalOutPin::MechBrakeRelease),
	doutDebug1(DigitalOutPin::Debug1),
	doutDebug2(DigitalOutPin::Debug2),
	doutDebug3(DigitalOutPin::Debug3),
	doutDebug4(DigitalOutPin::Debug4),

	digitalInputs(0),
	digitalOutputs(0),
	analogInput1(0),
	analogInput2(0),
	analogInputEncA(0),
	analogInputEncB(0),
	dinHSIN1(DigitalInputPin::HSIN1,parentSys),
	dinHSIN2(DigitalInputPin::HSIN2,parentSys),
	dinENCA(DigitalInputPin::ENCA,parentSys),
	dinENCB(DigitalInputPin::ENCB,parentSys),
	dinENCC(DigitalInputPin::ENCC,parentSys),
	dinENCD(DigitalInputPin::ENCD,parentSys),
	dinENCE(DigitalInputPin::ENCE,parentSys),
	dinAna1(DigitalInputPin::Ana1,parentSys),
	dinAna2(DigitalInputPin::Ana2,parentSys),
	dinDIP1(DigitalInputPin::DIP1,parentSys),
	dinDIP2(DigitalInputPin::DIP2,parentSys),
	dinDIP3(DigitalInputPin::DIP3,parentSys),
	dinDIP4(DigitalInputPin::DIP4,parentSys),
	dinHallU(DigitalInputPin::HallU,parentSys),
	dinHallV(DigitalInputPin::HallV,parentSys),
	dinHallW(DigitalInputPin::HallW,parentSys),
	dinGPI1_HomeSwitch(DigitalInputPin::GPI1_HomeSwitch,parentSys),
	dinGPI2_EnablePosFeed(DigitalInputPin::GPI2_EnablePosFeed,parentSys),
	dinGPI3_EnableNegFeed(DigitalInputPin::GPI3_EnableNegFeed,parentSys),
	dinGPI4_ClearFaults(DigitalInputPin::GPI4_Clearfaults,parentSys),
	dinGPI5_Enable(DigitalInputPin::GPI5_Enable,parentSys)
{

}

PhysicalIO::~PhysicalIO()
{
	// TODO Auto-generated destructor stub
}

bool PhysicalIO::getDirectionPinState()
{
	return dinHSIN2.inputState();
}

void PhysicalIO::updatePhysInputs()
{
	digitalInputs=0;//reset

	//last in the list becomes LSB, first MSB
	updateDigInBit(dinDIP1); //becomes MSB
	updateDigInBit(dinDIP2);
	updateDigInBit(dinDIP3);
	updateDigInBit(dinDIP4);
	updateDigInBit(dinHallW);
	updateDigInBit(dinHallV);
	updateDigInBit(dinHallU);
	updateDigInBit(dinENCD);
	updateDigInBit(dinENCC);
	updateDigInBit(dinENCB);
	updateDigInBit(dinENCA);
	updateDigInBit(dinAna2);
	updateDigInBit(dinAna1);
	updateDigInBit(dinHSIN2);
	updateDigInBit(dinHSIN1);
	updateDigInBit(dinGPI5_Enable);
	updateDigInBit(dinGPI4_ClearFaults);
	updateDigInBit(dinGPI3_EnableNegFeed);
	updateDigInBit(dinGPI2_EnablePosFeed);
	updateDigInBit(dinGPI1_HomeSwitch); //becomes LSB

	analogInput1=ADin.getVoltage(AnalogIn::AnaIn1);
	analogInput2=ADin.getVoltage(AnalogIn::AnaIn2);
	analogInputEncA=ADin.getVoltage(AnalogIn::EncA);
	analogInputEncB=ADin.getVoltage(AnalogIn::EncB);

}

void PhysicalIO::updateDigInBit( DigitalInputPin& in )
{
	digitalInputs<<=1;
	if(in.inputState())
		digitalInputs|=1;
}


