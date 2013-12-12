/*
 * Device.h
 *
 *  Created on: Mar 26, 2012
 *      Author: tero
 */

#ifndef DEVICE_H_
#define DEVICE_H_
#include "DigitalInputPin.h"
#include "AnalogIn.h"
#include "DigitalOutPin.h"

class System;
class PhysicalIO
{
public:
	PhysicalIO( System *parentSys );
	virtual ~PhysicalIO();

	//todo siirrä kaikki statukset, serial portit yms tänne. kaikki globaalit & staattinen melkeinpä
	/*void setStatus(u32 bits)
	{

	}*/

	/* sample all physical inputs. getter methods will return a value from update moment */

	void updatePhysInputs();
	//return true if pulsetrain counting direction should be up, false if down. no updatePhysInputs needed as this reads IO directly
	bool getDirectionPinState();

	s32 getAnalogInput1() const
	{
		return analogInput1;
	}

	s32 getAnalogInput2() const
	{
		return analogInput2;
	}

	s32 getAnalogInputEncA() const
	{
		return analogInputEncA;
	}

	s32 getAnalogInputEncB() const
	{
		return analogInputEncB;
	}

	/* returns integer where each bit repreasents a digital input current state.
	 *
	 * order of bits:
	 *bit 0:	updateDigInBit(dinDIP4);
	  bit 1: updateDigInBit(dinDIP3);
	etc.. updateDigInBit(dinDIP2);
	updateDigInBit(dinDIP1);
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
	updateDigInBit(dinGPI5);
	updateDigInBit(dinGPI4);
	updateDigInBit(dinGPI3);
	updateDigInBit(dinGPI2);
	updateDigInBit(dinGPI1);
	 * */
	u32 getDigitalInputs() const
	{
		return digitalInputs;
	}

	u32 getGPInputs() const
	{
		//for masking GPI1-5 from getDigitalInputs() value
		const static u32 GPIBitMask=0x1f;
		return digitalInputs&GPIBitMask;
	}

	//get dip switch address. DIP1=lsb DIP4=msb DIP5=terimination
	s32 getDIPSwitchAddress()
	{
		updatePhysInputs();
		int addr=((getDigitalInputs()>>16)&0xf);
		if(addr==0) addr=16;//DIP addr 0 reserved for BL mode, but were not in BL mode so make some other addr
		return(addr);
	}

	u8 getHallSensorState()
	{
		return((getDigitalInputs()>>13)&0x7);
	}


	/*u32 FaultBitsReg=0;
	u32 FirstFaultBitsReg=0;
	u32 FirstFaultLocation=0;
	u32 LastFaultLocation=0;
	u32 StatusBitsReg=0;*/

	DigitalOutPin doutCHD;
	DigitalOutPin doutCHDDir;
#ifndef VSDHV_033
	DigitalOutPin doutCHE;
	DigitalOutPin doutCHEDir;
#endif
	DigitalOutPin doutGPO1;
	DigitalOutPin doutGPO2;
	DigitalOutPin doutGPO3;
	DigitalOutPin doutGPO4;
	DigitalOutPin doutLED1;
	DigitalOutPin doutLED2;
	DigitalOutPin mechBrakeRelease;
	DigitalOutPin doutDebug1;
	DigitalOutPin doutDebug2;

	AnalogIn ADin;

	DigitalInputPin dinHSIN1;
	DigitalInputPin dinHSIN2;
	DigitalInputPin dinENCA;
	DigitalInputPin dinENCB;
	DigitalInputPin dinENCC;
	DigitalInputPin dinENCD;
	DigitalInputPin dinENCE;
	DigitalInputPin dinAna1;
	DigitalInputPin dinAna2;
	DigitalInputPin dinDIP1;
	DigitalInputPin dinDIP2;
	DigitalInputPin dinDIP3;
	DigitalInputPin dinDIP4;
	DigitalInputPin dinHallU;
	DigitalInputPin dinHallV;
	DigitalInputPin dinHallW;
	DigitalInputPin dinGPI1_HomeSwitch;
	DigitalInputPin dinGPI2_EnablePosFeed;
	DigitalInputPin dinGPI3_EnableNegFeed;
	DigitalInputPin dinGPI4_ClearFaults;
	DigitalInputPin dinGPI5_Enable;


private:

	u32 digitalInputs;
	s32 digitalOutputs;
	s32 analogInput1;
	s32 analogInput2;
	s32 analogInputEncA;
	s32 analogInputEncB;


	void updateDigInBit(DigitalInputPin &in);


};

#endif /* DEVICE_H_ */
