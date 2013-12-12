/*
 * ProductionTester.h
 *
 *  Created on: Mar 26, 2013
 *      Author: tero
 *
 *  This class implements various production QC tests
 *  Requries special external hardware, not useful class for end applications
 */

#ifndef PRODUCTIONTESTER_H_
#define PRODUCTIONTESTER_H_

#include "globals.h"

class System;

class ProductionTester
{
public:
	ProductionTester( );
	virtual ~ProductionTester();

	void doTests();

	void testGPIO();
	void testFBIO();
	void testSMV2PortIO();

	//return true if STO is on
	bool getSTO1State();
	bool getSTO2State();

	//TODO test DIP I/O by setting it as output and reading value?

	//first is LSB
	//enum TestFaultBits{GPO1fast,GPO1slow,GPO2fast,GPO2slow,GPO3fast,GPO3slow,GPO4fast,GPO4slow};

	//max 16 items per enum to fit in register. written bit is 2*enum(+1 if fast mode test)
	enum TestFaultBits1{GPI1=0,GPI2=1,GPI3=2,GPI4=3,HSIN1=4,HSIN2=5,
		Ana1=6,Ana2=7,Ana1b=8,Ana2b=9,CHAAna=10,CHBAna=11,CHAAnab=12,CHBAnab=13};
	/*CHD & CHE will pass even without test plug because they're outputs. however they're verified by results of other inputs*/
	enum TestFaultBits2{CHA=0,CHB=1,CHC=2,CHD=3,CHE=4,HallU=5,HallV=6,
		HallW=7, Ena=8,STO1a=9,STO1b=10,STO2a=11,STO2b=12};

private:

	void  isAnalogInWithinRange( AnalogIn::Channel channel, float minV, float maxV, TestFaultBits1 faultTarget, bool alternativeTestAKAFast );
	void isGPIInExpectedState( bool gpi1, bool gpi2, bool gpi3, bool gpi4, bool hsin1, bool hsin2,bool fastresponsetest );
	void isFBInExpectedState( int cha, int chb, int chc, int chd,int che,
			int hallu,int hallv, int hallw,bool fastresponsetest);
	void setFault1(TestFaultBits1 fault, bool fastresponsetest)
	{
		u32 flt=2*fault;
		if(fastresponsetest==true)
			flt+=1;
		faultBits1|=1<<u32(flt);
		faultCount1++;
	}

	void setFault2(TestFaultBits2 fault, bool fastresponsetest)
	{
		u32 flt=2*fault;
		if(fastresponsetest==true)
			flt+=1;
		faultBits2|=1<<u32(flt);
		faultCount2++;
	}

	u32 faultBits1;
	u32 faultCount1;
	u32 faultBits2;
	u32 faultCount2;
	float faultVoltage;

};

#endif /* PRODUCTIONTESTER_H_ */
