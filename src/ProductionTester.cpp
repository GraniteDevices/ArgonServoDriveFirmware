/*
 * ProductionTester.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: tero
 *
 *  This class implements various production QC tests
 *  Requries special external hardware, not useful class for end applications
 */



#include "ProductionTester.h"
#include "utils.h"

/* FAULTLOCATION_BASE is used in setFault as starting value of fault location to identify where fault occurred
 * Format XXYYZZ XX=file specific number YY=function specific number ZZ=line specific number
 */
#define FAULTLOCATION_BASE 870000

ProductionTester::ProductionTester():faultBits1(0),faultCount1(0),faultBits2(0),faultCount2(0),faultVoltage(0.1234)
{

}

ProductionTester::~ProductionTester()
{
}

void ProductionTester::isAnalogInWithinRange( AnalogIn::Channel channel, float minV, float maxV, TestFaultBits1 faultTarget, bool alternativeTestAKAFast )
{
	float in=sys.physIO.ADin.getVoltageVolts(channel);
	if(in>minV && in<maxV)
	{
		return;
	}
	setFault1(faultTarget,alternativeTestAKAFast);
	faultVoltage=in;
}

void ProductionTester::isGPIInExpectedState( bool gpi1, bool gpi2, bool gpi3, bool gpi4, bool hsin1,
		bool hsin2, bool fastresponsetest )
{
	bool pgpi1,pgpi2,pgpi3,pgpi4,phsin1,phsin2;

	sys.physIO.updatePhysInputs();

	//read input states
	pgpi1=sys.physIO.dinGPI1_HomeSwitch.inputState();
	pgpi2=sys.physIO.dinGPI2_EnablePosFeed.inputState();
	pgpi3=sys.physIO.dinGPI3_EnableNegFeed.inputState();
	pgpi4=sys.physIO.dinGPI4_ClearFaults.inputState();
	phsin1=sys.physIO.dinHSIN1.inputState();
	phsin2=sys.physIO.dinHSIN2.inputState();

	//compare to expected value and set fault if differ
	if(pgpi1!=gpi1 )
		setFault1(GPI1,fastresponsetest);
	if(pgpi2!=gpi2 )
		setFault1(GPI2,fastresponsetest);
	if(pgpi3!=gpi3 )
		setFault1(GPI3,fastresponsetest);
	if(pgpi4!=gpi4 )
		setFault1(GPI4,fastresponsetest);
	if(phsin1!=hsin1 )
		setFault1(HSIN1,fastresponsetest);
	if(phsin2!=hsin2 )
		setFault1(HSIN2,fastresponsetest);
}


void ProductionTester::testGPIO()
{
	//TODO verify that fast resp timings are "safe"
	int slowRespWait=10000;
	//FIXME problem with fast response is that this task runs low priority and timings are very inaccurate
	int fastRespWait=20;//apparently even at 0, the algos arent fast enough to catch GPI in old state, so add some safety value here

	const float anaTol=0.33;

	/*
	 * GPO delay past 50% voltage abt 2µs 0->1, 1µs 1->0 with load: 400ohm 5V
	 * GPI delay abt 10µs driven from this GPO
	 */

	//init states
	sys.physIO.doutGPO1.setState(0);
	sys.physIO.doutGPO2.setState(0);
	sys.physIO.doutGPO3.setState(0);
	sys.physIO.doutGPO4.setState(0);
	Delay_1us(slowRespWait);

	/*replaced pulldown res 470ohm -> 1200 ohm. analog values changed*/

	Delay_1us(fastRespWait);
	//toggle GPO1 and compare input responses
	sys.physIO.doutGPO1.setState(1);//change output state
	//V1006	Delay_1us(fastRespWait);
	//V1006	isGPIInExpectedState(true,false,false,false, true,true, true);//sample fast respones right after output pin change
	Delay_1us(slowRespWait);//wait some time berore samplig slow responses
	isGPIInExpectedState(true,false,false,false, true,true, false);//sample slow respones, slow reacting inputs have now settled
	isAnalogInWithinRange(AnalogIn::AnaIn1, 0-anaTol, 0+anaTol, Ana1,false);//in sample1 1.52V
	isAnalogInWithinRange(AnalogIn::AnaIn2, 5-anaTol, 5+anaTol, Ana2,false);//in sample1 4.857, sample2 4.1x
	sys.physIO.doutGPO1.setState(0);//restore output to 0

	//toggle GPO2 and compare input responses
	sys.physIO.doutGPO2.setState(1);//change output state
	//V1006	Delay_1us(fastRespWait);
	//V1006	isGPIInExpectedState(false,true,false,false, true,true, true);//sample fast respones right after output pin change
	Delay_1us(slowRespWait);//wait some time berore samplig slow responses
	isGPIInExpectedState(false,true,false,false, true,true, false);//sample slow respones, slow reacting inputs have now settled
	isAnalogInWithinRange(AnalogIn::AnaIn1, 5-anaTol, 5+anaTol, Ana1,true);//4.91, s2=4.075 read 4.13 real
	isAnalogInWithinRange(AnalogIn::AnaIn2, 0-anaTol, 0+anaTol, Ana2,true);//0.84
	sys.physIO.doutGPO2.setState(0);//restore output to 0

	//toggle GPO3 and compare input responses
	sys.physIO.doutGPO3.setState(1);//change output state
	//V1006Delay_1us(fastRespWait);
	//V1006isGPIInExpectedState(false,false,true,false, true,false, true);//sample fast respones right after output pin change
	Delay_1us(slowRespWait);//wait some time berore samplig slow responses
	isGPIInExpectedState(false,false,true,false, true,false, false);//sample slow respones, slow reacting inputs have now settled
	isAnalogInWithinRange(AnalogIn::AnaIn1, 0-anaTol, 0+anaTol, Ana1b,false);//0.824
	isAnalogInWithinRange(AnalogIn::AnaIn2, -5-anaTol, -5+anaTol, Ana2b,false);//-4.085
	sys.physIO.doutGPO3.setState(0);//restore output to 0

	//toggle GPO4 and compare input responses
	sys.physIO.doutGPO4.setState(1);//change output state
	//V1006Delay_1us(fastRespWait);
	//V1006isGPIInExpectedState(false,false,false,true, false,true, true);//sample fast respones right after output pin change
	Delay_1us(slowRespWait);//wait some time berore samplig slow responses
	isGPIInExpectedState(false,false,false,true, false,true, false);//sample slow respones, slow reacting inputs have now settled
	isAnalogInWithinRange(AnalogIn::AnaIn1, -5-anaTol, -5+anaTol, Ana1b,true);//-4.014, s2 -3.35read -3.3real
	isAnalogInWithinRange(AnalogIn::AnaIn2, 0-anaTol, 0+anaTol, Ana2b,true);//0.842
	sys.physIO.doutGPO4.setState(0);//restore output to 0
}


void ProductionTester::isFBInExpectedState( int cha, int chb, int chc, int chd,int che,
		int hallu,int hallv, int hallw,
		bool fastresponsetest )
{
	int pa,pb,pc,pd,pe,hu,hv,hw;

	sys.physIO.updatePhysInputs();


	//read input states
	pa=sys.physIO.dinENCA.inputStateInt();
	pb=sys.physIO.dinENCB.inputStateInt();
	pc=sys.physIO.dinENCC.inputStateInt();
	pd=sys.physIO.dinENCD.inputStateInt();
	pe=sys.physIO.dinENCE.inputStateInt();
	hu=sys.physIO.dinHallU.inputStateInt();
	hv=sys.physIO.dinHallV.inputStateInt();
	hw=sys.physIO.dinHallW.inputStateInt();

	//compare to expected value and set fault if differ
	if(pa!=cha )
		setFault2(CHA,fastresponsetest);
	if(pb!=chb )
		setFault2(CHB,fastresponsetest);
	if(pc!=chc )
		setFault2(CHC,fastresponsetest);
	if(pd!=chd )
		setFault2(CHD,fastresponsetest);
	if(pe!=che )
		setFault2(CHE,fastresponsetest);
	if(hu!=hallu )
		setFault2(HallU,fastresponsetest);
	if(hv!=hallv )
		setFault2(HallV,fastresponsetest);
	if(hw!=hallw )
		setFault2(HallW,fastresponsetest);

}

void ProductionTester::testFBIO()
{
	int fastRespWait=2; //differential inputs toggled, halls not reacted yet
	//FIXME problem with fast response is that this task runs low priority and timings are very inaccurate
	int slowRespWait=2500;//halls reacted

	sys.physIO.doutCHDDir.setState(1);
	sys.physIO.doutCHEDir.setState(1);

	//test FB plug wiring:
	//D+ -> B-, C+
	//D- -> B+, C-
	//E+ -> A+, HallV
	//E- -> HallU, HallW, A-

	//set output states
	sys.physIO.doutCHD.setState(0);
	sys.physIO.doutCHE.setState(0);
	sys.physIO.doutCHD.setState(0);
	sys.physIO.doutCHE.setState(0);


	//check if inputs are ok
	Delay_1us(slowRespWait);
	isFBInExpectedState( 0, 1, 0, 0, 0,   1, 0, 1,   false );//check steady state

	//switch CHD=1
	sys.physIO.doutCHD.setState(1);
	Delay_1us(fastRespWait);//wait some time berore samplig slow responses
	isFBInExpectedState( 0, 0, 1, 1, 0,   1, 0, 1,   true );//state not changed yet
	Delay_1us(slowRespWait);
	isFBInExpectedState( 0, 0, 1, 1, 0,   1, 0, 1,   false );//all states settled now

	isAnalogInWithinRange(AnalogIn::EncA, -0.8, -0.7, CHAAna,false);
	isAnalogInWithinRange(AnalogIn::EncB, -0.8, -0.7, CHBAna,false);

	//switch CHE=1
	sys.physIO.doutCHE.setState(1);
	Delay_1us(fastRespWait);//wait some time berore samplig slow responses
	isFBInExpectedState( 1, 0, 1, 1, 0,   1, 0, 1,   true );//state not changed yet
	Delay_1us(slowRespWait);
	isFBInExpectedState( 1, 0, 1, 1, 0,   0, 1, 0,   false );//all states settled now

	isAnalogInWithinRange(AnalogIn::EncA, 0.7, 0.8, CHAAna,true);
	isAnalogInWithinRange(AnalogIn::EncB, -0.8, -0.7, CHBAna,true);

	//switch CHD=0
	sys.physIO.doutCHD.setState(0);
	Delay_1us(fastRespWait);//wait some time berore samplig slow responses
	isFBInExpectedState( 1, 1, 0, 0, 0,   0, 1, 0,   true );//state not changed yet
	Delay_1us(slowRespWait);
	isFBInExpectedState( 1, 1, 0, 0, 0,   0, 1, 0,   false );//all states settled now

	isAnalogInWithinRange(AnalogIn::EncA, 0.7, 0.8, CHAAnab,false);
	isAnalogInWithinRange(AnalogIn::EncB, 0.7, 0.8, CHBAnab,false);

	//switch CHE=0
	sys.physIO.doutCHE.setState(0);
	Delay_1us(fastRespWait);//wait some time berore samplig slow responses
	isFBInExpectedState( 0, 1, 0, 0, 0,   0, 1, 0,   true );//state not changed yet
	Delay_1us(slowRespWait);
	isFBInExpectedState( 0, 1, 0, 0, 0,   1, 0, 1,   false );//all states settled now


	sys.physIO.doutCHDDir.setState(0);
	sys.physIO.doutCHEDir.setState(0);

	NOP;
}

void ProductionTester::testSMV2PortIO()
{
	if(sys.physIO.mechBrakeRelease.getState()==true)
	{
		sys.physIO.mechBrakeRelease.setState(false);//set tester relays to init state
		Delay_1us(2500000);//this is not enough as STO1 reacts very slowly (PSU discharge)
	}
	else//just in case
	{
		sys.physIO.mechBrakeRelease.setState(false);//set tester relays to init state
		Delay_1us(100000);
	}

	if(sys.physIO.dinGPI5_Enable.inputState()==true)//check ena
		setFault2(Ena,false);
	if(getSTO1State()==false)//check sto1
		setFault2(STO1a,false);
	//getSTO2State();//WORKAROUND, don't know why it shows wrong result sometimes on first call. ISSUE51
	getSTO2State();
	if(getSTO2State()==false)//check sto2
		setFault2(STO2a,false);

	sys.physIO.mechBrakeRelease.setState(true);//switch tester relays

	Delay_1us(250000);//STO1 should be off after this time

	if(sys.physIO.dinGPI5_Enable.inputState()==true)//check that ena is not changed yet
		setFault2(Ena,false);
	if(getSTO1State()==true)//check sto1
		setFault2(STO1a,true);
	if(getSTO2State()==false)//check sto2
		setFault2(STO2a,true);

	Delay_1us(650000);

	if(sys.physIO.dinGPI5_Enable.inputState()==false)//ena should be changed by now due to tester 0.5 sec delay
		setFault2(Ena,true);
	if(getSTO1State()==true)//check sto1
		setFault2(STO1b,false);
	if(getSTO2State()==false)//check sto2
		setFault2(STO2b,false);

	Delay_1us(2000000);//more delay, so STO2 should be switched off


	if(sys.physIO.dinGPI5_Enable.inputState()==false)//ena should be changed by now due to tester 0.5 sec delay
		setFault2(Ena,true);
	if(getSTO1State()==true)//check sto1
		setFault2(STO1b,true);
	if(getSTO2State()==true)//check sto2
		setFault2(STO2b,true);

}

//return true if STO is on
bool ProductionTester::getSTO1State()
{
	int voltage;
	bool fail=false;
	voltage=sys.getParameter(SMP_ACTUAL_BUS_VOLTAGE,fail);

	if (fail)
		sys.setFault( FLT_GC_COMM, FAULTLOCATION_BASE+0201 );

	if(voltage<5000)//50VDC
		return true;
	else
		return false;
}


//return true if STO is on
bool ProductionTester::getSTO2State()
{
	//static int callcount=0;//callcount=6 jollon debugparam1=1140 aina
	bool fail=false;

	//callcount++;
	sys.setParameter( SMP_SYSTEM_CONTROL, SMP_SYSTEM_SAMPLE_TEST_VARIABLES );
	//tried delay here, no help to ISSUE51
	int STO2inactive = sys.getParameter( SMP_DEBUGPARAM1, fail );//returns nonzero if STO2 not active
	//kun luetaan DEBUGPARAM2 bugi näyttää välttyvän
	//int test2 = sys.getParameter( SMP_DEBUGPARAM2, fail );//returns nonzero if STO2 not active
	//int test3 = sys.getParameter( SMP_DEBUGPARAM3, fail );//returns nonzero if STO2 not active
	//tried read debug1 twice, no help to ISSUE51

	if (fail)
		sys.setFault( FLT_GC_COMM, FAULTLOCATION_BASE+0101 );

	if(STO2inactive==10)
		return false; //torque on
	else if(STO2inactive==11)
		return true; //torque off
	else
	{
		//reading test failed
		sys.setFault(FLT_GC_COMM,FAULTLOCATION_BASE+0102);
		return false;
	}
}


void ProductionTester::doTests()
{


	testGPIO();
	testFBIO();
	testSMV2PortIO();

	//sys.setDebugParam(4,);
	sys.setDebugParam(4,faultBits1);
	sys.setDebugParam(5,faultBits2);

	//lower 15 bits for fault count, upper 15 for faultVoltage
	sys.setDebugParam(6,faultCount1+faultCount2+11000  +  ((int(faultVoltage*1000.0)<<15)&0xffff8000) );
}



