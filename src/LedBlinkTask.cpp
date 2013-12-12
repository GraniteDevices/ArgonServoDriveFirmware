/*
 * LedBlinkTask.cpp
 *
 *  Created on: Jun 28, 2012
 *      Author: tero
 */

#include "LedBlinkTask.h"
#include "globals.h"
#include "utils.h"

/*TODO
  * add pause between color change
 */

#if oldblink
void LedBlinkTask( void *pvParameters )
{
	s32 LED1Blinks = 0, LED2Blinks = 0; //0=off 1-99=n blinks 100=always on
	int LED1BlinkTimer = 0, LED1BlinksBuffered = 100;
	int LED2BlinkTimer = 0, LED2BlinksBuffered = 100;
	int paused = 0;
	int blinkTimerPeriod;
	//, pauseLength;


	for( ;; )
	{
		//decide how fast to blink leds, use slower blinks if number of blinks is large to make them easier to count
		if(LED1Blinks>4 || LED2Blinks>4 )
		{
			blinkTimerPeriod=180000;
//			pauseLength=17; //pause time when switching blinks from led1 to led2
		}
		else
		{
			blinkTimerPeriod=95000;
//			pauseLength=13;//pause time when switching blinks from led1 to led2
		}

		if(pvParameters==NULL) //if freeRTOS task calls
			vTaskDelay( microsecsToTicks(blinkTimerPeriod) );
		else //if function called normally to stay in infinite loop (serious HW errors etc)
			Delay_100us(blinkTimerPeriod/100);

		if (LED1BlinkTimer < LED1BlinksBuffered * 5 && LED1BlinksBuffered != 100) //first blink led 1
		{
			if ((LED1BlinkTimer % 5) >= 1 && (LED1BlinkTimer % 5) <= 2)//generate about 2/5 on-duty cycle
				sys.physIO.doutLED1.setState( true );
			else
				sys.physIO.doutLED1.setState( false );
			LED1BlinkTimer++;
		}
		else if (LED2BlinkTimer < LED2BlinksBuffered * 5 && LED2BlinksBuffered != 100) //then led 2
		{

			if ((LED2BlinkTimer % 5) >= 1 && (LED2BlinkTimer % 5) <= 2)//generate about 2/5 on-duty cycle
				sys.physIO.doutLED2.setState( true );
			else
				sys.physIO.doutLED2.setState( false );
			LED2BlinkTimer++;
		}
		else //generate pause after blinks (switching blinkging to second led) to make then easily countable
		{
			paused++;

			//skip pause if no blinking previously occurred (constant on/offs only)
			if (paused > 13 /*|| (led2blinkcount==0 && led2blinkcount==0)*/)
			{
				//reset & sample new target
				LED1BlinkTimer = LED2BlinkTimer = paused = 0;
				LED1BlinksBuffered = LED1Blinks;
				LED2BlinksBuffered = LED2Blinks;

				//handle stationary led states
				if (LED1BlinksBuffered == 0)
					sys.physIO.doutLED1.setState( false );
				if (LED1BlinksBuffered == 100)
					sys.physIO.doutLED1.setState( true );
				if (LED2BlinksBuffered == 0)
					sys.physIO.doutLED2.setState( false );
				if (LED2BlinksBuffered == 100)
					sys.physIO.doutLED2.setState( true );
			}
		}

		/* shortest blink seqs led1 led2:
		 * 1 0
		 * 0 1 =bad to use symmetric patterns because confusion (swapped leds)
		 * 2 0
		 * 1 1
		 * 0 2 =bad to use symmetric patterns because confusion (swapped leds)
		 * 3 0
		 * 2 1
		 * 1 2 =bad to use symmetric patterns because confusion (swapped leds)
		 * 0 3 =bad to use symmetric patterns because confusion (swapped leds)
		 * 4 0
		 * 3 1
		 * 2 2
		 * 1 3 =bad to use symmetric patterns because confusion (swapped leds)
		 * 0 4 =bad to use symmetric patterns because confusion (swapped leds)
		 * 5 0
		 * 4 1
		 * 3 2
		 * 2 3 =bad to use symmetric patterns because confusion (swapped leds)
		 * 1 4 =bad to use symmetric patterns because confusion (swapped leds)
		 * 0 5 =bad to use symmetric patterns because confusion (swapped leds)
		 * 6 0
		 * 5 1
		 * 4 2
		 * 3 3
		 * 2 4 =bad to use symmetric patterns because confusion (swapped leds)
		 * 1 5 =bad to use symmetric patterns because confusion (swapped leds)
		 * 0 6 =bad to use symmetric patterns because confusion (swapped leds)
		 * etc
		 */

		/*
		 * decide what sequence to play, priorities:
		 * 1. stm32 side first fault (GC comm failure etc)
		 * 2. gc side first fault
		 * 3. gc status
		 */


		if (sys.getFaultBitsReg())
		{
			u32 faultBits = sys.getFirstFaultBitsReg();
			//priority: highest priority fault listed here first.
			//so if there are multiple fault bits on, then highest priority will be blinked
			if (faultBits & FLT_HARDWARE)
			{
				LED1Blinks = 7;
				LED2Blinks = 6;
			}
			else if (faultBits & FLT_PROGRAM_OR_MEM)
			{
				LED1Blinks = 7;
				LED2Blinks = 5;
			}
			//if no connecetion to GC succeed
			else if( (faultBits & FLT_GC_COMM) && ((sys.GCStatusBits&STAT_INITIALIZED) == 0) )
			{
				LED1Blinks = 7;
				LED2Blinks = 7;
			}
			//if GC connection fails during use
			else if (faultBits & FLT_GC_COMM)
			{
				LED1Blinks = 7;
				LED2Blinks = 4;
			}
			else if (faultBits & FLT_COMMUNICATION)
			{
				LED1Blinks = 3;
				LED2Blinks = 1;
			}
			else //unknown/other fault
			{
				LED1Blinks = 4;
				LED2Blinks = 2;
			}
		}
		else if (sys.GCFaultBits || (sys.GCStatusBits & STAT_FAULTSTOP))
		{
			//u32 faultBits = sys.GCFirstFault;
			u32 faultBits = sys.GCFaultBits;
			if (faultBits & FLT_HARDWARE)
			{
				LED1Blinks = 7;
				LED2Blinks = 3;
			}
			else if (faultBits & FLT_PROGRAM_OR_MEM)
			{
				LED1Blinks = 7;
				LED2Blinks = 2;
			}
			else if (faultBits & FLT_GC_COMM)//reporeted error from GC (crc error etc)
			{
				LED1Blinks = 7;
				LED2Blinks = 1;
			}
			else if (faultBits & FLT_INIT)
			{
				LED1Blinks = 4;
				LED2Blinks = 1;
			}
			else if (faultBits & FLT_OVERCURRENT)
			{
				LED1Blinks = 4;
				LED2Blinks = 0;
			}
			else if (faultBits & FLT_OVERTEMP)
			{
				LED1Blinks = 2;
				LED2Blinks = 1;
			}
			else if (faultBits & FLT_OVERVOLTAGE)
			{
				LED1Blinks = 3;
				LED2Blinks = 0;
			}
			else if (faultBits & FLT_FOLLOWERROR)
			{
				LED1Blinks = 2;
				LED2Blinks = 0;
			}
			else if (faultBits & FLT_UNDERVOLTAGE)
			{
				LED1Blinks = 1;
				LED2Blinks = 1;
			}
			else if (faultBits & FLT_MOTION)
			{
				LED1Blinks = 2;
				LED2Blinks = 2;
			}
			else if (faultBits & FLT_RANGE)
			{
				LED1Blinks = 5;
				LED2Blinks = 0;
			}
			else //unknown / other
			{
				LED1Blinks = 6;
				LED2Blinks = 0;
			}
		}
		else //GC status bits
		{
			u32 statusBits = sys.GCStatusBits;
			if (statusBits & STAT_PERMANENT_STOP)
			{
				LED1Blinks = 3;
				LED2Blinks = 2;
			}
			else if (statusBits & STAT_FERROR_RECOVERY)
			{
				LED1Blinks = 1;
				LED2Blinks = 100;//led 2 on only when driving motor
			}
			else if ((statusBits & STAT_INITIALIZED) == 0) //initializing, FIXME no proper error if fails
			{
				LED1Blinks = 1;
				LED2Blinks = 0;
			}
			else if (statusBits & STAT_HOMING)
			{
				LED1Blinks = 2;
				LED2Blinks = 100;//led 2 on only when driving motor
			}
			else if (statusBits & STAT_RUN)
			{
				LED1Blinks = 0;
				LED2Blinks = 100;//led 2 on only when driving motor
			}
			else
			{
				LED1Blinks = 3;
				LED2Blinks = 3;
			}
		}
	}
}
#else

/*typedef enum {None,Short,Long,On} LedState;
LedState orangeLed[5];
LedState greenLed[5];
int orangeLedSeqLength=0;
int greenSeqLength=0;*/

/*
 * sequence blinks
 * ' ' = led off
 * 'S' = short blink
 * 'L' = long blink
 * 'O' = constant on
 */
#define MAX_SEQ_LENGTH 6
const char *orangeLed="      ";
const char *greenLed="      ";


void setLedBlinkState()
{
	/*
	 * decide what sequence to play, priorities for fault led (orange):
	 *  1. stm32 side first fault (GC comm failure etc)
	 *  2. gc side first fault
	 * Priorities for status led (green)
	 *  1. gc status
	 */

	/* rules:
	 * -all sequences must have at least one long and one short blink to make the difference
	 * distinquishable
	 * -no blanks between blinks, all none states must be last
	 * -On state only for always on sequences
	 */

	/* possible sequences (star at end means its in use)
	 *
	 * CONSTANT ON * (only for green led)
	 *
	 * CONSTANT OFF *
	 *
	 * LS*
	 * SL
	 *
	 * LLS*
	 * LSL*
	 * LSS*
	 * SLL*
	 * SLS*
	 * SSL*
	 *
	 * SLLS*
	 * SLSL*
	 * SLSS*
	 * SSLL*
	 * SSLS*
	 * SSSL*
	 * LLLS*
	 * LLSL*
	 * LLSS*
	 * LSLL*
	 * LSLS*
	 * LSSL*
	 * LSSS*
	 * SLLL*
	 *
	 */

	if (sys.getFaultBitsReg())//local faults
	{
		u32 faultBits = sys.getFirstFaultBitsReg();
		//priority: highest priority fault listed here first.
		//so if there are multiple fault bits on, then highest priority will be blinked
		if (faultBits & FLT_HARDWARE)
		{
			orangeLed="SLLS  ";
		}
		else if (faultBits & FLT_PROGRAM_OR_MEM)
		{
			orangeLed="SLSL  ";
		}
		//if no connecetion to GC succeed
		else if( (faultBits & FLT_GC_COMM) && ((sys.GCStatusBits&STAT_INITIALIZED) == 0) )
		{
			orangeLed="SLSS  ";
		}
		//if GC connection fails during use
		else if (faultBits & FLT_GC_COMM)
		{
			orangeLed="SSLL  ";
		}
		else if (faultBits & FLT_COMMUNICATION)
		{
			orangeLed="LSSS  "; //FIXME one time this fault was displayed when GC comm broke (SLL blinks on drive pcb), on GCFW 1117 with full debug scope on
		}
		else//unknown/other fault
		{
			orangeLed="LLSL  ";
		}
	}
	else if (sys.GCFaultBits || (sys.GCStatusBits & STAT_FAULTSTOP))//GC faults
	{
		//u32 faultBits = sys.GCFirstFault;
		u32 faultBits = sys.GCFaultBits;
		if (faultBits & FLT_HARDWARE)
		{
			orangeLed="LLSS  ";
		}
		else if (faultBits & FLT_PROGRAM_OR_MEM)
		{
			orangeLed="LSLL  ";
		}
		else if (faultBits & FLT_GC_COMM)//reporeted error from GC (crc error etc)
		{
			orangeLed="LSL   ";
		}
		else if (faultBits & FLT_INIT)
		{
			orangeLed="LSS   ";
		}
		else if (faultBits & FLT_OVERCURRENT)
		{
			orangeLed="SLL   ";
		}
		else if (faultBits & FLT_OVERTEMP)
		{
			orangeLed="LSLS  ";
		}
		else if (faultBits & FLT_OVERVOLTAGE)
		{
			orangeLed="SLS   ";
		}
		else if (faultBits & FLT_FOLLOWERROR)
		{
			orangeLed="LS    ";
		}
		else if (faultBits & FLT_UNDERVOLTAGE)
		{
			orangeLed="SL    ";
		}
		else if (faultBits & FLT_MOTION)
		{
			orangeLed="SSL   ";
		}
		else if (faultBits & FLT_RANGE)
		{
			orangeLed="LSSL  ";
		}
		else //unknown / other
		{
			orangeLed="SSSL  ";
		}
	}
	else
	{
		orangeLed="NNNNNN";//no fault
	}

	//GC status bits
	{
		u32 statusBits = sys.GCStatusBits;
		if (statusBits & STAT_PERMANENT_STOP)
		{
			greenLed="LLS   ";
		}
		else if (statusBits & STAT_FAULTSTOP)
		{
			greenLed="      ";
		}
		else if (statusBits & STAT_FERROR_RECOVERY)
		{
			greenLed="LS    ";
		}
		else if ((statusBits & STAT_INITIALIZED) == 0) //initializing, FIXME no proper error if fails
		{
			greenLed="SL    ";
		}
		else if (statusBits & STAT_HOMING)
		{
			greenLed="LSS    ";
		}
		else if (statusBits & STAT_RUN)
		{
			greenLed="OOOOOO";
		}
		else
		{
			greenLed="LLSS  ";
		}
	}
}


bool getLedState( char state, int substep )
{
	switch(state)
	{
	case 'S':
//		if(substep<2)selkeempi vai ei?
		if(substep>6 && substep<9)
			return true;
		break;
	case 'L':
//		if(substep<8)selkeempi vai ei?
		if(substep>2 && substep<14)
			return true;
		break;
	case 'O':
		return true;
		break;
	}

	return false; //ie ' '
}

void LedBlinkTask( void *pvParameters )
{

	//flags that are true if null char received from string which means rest of seq should be off or "none"
	char orangeCurState,greenCurState;

	int blinkTimerPeriod;
	int step=0, substep=0;

	setLedBlinkState();

	for( ;; )
	{
		blinkTimerPeriod=35000;

		if(pvParameters==NULL) //if freeRTOS task calls this function
			vTaskDelay( microsecsToTicks(blinkTimerPeriod) );
		else //function called directly to stay in infinite loop (serious HW errors etc)
			Delay_100us(blinkTimerPeriod/100);

		orangeCurState=orangeLed[step];
		greenCurState=greenLed[step];
		sys.physIO.doutLED1.setState( getLedState(orangeCurState,substep) );
		sys.physIO.doutLED2.setState( getLedState(greenCurState,substep) );

		substep++;
		if(substep>16)
		{
			substep=0;
			step++;
			if(step>=MAX_SEQ_LENGTH)
			{
				step=0;
				setLedBlinkState();
			}
		}
	}
}
#endif

