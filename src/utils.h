/*
 * utils.h
 *
 *  Created on: Dec 6, 2011
 *      Author: tero
 */

#ifndef UTILS_H_
#define UTILS_H_


#include "types.h"
#include "misc.h" //ST lib header

class System;
class DelayedConditional
{
public:
	//pointer to System struct (because of clock). delay in seconds
	DelayedConditional( System *sys, float delay, bool initialOutput );

	bool delayedTrue(bool inputstate );

	void setDelay(float delay);

private:
	u32 inputStateLastTransitionToTrueTime;//time value when state changed last time to true
	s32 delayUs;
	System *system;
	bool outputState;
};



#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOSConfig.h"

u8 CRC8CalcFromBuffer( u8 *buf, int len, int crcinit );
u16 calcCRC16(u8 data, u16 crc);
void configCPUBronwOutSetting();

void NVIC_Init_GD(NVIC_InitTypeDef* NVIC_InitStruct);

//some handy macros
//usage NOP;
#define NOP __asm__ __volatile__ ("nop")
//usage void OPTIMIZE_FUNCTION myfunc()
#define OPTIMIZE_FUNCTION __attribute__((optimize("O3")))

#define microsecsToTicks(us) ((us)*configTICK_RATE_HZ/1000000L)

//Delay about 100us per dly count. Warning: varies with compiler optimization settings & CPU speed
void Delay_100us( int dly );
//Delay about 1us per dly count. Warning: varies with compiler optimization settings & CPU speed
//Probably very inaccurate at low values such as <10us
void Delay_1us( int dly );


#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_ */
