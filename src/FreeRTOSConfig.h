/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( ( unsigned long ) 120000000 )
#define configTICK_RATE_HZ			( ( portTickType ) 2500 )
#define configMAX_PRIORITIES		( ( unsigned portBASE_TYPE ) 10 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 32 )
//not use in heap_3.c #define configTOTAL_HEAP_SIZE		( ( size_t ) ( 25 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 16 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1
#define configQUEUE_REGISTRY_SIZE   32
#define configUSE_MUTEXES			1

//for swich hook
#define configUSE_APPLICATION_TASK_TAG 0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		0
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete				0
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_xTaskResumeFromISR		1


/*
 * stm32 settings that should work
 * kernel int prio 0xf0 =15
 * max syscall prio 0xb0
 * libr kernel prio 15
 *
 * bug in stm32 periph library sets int prio to 0 no matter what is input so reduce other prios to 0 to workaround
 */
#ifndef periphlibbug
	/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
	(lowest) to 0 (1?) (highest). */

//configKERNEL_INTERRUPT_PRIORITY must not never be 0 or FreeRTOS breaks!

/*
 * Proper priority value relations must be (not 100% tested but mostly experimented):
 *
 * configLIBRARY_KERNEL_INTERRUPT_PRIORITY*16 >= configKERNEL_INTERRUPT_PRIORITY >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 *
 * these values given in Cortex M3 priorities (lower number=higher prio)
 */

	#define configKERNEL_INTERRUPT_PRIORITY 		0xf0
	#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	0xb0 /* equivalent to 0xb0, or priority 11. */

	/* This is the value being used as per the ST library which permits 16
	priority values, 0 to 15.  This must correspond to the
	configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
	NVIC value of 255. */
	//15=alin prioriteetti, tata vakiota kaytetaan omassa koodissa mm serial.cpp
	#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	0xf

#else
	/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
	(lowest) to 0 (1?) (highest). */
	//#define configKERNEL_INTERRUPT_PRIORITY 		255
	#define configKERNEL_INTERRUPT_PRIORITY 		0xf0
	//#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /*191=0xbf*//* 176=equivalent to 0xb0, or priority 11. */
	#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	0xb0 /* equivalent to 0xb0, or priority 11. */

	/* This is the value being used as per the ST library which permits 16
	priority values, 0 to 15.  This must correspond to the
	configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
	NVIC value of 255. */
	//15=alin prioriteetti, tata vakiota kaytetaan omassa koodissa mm serial.cpp
	#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

#endif

//extern int task;
//#define traceTASK_SWITCHED_IN() task=( int ) pxCurrentTCB->pxTaskTag;

#endif /* FREERTOS_CONFIG_H */

