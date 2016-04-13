/*
 * globals.h
 *
 *  Created on: Dec 10, 2011
 *      Author: tero
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "types.h"
#include "SMCommandQueue.h"
#include "sm485.h"
#include "Device.h"
#include "System.h"

//freeRTOS task handles
extern xTaskHandle GCCommTaskHandle;
extern xTaskHandle ledBlinkTaskHandle;
extern xTaskHandle GCPowerSupplyTaskHandle;
extern xTaskHandle SimpleMotionTaskHandle;
extern xTaskHandle SimpleMotionBufferedTaskHandle;
extern xTaskHandle SystemInitTaskHandle;
extern xTaskHandle SystemPeriodicTaskHandle;
extern xTaskHandle UpdateGPIOandBrakeTaskHandle;
extern xTaskHandle SlowTaskHandle;
extern xTaskHandle EncoderOutTaskHandle;

//freeRTOS semaphore handles
extern xSemaphoreHandle MCCommTaskSemaphore;
extern xSemaphoreHandle SystemInitLauchSemaphore;
extern xSemaphoreHandle SystemPeriodicTaskSemaphore;

//main class
extern System sys;

//bitvise shift
#ifndef BV
#define BV(bit) (1L<<(bit))
#endif

//call before scheluder start
void initGlobals();


//hardware versions
//#define VSDHV_033


//affects ISR and System class
#define HIFREQ_TASK_FREQ 40000
#define HIFREQ_TASK_ISR_PRIORITY 1
#define HIFREQ_TASK_TIMER TIM4
#define HIFREQ_TASK_TIMER_RCC RCC_APB1Periph_TIM4
#define HIFREQ_TASK_TIMER_IRQ TIM4_IRQn
#define HIFREQ_TASK_TIMER_IRQ_HANDLER TIM4_IRQHandler

#endif /* GLOBALS_H_ */
