/*
 * globals.cpp
 *
 *  Created on: Dec 10, 2011
 *      Author: tero
 */

#include "globals.h"

xTaskHandle GCCommTaskHandle;
xTaskHandle ledBlinkTaskHandle;
xTaskHandle GCPowerSupplyTaskHandle;
xTaskHandle SimpleMotionTaskHandle;
xTaskHandle SimpleMotionBufferedTaskHandle;
xTaskHandle SystemInitTaskHandle;
xTaskHandle SystemPeriodicTaskHandle;
xTaskHandle UpdateGPIOandBrakeTaskHandle;
xTaskHandle SlowTaskHandle;


//syncing mutex from GC RX complete interrupt to MC communication task:
xSemaphoreHandle MCCommTaskSemaphore = NULL;
//signal for running system init task:
xSemaphoreHandle SystemInitLauchSemaphore= NULL;
//semaphore given at mc comm task and taken at SystemPeriodicTask:
xSemaphoreHandle SystemPeriodicTaskSemaphore= NULL;


System sys;


void initGlobals()
{
    vSemaphoreCreateBinary( MCCommTaskSemaphore );
    if(MCCommTaskSemaphore==NULL)
    {
    	sys.setFault(FLT_FIRMWARE|FLT_ALLOC,70200);
    }

    vSemaphoreCreateBinary( SystemInitLauchSemaphore );
    if(SystemInitLauchSemaphore==NULL)
    {
    	sys.setFault(FLT_FIRMWARE|FLT_ALLOC,70201);
    }
    //take semaphore so its initially blocking the sys init task
    xSemaphoreTake(SystemInitLauchSemaphore,0);


    vSemaphoreCreateBinary( SystemPeriodicTaskSemaphore );
    if(SystemPeriodicTaskSemaphore==NULL)
    {
    	sys.setFault(FLT_FIRMWARE|FLT_ALLOC,70202);
    }
    //take semaphore so its initially blocking the sys init task
    xSemaphoreTake(SystemPeriodicTaskSemaphore,0);
}

