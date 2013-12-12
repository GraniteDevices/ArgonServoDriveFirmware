/*
 * DSCPowerTask.h
 *
 *  Created on: Dec 14, 2011
 *      Author: tero
 */

#ifndef DSCPOWERTASK_H_
#define DSCPOWERTASK_H_

#include "globals.h"

void GCPSUSetState(bool enabled);
void GCPSUTask( void *pvParameters );
void GCPSUInit();

#endif /* DSCPOWERTASK_H_ */
