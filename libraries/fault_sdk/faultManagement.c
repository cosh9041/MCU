/*
 * faultManagement.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fm_util.h>
#include "faultManagement.h"

void faultManagement(FmState *fmState, float *angularAccel, float *commandedTorque,
		uint16_t dataLength, float MOI) {
	if (fmState->isFaulted) {
		manageFaultAlreadyDetected(fmState);
		return;
	}

	// //faultCheckRW(fmState->faultType, ); //TODO: Impl fault check rw
	// //faultCheckFS(); TODO: Impl fault check fs

	/* fmState->faultType will be 0 if there is no fault, 1 if fs fault, 2 if RW*/
	if (!fmState->faultType) {
		fmState->isRecovering = 0; 
		return; 
	}

	if (fmState->isRecovering) return;
	manageNewFaultDetected(fmState);
}

void manageNewFaultDetected(FmState *fmState) {
	fmState->isFaulted = 1;
	/*TODO: Function to Alert GSU Function here*/
	//alertGSU(fmState->faultType);
	switch(fmState->faultType) {
		case 2:  // RW fault. turn off command to RW 1 to allow for visible deterioration of control
			fmState->isPrimaryRWActive = 0;
			break;
		case 1: break; // FS fault. In this case, we do nothing
		default: break;
	}
}

void manageFaultAlreadyDetected(FmState *fmState) {
	// fmState->cmdToRecover will be 1 when the system is commanded to initiate recovery
	// from the GSU. The setting of this bit is handled by the communication
	// handlers
	if (fmState->cmdToRecover)
		return;

	fmState->cmdToRecover = 0;
	//recovery(fmState->faultType); TODO: re-impl Initiate recovery func
	fmState->isFaulted = 0;
	fmState->isRecovering = 1;
}

uint8_t checkThreshold()
{
	return 0; /*TODO: Change to actual threshold checking method*/
}


faultCheckRW(FmState *fmState) {
	uint8_t faultDetected, faultTimerActive;
	
	/*Run threshold check*/
	faultDetected = checkThreshold(); //TODO: Impl check threshold w/ data. Tune for run time perf
	
	if (faultDetected) {
		if (faultTimerActive) {
			/*TODO: if (time < 30)
			{
				return;
			}
			else
			{
				timer_delete();
				faultDetected = 1;
			}
			*/
		}
		else {
			/*TODO: timer_create*/
		}
	} 
	else {
		if (faultTimerActive == 1)
		{
			/*TODO: timer_delete*/	
		}
		else
		{
			return 0;
		}
	}
	return 0;
}

uint8_t recovery(FmState *fmState)
{
	if (fmState->faultType == 0)
	{
		return 0;
	} 
	if (fmState->faultType == 1)
	{
		return 1;
	}
	if (fmState->faultType == 2)
	{
		return 2;
	}
	else
	{
		return 10;
	}
}