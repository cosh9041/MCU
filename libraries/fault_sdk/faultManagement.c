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

	uint8_t rwFaultDetected = faultCheckRW(fmState->faultType, angularAccel, commandedTorque, dataLength, MOI); 
	if (rwFaultDetected) 
		fmState->faultType = 2;
	//uint8_t fsFaultDetected = faultCheckFS();// TODO: Impl fault check fs

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
			fmState->activeRW = 0;
			break;
		case 1: break; // FS fault. In this case, we do nothing
		default: break;
	}
}

void manageFaultAlreadyDetected(FmState *fmState) {
	// fmState->cmdToRecover will be 1 when the system is commanded to initiate recovery
	// from the GSU. The setting of this bit is handled by the communication
	// handlers
	if (!fmState->cmdToRecover)
		return;

	fmState->cmdToRecover = 0;
	//recovery(fmState->faultType); TODO: re-impl Initiate recovery func
	fmState->isFaulted = 0;
	fmState->isRecovering = 1;
}

uint8_t checkThreshold()//float *stream1, float *stream2, float )
{
	return 0; /*TODO: Change to actual threshold checking method*/
}

uint8_t faultCheckRW(FmState *fmState, float *angularAccel, float *commandedTorque, uint16_t length, 
	float MOI) {
	uint8_t faultDetected, faultTimerActive;
	
	/*Run threshold check*/
	faultDetected = checkThreshold(); //TODO: Impl check threshold w/ data. Tune for run time perf

	handleFaultStatus(fmState, faultDetected);
}

void handleFaultStatus(FmState *fmState, uint8_t faultDetected) {
	if (!faultDetected) {
		fmState->faultTimerActive = 0;
		return;
	}

	if (!fmState->faultTimerActive) {
		fmState->faultTimerStart = millis();
		fmState->faultTimerActive = 1;
	}
	
	if ((millis() - fmState->faultTimerStart) > 30000) {
		fmState->faultTimerActive = 0;
	}
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