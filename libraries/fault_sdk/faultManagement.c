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

uint8_t faultCheckFS() {
	return 0;
}

void faultManagement(FmState *fmState, float *reactionWheelSpeedHistory, float *timeStampHistory,
 	float *commandedTorque, uint16_t dataLength, float MOI) {

	float responseTorque[dataLength];
	getResponseTorque(reactionWheelSpeedHistory, timeStampHistory, responseTorque,
		dataLength, MOI);

	if (fmState->isFaulted) {
		manageFaultAlreadyDetected(fmState);
		return;
	}

	uint8_t rwFaultDetected = faultCheckRW(fmState->faultType, responseTorque, commandedTorque, dataLength, MOI); 
	if (rwFaultDetected) {
		fmState->faultType = 2;
	}
	uint8_t fsFaultDetected = faultCheckFS();// TODO: Impl fault check fs
	if (fsFaultDetected) {
		fmState->faultType = 1;
	}

	/* fmState->faultType will be 0 if there is no fault, 1 if fs fault, 2 if RW*/
	if (!fmState->faultType) {
		fmState->isRecovering = 0; 
		return; 
	}

	if (fmState->isRecovering) return;
	manageNewFaultDetected(fmState);
}

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t, then multiply by
// moment of intertia to calculate response torque of system
void getResponseTorque(float *omega, float *t, float *responseTorque, uint16_t length, float MOI) {
  float omegaDiff;
  float tDiff;
  for (int i = 0; i < length-1; i++) {
    omegaDiff = omega[i+1] - omega[i];
    tDiff = t[i+1] - t[i];
    if (tDiff != 0)
      responseTorque[i] = omegaDiff / tDiff * MOI;
    else
      responseTorque[i] = 0;
  }
}

void manageNewFaultDetected(FmState *fmState) {
	fmState->isFaulted = 1;
	fmState->faulting = 0;
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

//uint8_t checkThreshold(float *data_x, float *data_y, sz_ma
uint8_t checkThreshold()
{
	return 0; /*TODO: Change to actual threshold checking method*/
}

uint8_t faultCheckRW(FmState *fmState, float *responseTorque, float *commandedTorque, uint16_t length, 
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