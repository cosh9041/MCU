#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "Arduino.h"
#include <math.h>
#include <fm_util.h>
#include "faultManagement.h"

void faultManagement(FmState *fmState, double *rwSpeedHist, unsigned long *timeStampHist, uint16_t rwDataLength, 
		double *commandedTorque, double *fineDelTheta, double *coarseDelTheta, uint16_t sensorDataLength, double MOI) {
	if (fmState->isFaulted) {
		manageFaultAlreadyDetected(fmState);
		return;
	}

	double responseTorque[rwDataLength];
	double frictionTorque[rwDataLength];
	getResponseTorque(rwSpeedHist, timeStampHist, responseTorque, commandedTorque, frictionTorque,
		rwDataLength, MOI);

  	uint8_t rwFaultDetected = 0;
	// uint8_t rwFaultDetected = faultCheckRW(fmState, frictionTorque, commandedTorque, rwSpeedHist, rwDataLength); 
	// if (rwFaultDetected) 
	// 	if (fmState->isRecovering) return;
	// 	fmState->faultType = 2;
	// }
	uint8_t fsFaultDetected = faultCheckFS(fmState, coarseDelTheta, fineDelTheta, sensorDataLength);
	if (fsFaultDetected) {
		if (fmState->isRecovering) return;
		fmState->faultType = 1;
	}

	/* fmState->faultType will be 0 if there is no fault, 1 if fs fault, 2 if RW*/
	if (!rwFaultDetected && !fsFaultDetected) {
		fmState->faultType = 0;
		fmState->isRecovering = 0; 
		return; 
	}

	manageNewFaultDetected(fmState);
}

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t, then multiply by
// moment of intertia to calculate response torque of system
void getResponseTorque(double *omega, unsigned long *t, double *responseTorque, double *commandedTorque, 
	  double *frictionTorque, uint16_t length, double MOI) {
//   double omegaDiff;
//   int tDiff;
//   double tDiffs;
//   double angularAccel;
//   for (int i = 0; i < length-1; i++) {
//     omegaDiff = omega[i] - omega[i+1];
// 	// NOTE: THIS LINE DOES NOT WORKK IN C I THINK
//     //tDiff = int(t[i] - t[i+1]);
//     tDiffs = tDiff * .000001;

//     if (tDiff != 0) {
//       angularAccel = omegaDiff/tDiffs;
//       responseTorque[i] = angularAccel * MOI;
//     } else {
//       responseTorque[i] = 0;
//     }
// 	  frictionTorque[i] = commandedTorque[i] - responseTorque[i];
//   }
}

void manageNewFaultDetected(FmState *fmState) {
	fmState->isFaulted = 1;
	fmState->faulting = 0;
	/*TODO: Function to Alert GSU Function here*/
	alertGSU(fmState);
	switch(fmState->faultType) {
		case 2:  // RW fault. turn off command to RW 1 to allow for visible deterioration of control
			fmState->activeRW = 0;
			break;
		case 1: 
      break; // FS fault. In this case, we do nothing
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
	if (fmState->faultType == 1) {
      fmState->activeFS = 2;
    }
	// if (fmState->faultType == 2) {
    //   fmState->activeRW = 2;
    // }
	fmState->isFaulted = 0;
	fmState->isRecovering = 1;
}

uint8_t checkThreshold(double *data_x, double *data_y, uint16_t length, double threshold) {
	double sum = 0;
  double diff = 0;
	for (int i = 0; i < length; i++) {
    diff = (data_y[i] - data_x[i]);
    // Serial.println(diff);
    diff = fabs(diff);
    // Serial.println(diff);
		sum += diff;
	}
	double meanDiff = sum/length;
  //Serial.println(meanDiff);
	if (meanDiff > threshold) return 1;
	return 0; 
}

uint8_t faultCheckRW(FmState *fmState, double *frictionTorque, double *commandedTorque, double *rwSpeed, uint16_t length) {
	double expectedFriction[length];
	for (int i = 0; i < length; i++) {
		expectedFriction[i] = fmState->p1*rwSpeed[i] + fmState->p2;
	}

	uint8_t faultDetected = checkThreshold(frictionTorque, expectedFriction, length-1, 3*(fmState->p2)); 
	return handleFaultStatus(fmState, faultDetected);
}

uint8_t faultCheckFS(FmState *fmState, double *coarseDelTheta, double *fineDelTheta, uint16_t length) {
	uint8_t faultDetected = checkThreshold(coarseDelTheta, fineDelTheta, length, 4*PI/180); //TODO: Impl check threshold w/ data. Tune for run time perf
	return handleFaultStatus(fmState, faultDetected);
}

uint8_t handleFaultStatus(FmState *fmState, uint8_t faultDetected) {
	if (!faultDetected) {
		fmState->faultTimerActive = 0;
		return 0;
	}

	if (!fmState->faultTimerActive && !fmState->isRecovering) {
		fmState->faultTimerStart = millis();
		fmState->faultTimerActive = 1;
		return 0;
	}
	
	if ((millis() - fmState->faultTimerStart) > fmState->timeToFault) {
		if (fmState->faultTimerActive) {
			fmState->faultTimerActive = 0;
			return 1;
		} else if (fmState->isRecovering) {
			return 1;
		}
	}
	return 0;
}

void alertGSU(FmState *fmState) {
	if (fmState->faultType == 1) {
		Serial.write(37);
	} 
}