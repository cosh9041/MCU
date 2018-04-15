/*
 * faultManagement.h
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 * 
*/

#ifndef FAULTMANAGEMENT_H_
#define FAULTMANAGEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <fm_util.h>
#include "Arduino.h"

void faultManagement(FmState *fmState, double *rwSpeedHist, double *timeStampHisty, uint16_t rwDataLength, 
		double *commandedTorque, double *fineDelTheta, double *coarseDelTheta, uint16_t sensorDataLength, double MOI);

void manageNewFaultDetected(FmState *fmState);

uint8_t faultCheckRW(FmState *fmState, double *frictionTorque, double *commandedTorque, double *rwSpeed, uint16_t length);

uint8_t checkThreshold(double *data_x, double *data_y, uint16_t length, double threshold);

void manageFaultAlreadyDetected(FmState *fmState);

uint8_t faultCheckFS(FmState *fmState, double *coarseDelTheta, double *fineDelTheta, uint16_t length);

uint8_t handleFaultStatus(FmState *fmState, uint8_t faultDetected);

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t. then multiplies by Moment of intertia (MOI)
// to get the response torque on the reaction wheel
void getResponseTorque(double *omega, double *t, double *responseTorque, double *commandedTorque, 
	double *frictionTorque, uint16_t length, double MOI);

#ifdef __cplusplus
}
#endif
#endif
