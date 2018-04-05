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

void faultManagement(FmState *fmState, float *reactionWheelSpeedHistory, float *timeStampHistory,
 	float *commandedTorque, uint16_t dataLength, float MOI);

void manageNewFaultDetected(FmState *fmState);

uint8_t faultCheckRW(FmState *fmState, float *frictionTorque, float *commandedTorque, uint16_t length);

uint8_t checkThreshold(float *data_x, float *data_y, uint16_t length, float threshold);

void manageFaultAlreadyDetected(FmState *fmState);

unsigned char faultCheck();

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t. then multiplies by Moment of intertia (MOI)
// to get the response torque on the reaction wheel
void getResponseTorque(float *omega, float *t, float *responseTorque, float *commandedTorque, 
	float *frictionTorque, uint16_t length, float MOI);

unsigned char recovery(FmState *fmState);

#ifdef __cplusplus
}
#endif
#endif
