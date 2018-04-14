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

void faultManagement(FmState *fmState, float *angularAccel, float *commandedTorque, double *fineDelTheta, double *coarseDelTheta,
		uint16_t dataLength, float MOI);

void manageNewFaultDetected(FmState *fmState);

void manageFaultAlreadyDetected(FmState *fmState);

unsigned char checkThreshold();

uint8_t faultCheckRW(FmState *fmState, float *angularAccel, float *commandedTorque, uint16_t length, float MOI);

uint8_t faultCheckFS(FmState *fmState, double *coarseDelTheta, double *fineDelTheta, uint16_t length);

unsigned char faultCheck();

unsigned char recovery(FmState *fmState);

#ifdef __cplusplus
}
#endif
#endif
