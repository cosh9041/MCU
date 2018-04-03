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

void faultManagement(FmState *fmState, float *angularAccel, float *commandedTorque,
		uint16_t dataLength, float MOI);

// void faultManagement(uint8_t *isFaulted, uint8_t *isRecovering, uint8_t *faultType, 
// 		uint8_t *cmdToRecover, uint8_t *faultTimerActive, uint8_t *isPrimaryRWActive, 
// 		uint8_t *isPrimaryFSActive, float *angularAccel, float *commandedTorque,
// 		uint16_t dataLength, float MOI);
void manageNewFaultDetected(FmState *fmState);

void manageFaultAlreadyDetected(FmState *fmState);

unsigned char checkThreshold();

unsigned char faultCheck();

unsigned char recovery(FmState *fmState);

#ifdef __cplusplus
}
#endif
#endif
