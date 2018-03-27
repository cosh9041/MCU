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


void faultManagement(int* isFaulted, int* isRecovering, int* faultType, 
		int* cmdToRecover, int* faultTimerActive);

void manageNewFaultDetected(int* isFaulted, int* faultType);

void manageFaultAlreadyDetected(int *isFaulted, int *cmdToRecover,
								int *isRecovering);

int checkThreshold();

int faultCheck();

int recovery(int faultType);

#ifdef __cplusplus
}
#endif
#endif
