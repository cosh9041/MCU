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


void faultManagement(unsigned char *isFaulted, unsigned char *isRecovering, unsigned char *faultType, 
		unsigned char *cmdToRecover, unsigned char *faultTimerActive, 
		unsigned char *isPrimaryRWActive, unsigned char *isPrimaryFSActive);

void manageNewFaultDetected(unsigned char *isFaulted, unsigned char *faultType, unsigned char *isPrimaryRWActive, 
	unsigned char *isPrimaryFSActive);

void manageFaultAlreadyDetected(unsigned char *isFaulted, unsigned char *cmdToRecover,
								unsigned char *isRecovering);

unsigned char checkThreshold();

unsigned char faultCheck();

unsigned char recovery(unsigned char faultType);

#ifdef __cplusplus
}
#endif
#endif
