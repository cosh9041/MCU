﻿/*
 * faultManagement.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 


#include <stdio.h>
#include <stdlib.h>
#include "faultManagement.h"

void faultManagement(unsigned char *isFaulted, unsigned char *isRecovering, unsigned char *faultType, 
		unsigned char *cmdToRecover, unsigned char *faultTimerActive, unsigned char *isPrimaryRWActive, unsigned char *isPrimaryFSActive) {
	if (*isFaulted) {
		manageFaultAlreadyDetected(isFaulted, cmdToRecover, isRecovering);
		return;
	}

	//faultCheckRW(); TODO: Impl fault check rw
	//faultCheckFS(); TODO: Impl fault check fs

	/* faultType will be 0 if there is no fault, 1 if fs fault, 2 if RW*/
	if (!(*faultType)) {
		*isRecovering = 0; 
		return; 
	}

	if (*isRecovering) return;
	manageNewFaultDetected(isFaulted, faultType, isPrimaryRWActive, isPrimaryFSActive);
}

void manageNewFaultDetected(unsigned char *isFaulted, unsigned char *faultType, unsigned char *isPrimaryRWActive, unsigned char *isPrimaryFSActive) {
	*isFaulted = 1;
	/*TODO: Function to Alert GSU Function here*/
	//alertGSU(faultType);
	switch(*faultType) {
		case 2:  // RW fault. turn off command to RW 1 to allow for visible deterioration of control
			*isPrimaryRWActive = 0;
			break;
		case 1: break; // FS fault. In this case, we do nothing
		default: break;
	}
}

void manageFaultAlreadyDetected(unsigned char *isFaulted, unsigned char *cmdToRecover,
								unsigned char *isRecovering) {
	// *cmdToRecover will be 1 when the system is commanded to initiate recovery
	// from the GSU. The setting of this bit is handled by the communication
	// handlers
	if (!(*cmdToRecover))
		return;

	*cmdToRecover = 0;
	//recovery(faultType); TODO: re-impl Initiate recovery func
	*isFaulted = 0;
	*isRecovering = 1;
}

unsigned char checkThreshold()
{
	return 0; /*TODO: Change to actual threshold checking method*/
}


unsigned char faultCheckRW(unsigned char *faultType) {
	unsigned char faultDetected, faultTimerActive;
	
	/*Run threshold check*/
	//faultDetected = checkThreshold(); //TODO: Impl check threshold w/ data. Tune for run time perf
	
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

unsigned char recovery(unsigned char faultType)
{
	if (faultType == 0)
	{
		return 0;
	} 
	if (faultType == 1)
	{
		return 1;
	}
	if (faultType == 2)
	{
		return 2;
	}
	else
	{
		return 10;
	}
}