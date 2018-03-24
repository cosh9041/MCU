/*
 * faultManagement.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 


#include <stdio.h>
#include <stdlib.h>
#include "faultManagement.h"

void faultManagement(int* cmdToRecover, int* isRecovering, int* faultType, 
		int* isFaulted, int*cker faultTimerActive){

	// *isFaulted is 1 when the system is already in a faulted state
	if (*isFaulted == 1) {
		// *cmdToRecover will be 1 when the system is commanded to initiate recovery 
		// from the GSU. The setting of this bit is handled by the communication 
		// handlers
		if (*cmdToRecover == 1) {
			// When commanded to recover, we want to initiate recovery
			*cmdToRecover = 0;
			recovery(*faultType);
			*isFaulted = 0;
			*isRecovering = 1;
		}		
		// If we're not commanded to recover, we let the system keep faulting
		return;
	}
	
	faultType = faultCheck();

	if (faultType != 0) {
		if (isRecovering == 1) /*if currently recovering, return and let system recover*/
		{
			return;
		}
		else /*if not recovering yet, act dependent on what type of fault it is*/
		{
			isFaulted = 1;
			/*TODO: Function to Alert GSU Function here*/
			if (faultType == 2)
			{
				return;
			}
			else
			{
				return;
			}
		}
	}	
	if (faultType == 0) /*if no fault, leave recovery and begin nominal operation and fault checks*/
	{	
		if (isRecovering == 1)
		{
			isRecovering = 0;
			return;
		}	
		else
		{
			return;
		}
	}	
}

int checkThreshold()
{
	return 0; /*TODO: Change to actual threshold checking method*/
}

int faultCheck()
{
	/*Defining the Variables*/
	int faultDetected, faultTimerActive;
	
	/*Run threshold check*/
	faultDetected = checkThreshold();
	
	/*Is a fault detected?*/
	if (faultDetected == 1)
	{
		if (faultTimerActive == 1)
		{
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
		else
		{
			/*TODO: timer_create*/
		}
	}
	else
	{
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
	{

int recovery(int faultType)
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


