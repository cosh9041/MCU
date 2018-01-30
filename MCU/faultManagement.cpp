/*
 * faultManagement.cpp
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include "sam.h"
#include "faultManagement.h"
#include "faultCheckRW.h"
#include "faultCheckFS.h"
#include "recovery.h"

void faultManagement(){
	
	/*Definition of Variables*/
	int isFaulted,isRecovering,cmdToRecover,faultType;
	/*Determine if the system is already faulting. If so, recover, if not continue fault checks shown below*/
	if (isFaulted == 1)
	{
		if (cmdToRecover == 1) /*if system is commanded to recover, begin the recovery sequence*/
		{
			cmdToRecover == 0;
			recovery(faultType);
			isFaulted == 0;
			isRecovering == 1;
			return
		}		
		else /*if not, let system fault and return*/
		{
			return
		}
	}
	
	else /*if not faulting, begin fault checks*/
	{
		faultType faultCheckRW()
		faultType faultCheckFS()	
		if (faultType != 0) /*if faulted, faultType = 1 for RW fault and faultType = 2 for FS fault*/
		{
			if (isRecovering == 1) /*if currently recovering, return and let system recover*/
			{
				return
			}
			else /*if not recovering yet, act dependent on what type of fault it is*/
			{
				isFaulted == 1;
				/*Alert GSU Function here*/
				if (faultType == 2)
				{
					return
				}
				else
				{
					/*Shut off power to RW*/
					return
				}
			}
		}	
		if (faultType == 0) /*if no fault, leave recovery and begin nominal operation and fault checks*/
		{	
			if (isRecovering == 1)
			{
				isRecovering == 0;
				return
			}	
			else
			{
				return
			}
		}	
	}
}