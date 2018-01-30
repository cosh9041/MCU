/*
 * faultManagement.cpp
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include "sam.h"
#include <math.h>
#include "faultManagement.h"
#include "faultCheckRW.h"
#include "faultCheckFS.h"
#include "recovery.h"

void faultManagement(){
	
	/*Definiton of Variables*/
	int isFaulted,isRecovering,cmdToRecover,faultType;
	
	if (isFaulted == 1)
	{
		if (cmdToRecover == 1)
		{
			cmdToRecover == 0;
			recovery(faultType);
			isFaulted == 0;
			isRecovering == 1;
			return
		}		
		else
		{
			return
		}
	}
	
	else
	{
		faultType faultCheckRW()
		faultType faultCheckFS()	
		if (faultType != 0)
		{
			if (isRecovering == 1)
			{
				return
			}
			else
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
		if (faultType == 0)
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