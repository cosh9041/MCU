/*
 * faultCheckFS.cpp
 *
 * Created: 1/30/2018 10:29:34 PM
 * Author : Pol Sieira
 */ 

//#include "sam.h"
#include "faultCheckFS.h"
#include "checkThreshold.h"

int faultCheckFS()
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
			return;
		}
	}
	return 0;
}