/*
 * recovery.cpp
 *
 * Created: 1/29/2018 3:16:34 PM
 * Author : Pol Sieira
 */ 

#include "sam.h"
#include "recovery.h"

void recovery(int faultType)
{
	if (faultType == 0)
	{
		throw;
	} 
	if (faultType == 1)
	{
		/*TODO: Turn on power to RW2*/
		/*TODO: Switch command to RW2*/
	}
	if (faultType == 2)
	{
		/*TODO: Switch sensor reading to FS2*/
	}
return
}