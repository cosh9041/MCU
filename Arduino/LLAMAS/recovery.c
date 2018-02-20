/*
 * recovery.c
 *
 * Created: 1/29/2018 3:16:34 PM
 * Author : Pol Sieira
 */ 

#include <stdio.h>
//#include <stdlib.h>
#include "recovery.h"

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
