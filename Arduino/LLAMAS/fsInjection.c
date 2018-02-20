/*
 * fsInjection.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include <stdio.h>
#include <stdlib.h>
#include "fsInjection.h"

float fsInjection(int cmdToFaultSensor)
{
	/*Definition of Variables*/
	float theta_hat_rel,theta_rel,delta_theta_rel;	
	
	if (cmdToFaultSensor == 1)
	{
		delta_theta_rel = 10; /*TODO change to better bias*/
	}
	else
	{
		delta_theta_rel = 0; /*No bias if not commanded to fault*/;
	}
return delta_theta_rel;
}
