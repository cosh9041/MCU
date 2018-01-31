/*
 * fsInjection.cpp
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

//#include "sam.h"
#include "fsInjection.h"

void fsInjection(int cmdToFaultSensor)
{
	/*Definition of Variables*/
	float theta_hat_rel,theta_rel,delta_theta_rel;	
	if (cmdToFaultSensor == 1)
	{
		theta_hat_rel = theta_rel + delta_theta_rel;
	}
	else
	{
		theta_hat_rel = theta_rel;
	}
return;
}