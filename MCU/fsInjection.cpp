/*
 * fsInjection.cpp
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include "sam.h"
#include "fsInjection.h""

float theta_hat_rel,theta_rel,delta_theta_rel;

void fsInjection(int cmdToFaultSensor)
{
	if (cmdToFaultSensor == 1)
	{
		theta_hat_rel = theta_rel + delta_theta_rel;
	}
	else
	{
		theta_hat_rel = theta_rel;
	}
return
}