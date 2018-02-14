/*
 * rwInjection.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 


//#include "sam.h"
#include <stdio.h>
#include <stdlib.h>
#include "rwInjection.h"



float calcInducedFric(float omega)
{
	return 5; /*TODO: need to complete function later*/
}

float injectFault(int isPrimaryRWactive, int cmdToFaultRW, float tau_c, float omega)
{
	/*Definition of Variables*/
	float tau_hat_c, tau_hat_f;
	float delta_omega = 0.0001; /*TODO: Subject to change dependent on tolerances*/

	/*Check if we are command to fault and that command is given to the primary reaction wheel*/
	if (isPrimaryRWactive == 1 && cmdToFaultRW == 1)
	{
		if (omega < delta_omega && omega > -delta_omega)
		{
			tau_hat_c = 0; /* Determine if the motor is spinning too slowly and if it is set commanded torque to zero */
		}
		else 
		{
			tau_hat_f = calcInducedFric(omega); /*Calculate induced friction*/
			tau_hat_c = tau_c - tau_hat_f; /*Add induced friction into the commanded torque to appear as fault*/
		}
	}
	else
	{
		tau_hat_c = tau_c; /*If not commanded to fault keep commanded torque the same*/
	}
	return tau_hat_c;
}
