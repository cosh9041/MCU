/*
 * rwInjection.cpp
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 


#include "sam.h"
#include <math.h>
#include "rwInjection.h""


float rwInjection(int isPrimaryRWactive, int cmdToFaultRW, float tau_c, float omega)
{
	/*Definition of Variables*/
	float tau_hat_c, tau_hat_f;

	/*Check if we are command to fault and that command is given to the primary reaction wheel*/
	if (isPrimaryRWactive == 1) && (cmdToFaultRW == 1)
	{
		if (omega < delta_omega) && (omega > -delta_omega)
		{
			tau_hat_c = 0; /* Determine if the motor is spinning too slowly and if it is set commanded torque to zero */
		}
		else 
		{
			t_hat_f = calcInducedFric(); /*Calculate induced friction*/
			tau_hat_c = tau_c - tau_f; /*Add induced firiction into the commanded torque to appear af fault*/
		}
	}
	else
	{
		tau_hat_c = tau_c; /*If not commanded to fault keep commanded torque the same*/
	}
return tau_hat_c;
}