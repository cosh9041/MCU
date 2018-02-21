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


// Calculates the induced friction of a reaction wheel as a function of the natural friction
// Variables [All units are SI units]:
//	-omega: Reaction wheel speed 
//	-p1 & p2: polynomial coefficients that yield an approximation polynominal 
//		for the nominal friction of the form f(x) = p1*x + p2, where x is the 
//		wheel speed in rad/s. p1 and p2 should be determined by empirical analysis
//		of reaction wheel friction
//	-I: Moment of inertia of reaction wheel
float calcInducedFric(float omega, float p1, float p2, float I)
{
	fload induced = p1*omega + p2*5;
	return induced; 
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
