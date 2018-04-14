/*
 * rwInjection.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author(s) : Pol Sieira & Corwin Sheahan
 */ 

#include <fi_util.h>
#include <inttypes.h>
#include "rwInjection.h"


// Calculates the induced friction of a reaction wheel as a function of the natural friction
// Variables [All units are SI units]:
//	-omega: Reaction wheel speed 
//	-p1 & p2: polynomial coefficients that yield an approximation polynominal 
//		for the nominal friction of the form f(x) = p1*x + p2, where x is the 
//		wheel speed in rad/s. p1 and p2 should be determined by empirical analysis
//		of reaction wheel friction
//	-I: Moment of inertia of reaction wheel
float calcInducedFriction(float omega, float p1, float p2) {
	float induced;
	if (omega < 0)
		induced = p1*omega - p2*5;
	if (omega > 0)
		induced = p1*omega + p2*5;
	return induced; 
}

// returns an injected torque value:
// Variables [All units are SI units]:
//	-isPrimaryRWactive: Should be 1 if primary reaction wheel is active, 0 otherwise
float injectRWFault(FiState *fiState, float tau_c, float omega, float p1, float p2, float delta_omega, uint8_t primaryRWActive) {
	float tau_hat_c, tau_hat_f;
	if (primaryRWActive && fiState->cmdToFaultRW) {
		if (omega < delta_omega && omega > -delta_omega) {
			// If the reaction wheel speed is close to 0, then we actually want to just 
			// turn off the commanded torque. The inclusion of friction could force the
			// augmented friction to spin the wheel past 0 and in the opposite direction,
			// which is impossible for natural friction to do.
			tau_hat_f = calcInducedFriction(omega, p1, p2); 
			if (tau_c < tau_hat_f && tau_c > 0) {
				return 0;
			}
			if (tau_c > tau_hat_f && tau_c < 0) {
				return 0;
			}
			return tau_hat_c - tau_hat_f;
		} else {
			tau_hat_f = calcInducedFriction(omega, p1, p2); 
			// The commanded torque should be decreased by the 'induced' friction, which
			// mimics an increase in the natural friction of the reaction wheel
			tau_hat_c = tau_c - tau_hat_f; 
		}
	}
	else {
		tau_hat_c = tau_c; 
	}
	return tau_hat_c;
}
