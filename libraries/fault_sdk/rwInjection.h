/*
 * rwInjection.h
 *
 * Created: 1/29/2018 2:00:57 PM
 * Author : Pol Sieira
 */ 

#ifndef INJECTFAULT_H_
#define INJECTFAULT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <fi_util.h>

double calcInducedFriction(double omega, double p1, double p2);

double injectRWFault(FiState *fiState, double tau_c, double omega, double p1, double p2, 
	double delta_omega, uint8_t primaryRWActive);

#ifdef __cplusplus
}
#endif
#endif
