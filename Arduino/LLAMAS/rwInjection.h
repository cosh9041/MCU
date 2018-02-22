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

float calcInducedFriction(float omega, float p1, float p2);

float injectFault(unsigned char isPrimaryRWactive, unsigned char cmdToFaultRW, float tau_c, 
		float omega, float p1, float p2, float delta_omega);

#ifdef __cplusplus
}
#endif
#endif
