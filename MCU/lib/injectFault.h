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

float injectFault(int isPrimaryRWactive, int cmdToFaultRW, float tau_c, float omega);

#ifdef __cplusplus
}
#endif
#endif
