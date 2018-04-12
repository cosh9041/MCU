/*
 * fsInjection.h
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#ifndef FSINJECTION_H_
#define FSINJECTION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <fi_util.h>

double fsInjection(double deltaTheta, FiState *fiState);

#ifdef __cplusplus
}
#endif
#endif
