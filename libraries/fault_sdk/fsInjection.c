/*
 * fsInjection.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <fi_util.h>
#include <inttypes.h>
#include "fsInjection.h"

double fsInjection(double deltaTheta, FiState *fiState)
{   
	if (fiState->cmdToFaultFS){
		deltaTheta += fiState->fsBias;
	}
	return deltaTheta;
}
