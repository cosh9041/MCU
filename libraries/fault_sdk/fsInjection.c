/*
 * fsInjection.c
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <fm_util.h>
#include <inttypes.h>
#include "fsInjection.h"


void fsInjection(double *deltaTheta, FmState *fmState)
{   
	if (fmState->cmdToFaultFS){
		*deltaTheta += fmState->fsBias;
	}
}
