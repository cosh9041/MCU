/*
 * rwInjection.h
 *
 * Created: 1/29/2018 2:00:57 PM
 * Author : Pol Sieira
 */ 

#ifndef RWINJECTION_H_
#define RWINJECTION_H_

//#include "sam.h"

class RwInjection {
	public: 
		RwInjection();
		float injectFault(int isPrimaryRWactive, int cmdToFaultRW, float tau_c, float omega);
	private:
		float calcInducedFric(float omega);
};

#endif
