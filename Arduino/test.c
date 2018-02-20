#include <stdio.h>
//#include "LLAMAS/faultManagement.h"
//#include "LLAMAS/rwInjection.h"
#include "recovery.h"

/*void testRwInjection() {
	float tau_c = 0.0;
	float omega = 0.0;
	RwInjection rw = RwInjection();
	rw.injectFault(0, 0, tau_c, omega);
}*/

/*void testFM() {
	FaultManagement fm = FaultManagement();
	fm.faultManagement();
}*/

int main(void) {
	printf("Running unit tests...\n");
	printf("Testing fault management...\n");
//	testFM();
	printf("Testing fault reaction wheel fault injection...\n");
//	testRwInjection();
	int penis = recovery(1);
	printf("%d\n",penis);
}

