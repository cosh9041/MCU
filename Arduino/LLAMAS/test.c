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
	printf("\nRunning unit tests...\n\n");
//	printf("Testing fault management...\n");
//	testFM();
//	printf("Testing fault reaction wheel fault injection...\n");
//	testRwInjection();
	
	// Test for recovery.c 
	printf("Testing recovery.c...\n\n");
	
	// Test for no recovery
	printf("Testing with no recovery...\n");
	if (recovery(0) == 0)
	{
		printf("Output is %d\n", recovery(0));
		printf("Test Succesful!\n");
	}
	else
	{
		printf("Test failed!\n");
	}
	// Test for recovery of fine sensor
	printf("\nTesting recovery of fine sensor...\n");
	if (recovery(1) == 1)
	{
		printf("Output is %d\n", recovery(1));
		printf("Test success!\n");
	}
	else
	{
		printf("Test failed!\n");
	}
	// Test for recovery of reaction wheel
	printf("\nTesting recovery of reaction wheel..\n");
	if (recovery(2) == 2)
	{
		printf("Output is %d\n", recovery(2));
		printf("Test succesful!\n");
	}
	else
	{
		printf("Test failed!\n");
	}
}


