#include <stdio.h>
//#include "LLAMAS/faultManagement.h"
//#include "LLAMAS/rwInjection.h"
#include "LLAMAS/recovery.h"
#include "LLAMAS/rwInjection.h"

//TODO: recovery seems to be doing some really trivial shit. Let's revisit this
// and see if it's really necessary
void testRecovery() {
	printf("Testing recovery.c...\n");
	unsigned char didTestFail = 0;

	printf("Testing with no recovery...\n");
	if (recovery(0) != 0) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	printf("Testing recovery of fine sensor...\n");
	if (recovery(1) != 1) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	printf("Testing recovery of reaction wheel..\n");
	if (recovery(2) != 2) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	if (!didTestFail) {
		printf("All recovery tests passed!\n");
	}
}

void testRWInjection() {
	printf("Testing Reaction wheel fault injection...\n");
	unsigned char didTestFail = 0;

	printf("Testing calcInducedFriction...\n");
	float omega = 10;
	float p1 = .00076039;
	float p2 = .2033;
	float inducedFriction = calcInducedFriction(omega, p1, p2);
	float threshold = p1*omega + 4*p2;
	if (threshold > inducedFriction)  {
		printf("Test Failed. CalcInducedFriction produced too low of a value\n");
		didTestFail = 1;
	}

	if (!didTestFail) {
		printf("All rwInjection tests passed!\n");
	}
}


int main(void) {
	printf("Running unit tests...\n");

	testRecovery();
	testRWInjection();
}


