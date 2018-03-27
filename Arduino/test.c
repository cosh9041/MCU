
#include <stdio.h>
#include "LLAMAS/faultManagement.h"
#include "LLAMAS/rwInjection.h"

//TODO: recovery seems to be doing some really trivial shit. Let's revisit this
// and see if it's really necessary
void testRecovery() {
	printf("Testing recovery.c...\n");
	unsigned char didTestFail = 0;

	//printf("Testing with no recovery...\n");
	if (recovery(0) != 0) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	//printf("Testing recovery of fine sensor...\n");
	if (recovery(1) != 1) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	//printf("Testing recovery of reaction wheel..\n");
	if (recovery(2) != 2) {
		printf("Test failed!\n");
		didTestFail = 1;
	}

	if (!didTestFail) {
		printf("All recovery tests passed!\n");
	}
}

void testRWInjection() {
	printf("Testing rwInjection.c...\n");
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

	printf("Test injectFault...\n");
	//Test that it doesn't modify when there is not fault to inject
	unsigned char isPrimaryRWActive = 1;
	unsigned char cmdToFaultRW = 0;
	float tau_c = 10.0;
	float delta_omega = 0.01;

	float tau_c_augmented = injectFault(isPrimaryRWActive, cmdToFaultRW, tau_c,
			omega, p1, p2, delta_omega);
	if (tau_c_augmented != tau_c) {
		printf("injectFault changed the commanded torque when not commaned to\n");
		didTestFail = 1;
	}

	//Test that it doesn't modify when the secondary RW is active
	isPrimaryRWActive = 0;
	cmdToFaultRW = 1;
	tau_c_augmented = injectFault(isPrimaryRWActive, cmdToFaultRW, tau_c,
			omega, p1, p2, delta_omega);
	if (tau_c_augmented != tau_c) {
		printf("injectFault changed the commanded torque when secondary RW was active\n");
		didTestFail = 1;
	}

	//Test that returned torque is 0 when omega is less than delta_omega
	isPrimaryRWActive = 1;
	omega = 0.009;
	tau_c_augmented = injectFault(isPrimaryRWActive, cmdToFaultRW, tau_c,
			omega, p1, p2, delta_omega);
	if (tau_c_augmented != 0.0) {
		printf("injectFault did not properly handle small omega case\n");
		didTestFail = 1;
	}
	omega = -0.009;
	tau_c_augmented = injectFault(isPrimaryRWActive, cmdToFaultRW, tau_c,
			omega, p1, p2, delta_omega);
	if (tau_c_augmented != 0.0) {
		printf("injectFault did not properly handle small omega case\n");
		didTestFail = 1;
	}

	//Test that returned torque is augmented if commaned to fault and  
	//omega > delta_omega
	isPrimaryRWActive = 1;
	omega = 0.1;
	tau_c_augmented = injectFault(isPrimaryRWActive, cmdToFaultRW, tau_c,
			omega, p1, p2, delta_omega);
	float expected_tau_c = tau_c - calcInducedFriction(omega, p1, p2);
	if (tau_c_augmented != expected_tau_c) {
		printf("injectFault did not properly augment torque under nominal injection case\n");
		didTestFail = 1;
	}

	if (!didTestFail) {
		printf("All rwInjection tests passed!\n");
	}
}

void testFM() {
	printf("Testing faultManagement.c...\n");
	int cmdToRecover = 0;
	int isRecovering = 0;
	int faultType = 0;
	int isFaulted = 0;
	int faultTimerActive = 0;
	//faultMangement(&cmdToRecover, &isRecovering, &faultType, &isFaulted, &faultTimerActive); 
}


int main(void) {
	printf("Running unit tests...\n");
	testRecovery();
	testRWInjection();
	testFM();
}


