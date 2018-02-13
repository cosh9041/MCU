#include <stdio.h>
#include "lib/faultManagement.h"



void testFM() {
	FaultManagement fm = FaultManagement();
	fm.thisIsATest();
}

int main(void) {
	printf("Running unit tests...\n");
	printf("Testing fault management...\n");
	testFM();
}
