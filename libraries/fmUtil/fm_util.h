#ifndef FM_UTIL_H_
#define FM_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <inttypes.h>

#include <inttypes.h>

typedef struct {
	uint8_t isFaulted; 
	uint8_t isRecovering; 
	uint8_t faultType;
	uint8_t cmdToRecover;
	uint8_t faultTimerActive;
	uint8_t activeRW; //Reaction wheels: 0 == both off, 1 == primary is on, 2 == secondary is on
	uint8_t activeFS; //Fine sensors: 0 == both off, 1 == primary is on, 2 == secondary is on
    unsigned long faultTimerStart;
	uint8_t faulting; // Set to 1 when a fault had been detected, but has not been there for  straight
	unsigned long timeToFault; // Time until system is faulted. Used to debounce blips from triggering fault in system
} FmState;

void initializeFaultManagementState(FmState *fmState);

#ifdef __cplusplus
}
#endif
#endif
