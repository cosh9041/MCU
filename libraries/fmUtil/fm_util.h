#ifndef FM_UTIL_H_
#define FM_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

typedef struct {
	uint8_t isFaulted;
	uint8_t isRecovering;
	uint8_t faultType;
	uint8_t cmdToRecover;
	uint8_t faultTimerActive;
	uint8_t isPrimaryRWActive;
	uint8_t isPrimaryFSActive;
	uint8_t cmdToFaultRW;
	uint8_t cmdToFaultFS;
	float fsBias;
} FmState;

void initializeFaultState(FmState *fmState);

#ifdef __cplusplus
}
#endif
#endif
