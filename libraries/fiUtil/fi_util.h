#ifndef FI_UTIL_H_
#define FI_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

typedef struct {
	double fsBias;
	uint8_t cmdToFaultRW;
	uint8_t cmdToFaultFS;
} FiState;

void initializeFaultInjectState(FiState *fiState);

#ifdef __cplusplus
}
#endif
#endif