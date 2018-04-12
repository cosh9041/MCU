#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fi_util.h"

void initializeFaultInjectState(FiState *fiState) {
  fiState->fsBias = 4/360*2*3.14159265358979; //in radians
  fiState->cmdToFaultRW = 0;
  fiState->cmdToFaultFS = 0;
}