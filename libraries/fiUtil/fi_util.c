#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fi_util.h"

void initializeFaultInjectState(FiState *fiState) {
  fiState->fsBias = 10.0/360.0*2.0*3.14159265358979; //in radians
  fiState->cmdToFaultRW = 0;
  fiState->cmdToFaultFS = 0;
}