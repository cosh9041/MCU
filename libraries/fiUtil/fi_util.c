#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include "fi_util.h"

void initializeFaultInjectState(FiState *fiState) {
  fiState->fsBias = 6.0*2.0*M_PI/360;
  fiState->cmdToFaultRW = 0;
  fiState->cmdToFaultFS = 0;
}