#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fi_util.h"

void initializeFaultInjectState(FiState *fiState) {
  fiState->fsBias = 10;
  fiState->cmdToFaultRW = 0;
  fiState->cmdToFaultFS = 0;

}