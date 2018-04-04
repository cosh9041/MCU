#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fm_util.h"

void initializeFaultManagementState(FmState *fmState) {
  fmState->isFaulted = 0;
  fmState->isRecovering = 0;
  fmState->faultType = 0;
  fmState->cmdToRecover = 0;
  fmState->faultTimerActive = 0;
  fmState->activeRW = 1;
  fmState->activeFS = 1;
  fmState->faultTimerStart = 0;
}
