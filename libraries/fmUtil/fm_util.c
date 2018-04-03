#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fm_util.h"

void initializeFaultState(FmState *fmState) {
  fmState->isFaulted = 0;
  fmState->isRecovering = 0;
  fmState->faultType = 0;
  fmState->cmdToRecover = 0;
  fmState->faultTimerActive = 2;
  fmState->isPrimaryRWActive = 0;
  fmState->isPrimaryFSActive = 1;
}
