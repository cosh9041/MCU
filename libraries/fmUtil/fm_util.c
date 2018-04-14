#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fm_util.h"

void initializeFaultManagementState(FmState *fmState, uint16_t size_of_ma, float p1, float p2) {
  fmState->isFaulted = 0;
  fmState->isRecovering = 0;
  fmState->faultType = 0;
  fmState->cmdToRecover = 0;
  fmState->faultTimerActive = 0;
  fmState->activeRW = 1;
  fmState->activeFS = 1;
  fmState->faultTimerStart = 0;
  fmState->faulting = 0;
  fmState->timeToFault = 30000;
  fmState->size_of_ma = size_of_ma;
  fmState->p1 = p1;
  fmState->p2 = p2;
}
