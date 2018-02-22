/*
 * faultManagement.h
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 * 
*/

#ifndef FAULTMANAGEMENT_H_
#define FAULTMANAGEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

void faultManagement(int *pcmdToRecover, int *pisRecovering, int *pfaultType, int *isFaulted, int *FaultTimerActive);

int checkThreshold();

int faultCheck();

int recovery(int faultType);

#ifdef __cplusplus
}
#endif
#endif
