#include "data_util.h"
#include <inttypes.h>

void storeRWSpeed(float *rwSpeedHistory, float *timeHistory, uint16_t index, float rwSpeed, float timeStamp) {
  rwSpeedHistory[index] = rwSpeed;
  timeHistory[index] = timeStamp;
}

void storeTorqueAndIncrementIndex(float *commandedTorqueHistory, uint16_t *index, float commandedTorque_mNm, uint16_t length) {
  *index += 1;
  if (*index == length) *index = 0;
  commandedTorqueHistory[*index] = commandedTorque_mNm;
}

// Stores the last n = `length` data points from `data` in a destination array in a time ordered fashion
// currentIndex points to the most recent data point. The last n = `length` data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
void getOrderedHistory(float *data, float *destination, uint16_t length, uint16_t index) {
  uint16_t historyIndex = 0;
  int32_t i; 
  for (i = index; i >= 0; i--) {
    destination[historyIndex] = data[i];
    historyIndex++;
  }
  for (i = (length - 1); i > index; i--) {
    destination[historyIndex] = data[i];
    historyIndex++;
  }
}

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t
void getAngularAcceleration(float *omega, float *t, float *alpha, uint16_t length) {
  float omegaDiff;
  float tDiff;
  for (int i = 0; i < length-1; i++) {
    omegaDiff = omega[i+1] - omega[i];
    tDiff = t[i+1] - t[i];
    if (tDiff != 0)
      alpha[i] = omegaDiff / tDiff;
    else
      alpha[i] = 0;
  }
}