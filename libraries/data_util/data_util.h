#ifndef DATA_UTIL_H_
#define DATA_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

void storeRWSpeed(double *rwSpeedHistory, double *timeHistory, uint16_t index, double rwSpeed, double timeStamp);

void storeTorqueAndIncrementIndex(double *commandedTorqueHistory, uint16_t *index, 
    double commandedTorque_mNm, uint16_t length);

// Stores the last n = `length` data points from `data` in a destination array in a time ordered fashion
// currentIndex points to the most recent data point. The last n = `length` data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
void getOrderedHistory(double *data, double *destination, uint16_t length, uint16_t index);

#ifdef __cplusplus
}
#endif
#endif
