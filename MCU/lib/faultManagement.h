/*
 * faultManagement.h
 *
 * Created: 1/29/2018 2:41:34 PM
 * Author : Pol Sieira
 */ 
/*#ifndef FAULTMANAGEMENT_H_
#define FAULTMANAGEMENT_H_

class FaultManagement {
	public:
		FaultManagement();
    void thisIsATest();
    void faultManagement();
	private:
		int faultCheck();
    int checkThreshold();
    int faultCheckFS();
    int faultCheckRW();
    void recovery(int faultType);
};
#endif*/

#ifndef FAULTMANAGEMENT_H_
#define FAULTMANAGEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

void faultManagement();

int checkThreshold();

int faultCheck();

void recovery(int faultType);

#ifdef __cplusplus
}
#endif
#endif