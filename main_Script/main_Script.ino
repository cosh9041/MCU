#include <Servo.h> 
Servo myservo;

//#include <faultManagement.h>
#include <rwInjection.h>
#include <fm_util.h>
#include <fi_util.h>
#include <data_util.h>
#include <SPI.h>  
#include <PixySPI_SS.h>
#include <PID_v1.h>
#include <DuePWM.h>
#include <stdio.h>
#include <inttypes.h>
#include <fsInjection.h>

// Motor defines
#define PWM_FREQ1  6600
#define PWM_FREQ2  6600
#define LOGIC_ANALYZER_PIN 33
#define PRIMARY_MOTOR_ENABLE_PIN 29
#define REDUNDANT_MOTOR_ENABLE_PIN 39
#define PRIMARY_MOTOR_PIN 6
#define REDUNDANT_MOTOR_PIN 7

// Pixy defines
#define PRIMARY_FS_PIN 48
#define SECONDARY_FS_PIN 46
#define CS_PIN 50




/*


Start of fm code

*/

void faultManagement(FmState *fmState, double *rwSpeedHist, unsigned long *timeStampHist, uint16_t rwDataLength, 
		double *commandedTorque, double *fineDelTheta, double *coarseDelTheta, uint16_t sensorDataLength, double MOI);

void manageNewFaultDetected(FmState *fmState);

uint8_t faultCheckRW(FmState *fmState, double *frictionTorque, double *commandedTorque, double *rwSpeed, uint16_t length);

uint8_t checkThreshold(double *data_x, double *data_y, uint16_t length, double threshold);

void manageFaultAlreadyDetected(FmState *fmState);

uint8_t faultCheckFS(FmState *fmState, double *coarseDelTheta, double *fineDelTheta, uint16_t length);

uint8_t handleFaultStatus(FmState *fmState, uint8_t faultDetected);

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t. then multiplies by Moment of intertia (MOI)
// to get the response torque on the reaction wheel
void getResponseTorque(double *omega, double *t, double *responseTorque, double *commandedTorque, 
	double *frictionTorque, uint16_t length, double MOI);






#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "Arduino.h"
#include <math.h>
#include <fm_util.h>
//#include "faultManagement.h"

void faultManagement(FmState *fmState, double *rwSpeedHist, unsigned long *timeStampHist, uint16_t rwDataLength, 
		double *commandedTorque, double *fineDelTheta, double *coarseDelTheta, uint16_t sensorDataLength, double MOI) {
	if (fmState->isFaulted) {
		manageFaultAlreadyDetected(fmState);
		return;
	}

	double responseTorque[rwDataLength];
	double frictionTorque[rwDataLength];
	getResponseTorque(rwSpeedHist, timeStampHist, responseTorque, commandedTorque, frictionTorque,
		rwDataLength, MOI);

	uint8_t rwFaultDetected = faultCheckRW(fmState, frictionTorque, commandedTorque, rwSpeedHist, rwDataLength); 
	if (rwFaultDetected) {
		if (fmState->isRecovering) return;
		fmState->faultType = 2;
	}
	uint8_t fsFaultDetected = 0;
	// uint8_t fsFaultDetected = faultCheckFS(fmState->faultType, coarseDelTheta, fineDelTheta, rwDataLength);
	// if (fsFaultDetected){
		// if (fmState->isRecovering) return;
	// 	fmState->faultType = 1;
	// }

	/* fmState->faultType will be 0 if there is no fault, 1 if fs fault, 2 if RW*/
	if (!rwFaultDetected && !fsFaultDetected) {
		fmState->faultType = 0;
		fmState->isRecovering = 0; 
		return; 
	}

	manageNewFaultDetected(fmState);
}

// Performs numerical differentiation to determine angular acceleration 
// by taking the difference of omega / difference of t, then multiply by
// moment of intertia to calculate response torque of system
void getResponseTorque(double *omega, unsigned long *t, double *responseTorque, double *commandedTorque, 
	  double *frictionTorque, uint16_t length, double MOI) {
  double omegaDiff;
  int tDiff;
  double tDiffs;
  double angularAccel;
  Serial.println("Starting loop........................................................");
  for (int i = 0; i < length-1; i++) {
    omegaDiff = omega[i] - omega[i+1];
    tDiff = int(t[i] - t[i+1]);
    tDiffs = tDiff * .000001;
    //Serial.println("Omega");
    Serial.println(omega[i]);
    // Serial.println("tDiffs");
    // Serial.println(tDiffs, 10);

    if (tDiff != 0) {
      angularAccel = omegaDiff/tDiffs;
      responseTorque[i] = angularAccel * MOI;
    } else {
      Serial.println("Should really never be here");
      responseTorque[i] = 0;
    }
	  frictionTorque[i] = commandedTorque[i] - responseTorque[i];
    // Serial.println("omegaDiff");
    // Serial.println(omegaDiff, 15);
    // Serial.println("tDiff");
    // Serial.println(tDiff);
    // Serial.println("Angular Accel");
    // Serial.println(angularAccel);
    // Serial.println("Calculated friction torque");
    // Serial.println(frictionTorque[i], 15);
    // Serial.println("Response torque");
    // Serial.println(responseTorque[i], 15);
    // Serial.println("commanded torque");
    // Serial.println(commandedTorque[i], 15);
  }

  delay(10000);
  Serial.println("Starting time loop");
  for (int i = 0; i < length-1; i++) {
    tDiff = int(t[i] - t[i+1]);
    tDiffs = tDiff * .000001;
    Serial.println(tDiffs, 10);
  }
  delay(10000);
}

void manageNewFaultDetected(FmState *fmState) {
	fmState->isFaulted = 1;
	fmState->faulting = 0;
	/*TODO: Function to Alert GSU Function here*/
	//alertGSU(fmState->faultType);
	switch(fmState->faultType) {
		case 2:  // RW fault. turn off command to RW 1 to allow for visible deterioration of control
			Serial.println("RW fault detected");
			Serial.println(fmState->faultType);
			fmState->activeRW = 0;
      delay(10000);
			break;
		case 1: break; // FS fault. In this case, we do nothing
		default: break;
	}
}

void manageFaultAlreadyDetected(FmState *fmState) {
	// fmState->cmdToRecover will be 1 when the system is commanded to initiate recovery
	// from the GSU. The setting of this bit is handled by the communication
	// handlers
	if (!fmState->cmdToRecover)
		return;

	fmState->cmdToRecover = 0;
	//recovery(fmState->faultType); TODO: re-impl Initiate recovery func
	fmState->isFaulted = 0;
	fmState->isRecovering = 1;
}

uint8_t checkThreshold(double *data_x, double *data_y, uint16_t length, double threshold) {
	double sum = 0;
	for (int i = 0; i < length; i++) {
		sum += fabsf(data_y[i] - data_x[i]);
	}
	double meanDiff = sum/length;
	if (meanDiff > threshold) return 1;
	return 0; 
}

uint8_t faultCheckRW(FmState *fmState, double *frictionTorque, double *commandedTorque, double *rwSpeed, uint16_t length) {
	double expectedFriction[length];
	for (int i = 0; i < length; i++) {
		expectedFriction[i] = fmState->p1*rwSpeed[i] + fmState->p2;
	}

	uint8_t faultDetected = checkThreshold(frictionTorque, expectedFriction, length-1, 3*(fmState->p2)); 
	return handleFaultStatus(fmState, faultDetected);
}

uint8_t faultCheckFS(FmState *fmState, double *coarseDelTheta, double *fineDelTheta, uint16_t length) {
	uint8_t faultDetected, faultTimerActive;
	
	/*Run threshold check*/
	//faultDetected = checkThreshold(); //TODO: Impl check threshold w/ data. Tune for run time perf

	handleFaultStatus(fmState, faultDetected);
}

uint8_t handleFaultStatus(FmState *fmState, uint8_t faultDetected) {
	if (!faultDetected) {
		fmState->faultTimerActive = 0;
		return 0;
	}

	if (!fmState->faultTimerActive && !fmState->isRecovering) {
		fmState->faultTimerStart = millis();
		fmState->faultTimerActive = 1;
		return 0;
	}
	
	if ((millis() - fmState->faultTimerStart) > fmState->timeToFault) {
		if (fmState->faultTimerActive) {
			fmState->faultTimerActive = 0;
			return 1;
		} else if (fmState->isRecovering) {
			return 1;
		}
	}
	return 0;
}


///// UNFUCK THIS




























DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

PixySPI_SS finePixy1(PRIMARY_FS_PIN);
PixySPI_SS finePixy2(SECONDARY_FS_PIN);
PixySPI_SS coarsePixy(CS_PIN);

unsigned long timeLastReadPixy = 0;

//Define Variables we'll be connecting to
double deltaThetaRadFine1, deltaThetaRadFine2;
double deltaThetaRadCoarse;
double Setpoint, deltaThetaRad, commandedTorque_mNm;

//Specify the links and initial tuning parameters
double const Kp=6.31600775000411, Ki=0.200331632823233, Kd=36.7969404030176, N=0.5;
PID myPID(&deltaThetaRad, &commandedTorque_mNm, &Setpoint, Kp, Ki, Kd, N, DIRECT);

//Allocate for fm/fi state variables
FmState fmBase;
FmState *fmState = &fmBase;
FiState fiBase;
FiState *fiState = &fiBase;

//conversions for torques to PWM bins
double mNm_to_mA = 1000.0/8.4;
double mA_to_duty = 80.0/1800.0;
double duty_to_bin = 256.0/100.0;
double pwm_duty;
double pwmOffset = 50;
double pwm_duty2;
uint32_t pwm_duty3 = 127;
uint32_t pwm_duty_inactive = 127;
static int i = 0;
int k;
char buf[32];
uint16_t fineBlocks1;
uint16_t fineBlocks2;
uint16_t coarseBlocks;     

// Pixy conversions
double const convertPixToDegCoarse = 0.2748;
double const convertPixToDegFine = 0.0522;
double const centerOffsetDegFine = 160*convertPixToDegFine;
double const centerOffsetDegCoarse =160*convertPixToDegCoarse;
double const convertDegToRad = 3.1415926535897932384626433832795/180; 

// Reaction wheel speed variable
uint16_t rwSpeedBin;
double rwSpeedRad; 

// Physical characteristics
double const p1 = 0.0000335653965; // mNm/(rad/s), viscous friction 
double const p2 = 0.1303; // mNm, coloumb friction
double MOI = .047; //MOI of RW, kg*m^2
double delta_omega = 15; // small delta near zero where we set torque to zero to simulate friction. TODO: Tune this value
uint16_t const rwDataLength = 1000;
uint16_t const sensorDataLength = 50;
// rwStackPtr points to the most recent data point. The last 100 data points are accumulated "behind"
// this index. For example, if the rwStackPtr is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
uint16_t rwStackPtr = 0;
uint16_t sensorStackPtr = 0;

//TODO: Determine units and ideal length for these. 
double commandedTorqueHistory[rwDataLength];
double rwSpeedHist[rwDataLength];
unsigned long timeStampHistory[rwDataLength];
double orderedRWSpeedHistory[rwDataLength];
double orderedCommandedTorqueHistory[rwDataLength];
unsigned long orderedTimeStampHistory[rwDataLength];
double angularAccel[rwDataLength-1];
double fineDeltaTheta[sensorDataLength];
double coarseDeltaTheta[sensorDataLength];

// Utility function specifications. Implementations are below loop()
void sendTorque();
void getRWSpeed(double *rwSpeedRad, uint16_t analogReading);
void runControl();
void injectTimedRWFault();
void injectTimedFSFault();
void getPrimaryFSReading();
void getSecondaryFSReading();
void getCSReading();
void getBlockReading();
void runFM();

void setup() { 
  Serial.begin(9600);
  Serial.print("Starting...\n");
  finePixy1.init();
  finePixy2.init();
  coarsePixy.init();
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Initialize Pixy's
  digitalWrite(CS_PIN,HIGH);
  digitalWrite(PRIMARY_FS_PIN,HIGH);
  digitalWrite(SECONDARY_FS_PIN,HIGH);

  //Configure PWM for motors
  pwm.setFreq1( PWM_FREQ1 );
  pwm.setFreq2( PWM_FREQ2 );

  pwm.pinFreq1(PRIMARY_MOTOR_PIN);  // Pin 6 freq set to "pwm_freq1" on clock A
  pwm.pinFreq2(REDUNDANT_MOTOR_PIN);  // Pin 7 freq set to "pwm_freq2" on clock B

  // Initialize Pixy's
  digitalWrite(CS_PIN,HIGH);
  digitalWrite(PRIMARY_FS_PIN,HIGH);
  digitalWrite(SECONDARY_FS_PIN,HIGH);
  
  // Set pin modes for pixy
  pinMode(PRIMARY_FS_PIN, OUTPUT);
  pinMode(SECONDARY_FS_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // Sets the resolution of the ADC for rw speed feedback to be 12 bits (4096 bins). 
  analogReadResolution(12);
  pinMode(A0, INPUT);

  //Enable Primary Motor Cycle for Initialization; both motor controllers are 
  //written high and switching comes from the pwm signals sent to the motor controller
  //in the loop.
  digitalWrite(LOGIC_ANALYZER_PIN, HIGH); //set ref of logic analyzer to 3.3V
  digitalWrite(REDUNDANT_MOTOR_ENABLE_PIN, LOW);  //drive redundant motor low (disabled)
  digitalWrite(PRIMARY_MOTOR_ENABLE_PIN, HIGH); //drive primary motor high (enabled)
  delay(1000);
  digitalWrite(REDUNDANT_MOTOR_ENABLE_PIN, HIGH); //cycle redundant motor high (enabled)
  digitalWrite(PRIMARY_MOTOR_ENABLE_PIN, LOW); //cycle primary motor low (disabled)
  delay(1000);
  digitalWrite(PRIMARY_MOTOR_ENABLE_PIN, HIGH);//cycle primary motor high (enabled)

  initializeFaultManagementState(fmState, rwDataLength-1, p1, p2);
  initializeFaultInjectState(fiState);
} 

void loop() {
  getRWSpeed(&rwSpeedRad, analogRead(A0));

  unsigned long timeStamp = micros();
  // Serial.println("Storing RW speed with time: ");
  // Serial.println(timeStamp);
  // Serial.println(rwStackPtr);
  storeRWSpeed(rwSpeedHist, timeStampHistory, rwStackPtr, rwSpeedRad, timeStamp);

  // Stores ordered lists of last n = `rwDataLength` data points, with most recent being at index 0
  getOrderedHistory(rwSpeedHist, orderedRWSpeedHistory, rwDataLength, rwStackPtr);
  getOrderedHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, rwDataLength, rwStackPtr);
  getOrderedHistoryLong(timeStampHistory, orderedTimeStampHistory, rwDataLength, rwStackPtr);

  // Ensure the system is operational before starting fm
  if (millis() > 25000) runFM();

  if (fmState->faulting == 1 && fiState->cmdToFaultRW) {
    Serial.println("We're in a faulting state, detected, nice");
  }

  if (fmState->faulting == 1 && !(fiState->cmdToFaultRW)) {
    Serial.println("Detected fault when none present, fuck");
  }

  if (fmState->faulting == 0 && fiState->cmdToFaultRW) {
    Serial.println("Did not detect fault when fault was present, fuck");
  }

  // NOTE: Pixy.getBlocks does a dirty nasty thing and returns 0 both when there are no blocks
  // detected AND when there is no information. This ambiguous case cost us a lot of wasted time
  if ((millis() - timeLastReadPixy) > 20) {
    getBlockReading();
    timeLastReadPixy = millis();
  }

  if (fineBlocks1 || fineBlocks2) {
    deltaThetaRad = fmState->activeFS == 1 ? deltaThetaRadFine1 : deltaThetaRadFine2;

    // uncomment out these lines to inject a fs or rw fault. DO NOT DELETE UNTIL GSU IS INTEGRATED
    //injectTimedRWFault();
    //injectTimedFSFault();

    runControl();
  } else if(coarseBlocks) {
    deltaThetaRad = deltaThetaRadCoarse;
    runControl();
  } else {
    // If we do not pick up blocks set PWM to 50% to shut off motors
    // TODO: Figure out what the commanded torque is for the no-detect case here
    commandedTorque_mNm = 0;
    pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_inactive);
    pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_inactive);
  }
  storeTorqueAndIncrementIndex(commandedTorqueHistory, &rwStackPtr, commandedTorque_mNm, rwDataLength);
}

void runFM() {
  // faultManagement will update fmState struct contents to reflect fault detected or not
  faultManagement(fmState, orderedRWSpeedHistory, orderedTimeStampHistory, rwDataLength, orderedCommandedTorqueHistory, fineDeltaTheta, coarseDeltaTheta,
	    sensorDataLength, MOI);
}

void runControl() {
  // call to Compute assigns output to variable commandedTorque_mNm via pointers
  myPID.Compute(); 

  // TODO: Test injection strength and tune for delta_omega
  commandedTorque_mNm = injectRWFault(fiState, commandedTorque_mNm, rwSpeedRad, p1, p2, delta_omega, 
    fmState->activeRW == 1);

  sendTorque();
}

void sendTorque() {
  pwm_duty = (commandedTorque_mNm*mNm_to_mA*mA_to_duty + pwmOffset)*duty_to_bin;

  if (pwm_duty > 230) {
    pwm_duty = 230;
  } else if (pwm_duty < 26) {
    pwm_duty = 26; 
  }
  pwm_duty2 = round(pwm_duty);
  pwm_duty3 = (uint32_t) pwm_duty2;

  // Uncomment these lines to do a timed  switch to secondary rw
  // if (millis() > 20000 && fmState->activeRW == 1) {
  //   fmState->activeRW = 2;
  // }

  // Testing switching b/w rw
  if (fmState->activeRW == 1) {
    pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty3);  // computed duty cycle on Pin 6 (Primary RW)
    pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_inactive);  // inactive duty cycle (127 PWM) on Pin 7 (Secondary RW)
  } else if (fmState->activeRW == 2) {
    pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_inactive); // inactive duty cycle (127 PWM) on Pin 6 (Primary RW)
    pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty3);         // computed duty cycle on Pin 7 (Secondary RW)
  } else {
    pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_inactive); // inactive duty cycle (127 PWM) on Pin 6 (Primary RW)
    pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_inactive); // computed duty cycle on Pin 7 (Secondary RW)
  }

}

void getBlockReading() {
  // NOTE: TO run on board which is not pixy enabled, comment out the getBlocks() calls
  fineBlocks1 = finePixy1.getBlocks(); 
  if (fineBlocks1) {
    getPrimaryFSReading();
  }

  fineBlocks2 = finePixy2.getBlocks();// NOTE: TO run on board which is not pixy enabled, comment this line out
  if (fineBlocks2) {
    getSecondaryFSReading();
  }

  coarseBlocks = coarsePixy.getBlocks();
  if (coarseBlocks) {
    getCSReading();
  }
}

void getRWSpeed(double *rwSpeedRad, uint16_t analogReading) {
  // hard coded in the '- 81' portion. This tunes down to 0 rads. most likely our system isn't perfect and is causing this. Not sure tho
  *rwSpeedRad = (28000/4096*analogReading - 14000)*2*PI/60 - 81;
  Serial.println("analogReading");
}

void getPrimaryFSReading() {
  deltaThetaRadFine1 = ((finePixy1.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;
  fsInjection(&deltaThetaRadFine1, fiState);
}

void getSecondaryFSReading() {
  deltaThetaRadFine2 = ((finePixy2.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;
}

void getCSReading() {
  deltaThetaRadCoarse = ((coarsePixy.blocks[0].x)*convertPixToDegCoarse - centerOffsetDegCoarse)*convertDegToRad;
}

void injectTimedRWFault() {
  // uncomment out these lines to inject a rw fault. DO NOT DELETE UNTIL GSU IS INTEGRATED
  if (millis() > 40000 && !fiState->cmdToFaultRW && millis() < 80000) {
    Serial.println("faulting rw");
    fiState->cmdToFaultRW = 1;
  }
  if (millis() > 100000 && fiState->cmdToFaultRW) {
    Serial.println("Unfaulting rw");
    fiState->cmdToFaultRW = 0;
  }
}

void injectTimedFSFault() {
  if (millis() > 40000 && !fiState->cmdToFaultFS && millis() < 80000) {
    Serial.println("faulting fs");
    fiState->cmdToFaultFS = 1;
  }
  if (millis() > 100000 && fiState->cmdToFaultFS) {
    Serial.println("Unfaulting fs");
    fiState->cmdToFaultFS = 0;
  }
}

