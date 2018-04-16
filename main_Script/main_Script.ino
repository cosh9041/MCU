#include <Servo.h> 
Servo myservo;

#include <faultManagement.h>
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

DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

PixySPI_SS finePixy1(PRIMARY_FS_PIN);
PixySPI_SS finePixy2(SECONDARY_FS_PIN);
PixySPI_SS coarsePixy(CS_PIN);

unsigned long timeLastReadPixy = 0;
unsigned long current_time;

//Define Variables we'll be connecting to
double deltaThetaRadFine1, deltaThetaRadFine2;
double deltaThetaRadCoarse;
double Setpoint, deltaThetaRad, commandedTorque_mNm;

//Specify the links and initial tuning parameters
//double const Kp=6.31600775000411, Ki=0.200331632823233, Kd=36.7969404030176, N=0.5;
// THESE ARE PRETTY GOOT
double const Kp=9.31600775000411, Ki=0.800331632823233, Kd=46.7969404030176, N=0.5;
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
uint32_t pwm_duty_slew;
static int i = 0;
int k;
char buf[32];
uint16_t fineBlocks1;
uint16_t fineBlocks2;
uint16_t coarseBlocks;     

// Pixy conversions
double const convertPixToDegCoarse = 0.234375/1.5;
double const convertPixToDegFine = 0.0522;
// double const convertPixToDegCoarse = 0.234375;
// double const convertPixToDegFine = 0.05;
double const centerOffsetDegFine = 160*convertPixToDegFine;
double const centerOffsetDegCoarse = 160*convertPixToDegCoarse;
double const convertDegToRad = PI/180; 

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
double orderedFineDeltaTheta[sensorDataLength];
double orderedCoarseDeltaTheta[sensorDataLength];

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
  Serial.begin(115200);
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
  // Send current time to Labview
  current_time = millis();
  Serial.write(current_time>>24);
  Serial.write(current_time>>16);
  Serial.write(current_time>>8);
  Serial.write(current_time);

  // Pull rw speed from motor controller
  rwSpeedBin = analogRead(A0);
  getRWSpeed(&rwSpeedRad, rwSpeedBin);

  // Send rw speed to Labview
  Serial.write(rwSpeedBin>>8);
  Serial.write(rwSpeedBin);  
  delay(100);

  unsigned long timeStamp = micros();
  storeRWSpeed(rwSpeedHist, timeStampHistory, rwStackPtr, rwSpeedRad, timeStamp);

  // Stores ordered lists of last n = `rwDataLength` data points, with most recent being at index 0
  getOrderedHistory(rwSpeedHist, orderedRWSpeedHistory, rwDataLength, rwStackPtr);
  getOrderedHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, rwDataLength, rwStackPtr);
  getOrderedHistoryLong(timeStampHistory, orderedTimeStampHistory, rwDataLength, rwStackPtr);

  // Ensure the system is operational before starting fm
  if (millis() > 10000) runFM();

  // NOTE: Pixy.getBlocks does a dirty nasty thing and returns 0 both when there are no blocks
  // detected AND when there is no information. This ambiguous case cost us a lot of wasted time
  if ((millis() - timeLastReadPixy) > 20) {
    getBlockReading();
    timeLastReadPixy = millis();
  }

  if (fineBlocks1 || fineBlocks2) {
    deltaThetaRad = fmState->activeFS == 1 ? deltaThetaRadFine1 : deltaThetaRadFine2;

    // uncomment out these lines to inject a fs or rw fault. DO NOT DELETE UNTIL GSU IS INTEGRATED
    // injectTimedRWFault();
    //injectTimedFSFault();
    storeSensorData(fineDeltaTheta, coarseDeltaTheta, sensorStackPtr, deltaThetaRad, deltaThetaRadCoarse);
    getOrderedHistory(fineDeltaTheta, orderedFineDeltaTheta, sensorDataLength, sensorStackPtr);
    getOrderedHistory(coarseDeltaTheta, orderedCoarseDeltaTheta, sensorDataLength, sensorStackPtr);
    sensorStackPtr++;
    if (sensorStackPtr == sensorDataLength) sensorStackPtr = 0;

    if (fmState->isFaulted && fmState->faultType == 1) {
      fmState->activeFS = 2;
    }
    
    runControl();
  } else if(coarseBlocks) {
    deltaThetaRad = deltaThetaRadCoarse;
    runControl();
  } else if (millis() > 5000) {
    // If we do not pick up blocks set PWM to 50% to shut off motors
    pwm_duty_slew = deltaThetaRadCoarse < 0 ? 107 : 147;
    if (fmState->activeRW == 1) {
      pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_slew);
      pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_inactive);
    } else {
      pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_inactive);
      pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_slew);
    }
  }
  storeTorqueAndIncrementIndex(commandedTorqueHistory, &rwStackPtr, commandedTorque_mNm, rwDataLength);
}

void runFM() {
  // faultManagement will update fmState struct contents to reflect fault detected or not
  faultManagement(fmState, orderedRWSpeedHistory, orderedTimeStampHistory, rwDataLength, orderedCommandedTorqueHistory, orderedFineDeltaTheta, orderedCoarseDeltaTheta,
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
}

void getPrimaryFSReading() {
  double thetaDeg = ((finePixy1.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine);
  deltaThetaRadFine1 = thetaDeg*convertDegToRad;
  fsInjection(&deltaThetaRadFine1, fiState);
}

void getSecondaryFSReading() {
  deltaThetaRadFine2 = ((finePixy2.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;
}

void getCSReading() {
  double thetaDeg = ((coarsePixy.blocks[0].x)*convertPixToDegCoarse - centerOffsetDegCoarse) - 1.3;
  deltaThetaRadCoarse = thetaDeg*convertDegToRad;
}

void injectTimedRWFault() {
  // uncomment out these lines to inject a rw fault. DO NOT DELETE UNTIL GSU IS INTEGRATED
  if (millis() > 40000 && !fiState->cmdToFaultRW && millis() < 60000) {
    Serial.println("faulting rw");
    fiState->cmdToFaultRW = 1;
  }
  if (millis() > 60000 && fiState->cmdToFaultRW) {
    Serial.println("Unfaulting rw");
    fiState->cmdToFaultRW = 0;
    fmState->activeRW = 2;
  }
}

void injectTimedFSFault() {
  if (millis() > 20000 && !fiState->cmdToFaultFS && millis() < 80000) {
    Serial.println("faulting fs");
    fiState->cmdToFaultFS = 1;
  }
  // if (millis() > 100000 && fiState->cmdToFaultFS) {
  //   Serial.println("Unfaulting fs");
  //   fiState->cmdToFaultFS = 0;
  // }
}

