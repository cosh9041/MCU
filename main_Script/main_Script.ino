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

unsigned long timeLastReadPixy = 0;

void setup() 
{ 
  Serial.begin(9600);
  Serial.print("Starting...\n");
  finePixy1.init();
  finePixy2.init();
  coarsePixy.init();
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //Configure PWM
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

  initializeFaultManagementState(fmState);
  initializeFaultInjectState(fiState);
} 

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
float rwSpeedRad; 

// Physical characteristics
float const p1 = 0.0000335653965; // mNm/(rad/s), viscous friction 
float const p2 = 0.1303; // mNm, coloumb friction
float MOI = 1; //MOI of RW, stubbed out for now
float delta_omega = 15; // small delta near zero where we set torque to zero to simulate friction. TODO: Tune this value
uint16_t const lengthOfHistory = 1000;
// currentIndex points to the most recent data point. The last 100 data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
uint16_t currentIndex = 0;

//TODO: Determine units and ideal length for these. 
float commandedTorqueHistory[lengthOfHistory];
float reactionWheelSpeedHistory[lengthOfHistory];
float timeStampHistory[lengthOfHistory];
float orderedRWSpeedHistory[lengthOfHistory];
float orderedCommandedTorqueHistory[lengthOfHistory];
float orderedTimeStampHistory[lengthOfHistory];
float angularAccel[lengthOfHistory-1];
double fineDeltaTheta[lengthOfHistory];
double coarseDeltaTheta[lengthOfHistory];

// Utility function specifications. Implementations are below loop()
void sendTorque();
void getRWSpeed(float *rwSpeedRad, uint16_t analogReading);
void runFmAndControl();
void injectTimedRWFault();
void injectTimedFSFault();
void getPrimaryFSReading();
void getSecondaryFSReading();
void getCSReading();

void loop() {
  getRWSpeed(&rwSpeedRad, analogRead(A0));

  storeRWSpeed(reactionWheelSpeedHistory, timeStampHistory, currentIndex, rwSpeedRad, millis());

  // Stores ordered lists of last n = `lengthOfHistory` data points, with most recent being at index 0
  getOrderedHistory(reactionWheelSpeedHistory, orderedRWSpeedHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(timeStampHistory, orderedTimeStampHistory, lengthOfHistory, currentIndex);

  getAngularAcceleration(orderedRWSpeedHistory, orderedTimeStampHistory, angularAccel, lengthOfHistory);

  // NOTE: Pixy.getBlocks does a dirty nasty thing and returns 0 both when there are no blocks
  // detected AND when there is no information. This ambiguous case cost us a lot of wasted time
  if ((millis() - timeLastReadPixy) > 20) {
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
    timeLastReadPixy = millis();
  }

  if (fineBlocks1 || fineBlocks2) {
    deltaThetaRad = fmState->activeFS == 1 ? deltaThetaRadFine1 : deltaThetaRadFine2;

    // uncomment out these lines to inject a fs or rw fault. DO NOT DELETE UNTIL GSU IS INTEGRATED
    //injectTimedRWFault();
    //injectTimedFSFault();

    runFmAndControl();
  } else if(coarseBlocks) {
    Serial.println("using coarse blocks because no fine 1 or 2");
    deltaThetaRad = deltaThetaRadCoarse;
    runFmAndControl();
  } else {
    // If we do not pick up blocks set PWM to 50% to shut off motors
    pwm.pinDuty(PRIMARY_MOTOR_PIN, pwm_duty_inactive);
    pwm.pinDuty(REDUNDANT_MOTOR_PIN, pwm_duty_inactive);
  }
}

void runFmAndControl() {
  faultManagement(fmState, angularAccel, orderedCommandedTorqueHistory, fineDeltaTheta, coarseDeltaTheta,
	    lengthOfHistory, MOI);

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

  storeTorqueAndIncrementIndex(commandedTorqueHistory, &currentIndex, commandedTorque_mNm, lengthOfHistory);
}

void getRWSpeed(float *rwSpeedRad, uint16_t analogReading) {
  // hard coded in the '- 81' portion. This tunes down to 0 rads. most likely our system isn't perfect and is causing this. Not sure tho
  *rwSpeedRad = (28000/4096*analogReading - 14000)*2*PI/60 - 81;
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