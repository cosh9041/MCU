#include <Servo.h> 
Servo myservo;

#include <faultManagement.h>
#include <rwInjection.h>
#include <SPI.h>  
#include <Pixy.h>
#include <PID_v1.h>
#include <DuePWM.h>
#include <stdio.h>
#include <inttypes.h>


#define PWM_FREQ1  6600
#define PWM_FREQ2  6600

DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

Pixy pixy;

//Define Variables we'll be connecting to
double deltaThetaRadFine1, deltaThetaRadFine2;
double deltaThetaRadCoarse1, deltaThetaRadCoarse2;
double Setpoint, deltaThetaRad, commandedTorque_mNm;

//Specify the links and initial tuning parameters
double const Kp=0.4193213777, Ki=0.003150323227, Kd=12.61957147, N=0.155;
PID myPID(&deltaThetaRad, &commandedTorque_mNm, &Setpoint, Kp, Ki, Kd, N, DIRECT);

void setup() 
{ 
  digitalWrite(27,LOW);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  //pixy.init();
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //Configure PWM
  pwm.setFreq1( PWM_FREQ1 );
  pwm.setFreq2( PWM_FREQ2 );

  pwm.pinFreq1( 6 );  // Pin 6 freq set to "pwm_freq1" on clock A
  pwm.pinFreq2( 7 );  // Pin 7 freq set to "pwm_freq2" on clock B

  //Select Initial Pixy
  digitalWrite(46,HIGH);

  //Enable Cycle
  digitalWrite(27,HIGH);
} 

//conversions for torques to PWM bins
double mNm_to_mA = 1000.0/8.4;
double mA_to_duty = 80.0/1000.0;
double duty_to_bin = 256.0/100.0;
double pwm_duty;
double pwmOffset = 50;
double pwm_duty2;
uint32_t pwm_duty3;
static int i = 0;
int k;
char buf[32];
uint16_t blocks;     
float const convertPixToDegCoarse = 0.2748;
float const convertPixToDegFine = 0.0522;
float const centerOffsetDegCoarse = 160*convertPixToDegCoarse;
float const convertDegToRad = 3.1415926535897932384626433832795/180;

// Fault status "bits" as uint8_t since c doesn't support bools (c++ does, but not c)
uint8_t isPrimaryRWactive; 
uint8_t isPrimaryFSActive;
uint8_t cmdToFaultRW; // 0 if no command to fault, otherwise 1. Should be set only by comms
uint8_t isFaulted;
uint8_t isRecovering;
uint8_t faultType; // 0 if no fault, 1 if fine sensor fault, 2 if coarse sensor fault
uint8_t cmdToRecover;
uint8_t faultTimerActive;
float rwSpeedRad; 
float const p1 = 10.0; // TODO: p1 and p2 need to be set via data from Dalton. These are just placeholders
float const p2 = 10.0; 
float delta_omega; // small delta near zero where we set torque to zero to simulate friction. TODO: Tune this value
uint16_t const lengthOfHistory = 100;
// currentIndex points to the most recent data point. The last 100 data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
uint16_t currentIndex = 0;

//TODO: Determine units and ideal length for these. commanded
float commandedTorqueHistory[lengthOfHistory];
float reactionWheelSpeedHistory[lengthOfHistory];
float timeStampHistory[lengthOfHistory];

void storeRWSpeed(uint16_t index, float rwSpeed, float timeStamp) {
  reactionWheelSpeedHistory[index] = rwSpeed;
  timeStampHistory[index] = timeStamp;
}

void storeTorqueAndIncrementIndex(uint16_t *index, float commandedTorque_mNm) {
  *index += 1;
  if (*index == lengthOfHistory) *index = 0;
  commandedTorqueHistory[*index] = commandedTorque_mNm;
}

// Stores the last n = `length` data points from `data` in a destination array in a time ordered fashion
// currentIndex points to the most recent data point. The last n = `length` data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
void getHistory(float *data, float *destination, uint16_t length, uint16_t index) {
  uint16_t historyIndex = 0;
  for (int32_t i = index; i >= 0; i--) {
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
  }
}

float mockRWSpeed = 0.0;
float mockTimeStamp = 0.0;
float mockCommandTorque = 0.4;

void loop() 
{
  storeRWSpeed(currentIndex, mockRWSpeed, mockTimeStamp);

  float orderedRWSpeedHistory[lengthOfHistory];
  float orderedCommandedTorqueHistory[lengthOfHistory];
  float orderedTimeStampHistory[lengthOfHistory];
  float angularAccel[lengthOfHistory-1];

  // Stores ordered lists of last n = `lengthOfHistory` data points, with most recent being at index 0
  getHistory(reactionWheelSpeedHistory, orderedRWSpeedHistory, lengthOfHistory, currentIndex);
  getHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, lengthOfHistory, currentIndex);
  getHistory(timeStampHistory, orderedTimeStampHistory, lengthOfHistory, currentIndex);

  getAngularAcceleration(orderedRWSpeedHistory, orderedTimeStampHistory, angularAccel, lengthOfHistory);

  if (currentIndex%10 == 0) {
    for (int i = 0; i < lengthOfHistory-1; i++) {
      Serial.print(angularAccel[i]);
      Serial.print("\n");
    }
    delay(5000);
  }

  storeTorqueAndIncrementIndex(&currentIndex, mockCommandTorque);
  // TODO: Actually get these values and replace mocks in function calls
  mockCommandTorque += 0.4;
  mockRWSpeed += 0.1;
  mockTimeStamp += 0.01;

  digitalWrite(46,LOW);
  //blocks = pixy.getBlocks();
  
  if (blocks)
  {
    deltaThetaRadFine1 = ((pixy.blocks[0].x)*convertPixToDegFine - centerOffsetDegCoarse)*convertDegToRad;
    deltaThetaRad = deltaThetaRadFine1;

    // TODO: fault timer
    faultManagement(&isFaulted, &isRecovering, &faultType, &cmdToRecover, &faultTimerActive,
      &isPrimaryRWactive, &isPrimaryFSActive);

    // call to Compute assigns output to variable commandedTorque_mNm via pointers
    myPID.Compute(); 

    // TODO: Test injection strength and tune for delta_omega
    commandedTorque_mNm = injectRWFault(isPrimaryRWactive, cmdToFaultRW, commandedTorque_mNm, 
      rwSpeedRad, p1, p2, delta_omega);

    pwm_duty = (commandedTorque_mNm*15*mNm_to_mA*mA_to_duty + pwmOffset)*duty_to_bin;

    if (pwm_duty >230){
      pwm_duty = 230;
    }
    else if (pwm_duty <26){
      pwm_duty = 26; 
    }
    pwm_duty2 = round(pwm_duty);
    pwm_duty3 = (uint32_t) pwm_duty2;
    digitalWrite(27,HIGH);
    pwm.pinDuty( 6, pwm_duty3 );  // computed duty cycle on Pin 6
    
    // TODO: Do we need this printing stuff?
    Serial.print(commandedTorque_mNm,5);
    Serial.print(",");
    Serial.print(pwm_duty,5);
    Serial.print(",");
    Serial.print(pwm_duty2,5);
    Serial.print(",");
    Serial.print(pwm_duty3);
    Serial.print("\n");
    
    // TODO: Do we need these lines?
      //digitalWrite(27,LOW);
//      
//      //control servo (if using)
////    //myservo.write(xOut*170/320);
////    
////    
    i++;
    //Serial print of Pixy info
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%1==0) 
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
    // TODO: Do we need these lines?
//    for (k=0; k<blocks; k++)
//      {
//        sprintf(buf, "  block %d: ", k);
//        Serial.print(buf); 
//        pixy.blocks[k].print();
    }
//    }
  }  
}
