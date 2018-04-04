#include <Servo.h> 
Servo myservo;

#include <faultManagement.h>
#include <rwInjection.h>
#include <fm_util.h>
#include <data_util.h>
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

FmState base = {.isFaulted = 0};
FmState *fmState = &base;

void setup() 
{ 
  digitalWrite(27,LOW);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
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

  initializeFaultState(fmState);

  // Sets the resolution of the ADC to be 12 bits (4096 bins)
  analogReadResolution(12);
  pinMode(A0, INPUT);
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

// Pixy conversions
double const convertPixToDegCoarse = 0.2748;
double const convertPixToDegFine = 0.0522;
double const centerOffsetDegFine = 160*convertPixToDegFine;
double const convertDegToRad = 3.1415926535897932384626433832795/180; 

// Reaction Wheel speed conversions


// Reaction wheel speed variable
uint16_t rwSpeedBin;
double RWspeed;

// Fault status "bits" as uint8_t since c doesn't support bools (c++ does, but not c)
uint8_t isPrimaryRWActive; 
uint8_t isPrimaryFSActive;
uint8_t cmdToFaultRW; // 0 if no command to fault, otherwise 1. Should be set only by comms
uint8_t isFaulted;
uint8_t isRecovering;
uint8_t faultType; // 0 if no fault, 1 if fine sensor fault, 2 if coarse sensor fault
uint8_t cmdToRecover;
uint8_t faultTimerActive;
float const p1 = 10.0; // TODO: p1 and p2 need to be set via data from Dalton. These are just placeholders
float const p2 = 10.0; 
float delta_omega; // small delta near zero where we set torque to zero to simulate friction. TODO: Tune this value
uint16_t const lengthOfHistory = 1000;
// currentIndex points to the most recent data point. The last 100 data points are accumulated "behind"
// this index. For example, if the currentIndex is 55, the order (in terms of recency) of the stored indicies
// is 55....0...99...56. I.e. the most recently stored index is 55 (just stored), and the oldest stored index is 56.
uint16_t currentIndex = 0;
float MOI = 1; //MOI of RW, stubbed out for now

//TODO: Determine units and ideal length for these. 
float commandedTorqueHistory[lengthOfHistory];
float reactionWheelSpeedHistory[lengthOfHistory];
float timeStampHistory[lengthOfHistory];
float orderedRWSpeedHistory[lengthOfHistory];
float orderedCommandedTorqueHistory[lengthOfHistory];
float orderedTimeStampHistory[lengthOfHistory];
float angularAccel[lengthOfHistory-1];

float mockRWSpeed = 0.0;
float mockTimeStamp = 0.0;
float mockCommandTorque = 0.4;

//double Tstart, Tstop, a;
void loop() 
{
  //Tstart = millis();
  // Example of pulling rw speed from motor analog output
  rwSpeedBin = analogRead(A0);
  // hard code in the '- 81' portion. This tunes down to 0 rads. most likely our system isn't perfect and is causing this. Not sure tho
  RWspeed = (28000/4096*rwSpeedBin - 14000)*2*PI/60 - 81;
  //sprintf('Rw speed bin is\n');
  Serial.println(RWspeed);

  // Log RW speed.
  storeRWSpeed(reactionWheelSpeedHistory, timeStampHistory, currentIndex, mockRWSpeed, mockTimeStamp);


  // Stores ordered lists of last n = `lengthOfHistory` data points, with most recent being at index 0
  getOrderedHistory(reactionWheelSpeedHistory, orderedRWSpeedHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(timeStampHistory, orderedTimeStampHistory, lengthOfHistory, currentIndex);

  getAngularAcceleration(orderedRWSpeedHistory, orderedTimeStampHistory, angularAccel, lengthOfHistory);

  storeTorqueAndIncrementIndex(commandedTorqueHistory, &currentIndex, mockCommandTorque, lengthOfHistory);

  // TODO: Actually get these values and replace mocks in function calls. commandedTorque will come from 
  // control law. rwSpeed comes from encoder on RW, timeStamp comes from ???
  mockCommandTorque += 0.4;
  mockRWSpeed += 0.1;
  mockTimeStamp += 0.01;

  digitalWrite(46,LOW);

  blocks = pixy.getBlocks();// NOTE: TO run on board which is not pixy enabled, comment this line out

  if (blocks)
  {
    deltaThetaRadFine1 = ((pixy.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;
    deltaThetaRad = deltaThetaRadFine1;

    faultManagement(fmState, angularAccel, orderedCommandedTorqueHistory,
		    lengthOfHistory, MOI);

    // call to Compute assigns output to variable commandedTorque_mNm via pointers
    myPID.Compute(); 

    // TODO: Test injection strength and tune for delta_omega
    commandedTorque_mNm = injectRWFault(isPrimaryRWActive, cmdToFaultRW, commandedTorque_mNm, 
      RWspeed, p1, p2, delta_omega);

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
    //Serial.print(commandedTorque_mNm,5);
    //Serial.print(",");
    //Serial.print(pwm_duty,5);
    //Serial.print(",");
    //Serial.print(pwm_duty2,5);
    //Serial.print(",");
    //Serial.print(pwm_duty3);
    //Serial.print("\n");
    
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
      //sprintf(buf, "Detected %d:\n", blocks);
      //Serial.print(buf);
    // TODO: Do we need these lines?
//    for (k=0; k<blocks; k++)
//      {
//        sprintf(buf, "  block %d: ", k);
//        Serial.print(buf); 
//        pixy.blocks[k].print();
    }
//    }
  }  
  //Tstop = millis();
  //a = Tstop - Tstart;
  //Serial.print(a);
}
