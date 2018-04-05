#include <Servo.h> 
Servo myservo;

#include <faultManagement.h>
#include <rwInjection.h>
#include <fm_util.h>
#include <fi_util.h>
#include <data_util.h>
#include <SPI.h>  
#include <Pixy.h>
#include <PID_v1.h>
#include <DuePWM.h>
#include <stdio.h>
#include <inttypes.h>
#include <fsInjection.h>

#define PWM_FREQ1  6600
#define PWM_FREQ2  6600

DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

Pixy pixy;

//Define Variables we'll be connecting to
double deltaThetaRadFine1, deltaThetaRadFine2;
double deltaThetaRadCoarse1, deltaThetaRadCoarse2;
double Setpoint, deltaThetaRad, commandedTorque_mNm;

//Specify the links and initial tuning parameters
double const Kp=0.298334346525491, Ki=0.00116724851061565, Kd=13.4288733415698, N=0.155;
PID myPID(&deltaThetaRad, &commandedTorque_mNm, &Setpoint, Kp, Ki, Kd, N, DIRECT);

//Allocate for fm/fi state variables
FmState fmBase;
FmState *fmState = &fmBase;
FiState fiBase;
FiState *fiState = &fiBase;

void setup() 
{ 
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


  // Sets the resolution of the ADC for rw speed feedback to be 12 bits (4096 bins). 
  analogReadResolution(12);
  pinMode(A0, INPUT);

  //Enable Primary Motor Cycle for Initialization 
  digitalWrite(33,HIGH); //set ref of logic analyzer to 3.3V
  digitalWrite(39,LOW);  //drive redundant motor low (disabled)
  digitalWrite(29,HIGH); //drive primary motor high (enabled)
  delay(1000);
  digitalWrite(39,HIGH); //cycle redundant motor high (enabled)
  digitalWrite(29,LOW); //cycle primary motor low (disabled)
  delay(1000);
  
  digitalWrite(29,HIGH);//cycle primary motor high (enabled)

  initializeFaultManagementState(fmState);
  initializeFaultInjectState(fiState);
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

float rwSpeedRad; 
float const p1 = 0.0000335653965; // mNm/(rad/s), viscous friction 
float const p2 = 0.1303; // mNm, coloumb friction
float MOI = 1; //MOI of RW, stubbed out for now
float delta_omega = 50; // small delta near zero where we set torque to zero to simulate friction. TODO: Tune this value
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

float mockRWSpeed = 0.0;
float mockCommandTorque = 0.4;

void getRWSpeed(float *rwSpeedRad, uint16_t analogReading) {
  // hard coded in the '- 81' portion. This tunes down to 0 rads. most likely our system isn't perfect and is causing this. Not sure tho
  *rwSpeedRad = (28000/4096*analogReading - 14000)*2*PI/60 - 81;
}

void loop() 
{
  getRWSpeed(&rwSpeedRad, analogRead(A0));

  // Log RW speed.
  storeRWSpeed(reactionWheelSpeedHistory, timeStampHistory, currentIndex, rwSpeedRad, millis());

  // Stores ordered lists of last n = `lengthOfHistory` data points, with most recent being at index 0
  getOrderedHistory(reactionWheelSpeedHistory, orderedRWSpeedHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(commandedTorqueHistory, orderedCommandedTorqueHistory, lengthOfHistory, currentIndex);
  getOrderedHistory(timeStampHistory, orderedTimeStampHistory, lengthOfHistory, currentIndex);

  getAngularAcceleration(orderedRWSpeedHistory, orderedTimeStampHistory, angularAccel, lengthOfHistory);

  digitalWrite(46,LOW);

  blocks = pixy.getBlocks();// NOTE: TO run on board which is not pixy enabled, comment this line out

  if (blocks) {
    // uncomment out these lines to inject a rw fault
    // if (millis() > 30000 && !fmState->cmdToFaultRW && millis() < 60000) {
    //   Serial.println("faulting");
    //   fmState->cmdToFaultRW = 1;
    // }
    // if (millis() > 600000 && fmState->cmdToFaultRW) {
    //   Serial.println("Unfaulting");
    //   fmState->cmdToFaultRW = 0;
    // }

    deltaThetaRadFine1 = ((pixy.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;
    fsInjection(&deltaThetaRadFine1, fiState);
    deltaThetaRad = deltaThetaRadFine1;
    
    faultManagement(fmState, angularAccel, orderedCommandedTorqueHistory,
		    lengthOfHistory, MOI);

    // call to Compute assigns output to variable commandedTorque_mNm via pointers
    myPID.Compute(); 

    // TODO: Test injection strength and tune for delta_omega
    commandedTorque_mNm = injectRWFault(fmState, commandedTorque_mNm, rwSpeedRad, p1, p2, delta_omega);

    pwm_duty = (commandedTorque_mNm*15*mNm_to_mA*mA_to_duty + pwmOffset)*duty_to_bin;

    if (pwm_duty >230) {
      pwm_duty = 230;
    }
    else if (pwm_duty <26) {
      pwm_duty = 26; 
    }
    pwm_duty2 = round(pwm_duty);
    pwm_duty3 = (uint32_t) pwm_duty2;
    pwm.pinDuty( 6, pwm_duty3 );  // computed duty cycle on Pin 6
    
    storeTorqueAndIncrementIndex(commandedTorqueHistory, &currentIndex, commandedTorque_mNm, lengthOfHistory);
  } else {
    // Serial.println("No Blocks detected");
    // Serial.println(commandedTorque_mNm,5);
    // If we do not pick up blocks set PWM to 50% to shut off motors
    pwm_duty3 = 127;
    pwm.pinDuty( 6, pwm_duty3 );
  }
}
