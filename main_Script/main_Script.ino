#include <Servo.h> 
Servo myservo;

#include <faultManagement.h>
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
double Setpoint, deltaThetaRad, commandedTorque_mNm;

//Specify the links and initial tuning parameters
double Kp=0.4193213777, Ki=0.003150323227, Kd=12.61957147, N=0.155;
PID myPID(&deltaThetaRad, &commandedTorque_mNm, &Setpoint, Kp, Ki, Kd,N, DIRECT);

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

  //Select Inital Pixy
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
double const convertPixToDegCoarse = 0.2748;
double const convertPixToDegFine = 0.0522;
double const centerOffsetDegFine = 160*convertPixToDegFine;
double const convertDegToRad = 3.1415926535897932384626433832795/180;

void loop() 
{
  digitalWrite(46,LOW);
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    deltaThetaRad = ((pixy.blocks[0].x)*convertPixToDegFine - centerOffsetDegFine)*convertDegToRad;

    // call to Compute assigns output to variable commandedTorque_mNm via pointers
    myPID.Compute(); 

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
