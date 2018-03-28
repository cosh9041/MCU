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
#define PWM_FREQ2  6000

DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

Pixy pixy;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.4193213777, Ki=0.003150323227, Kd=12.61957147, N=0.155;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,N, DIRECT);

void setup() 
{ 

  digitalWrite(27,LOW);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  myservo.attach(9);
  pinMode(9,OUTPUT);
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

double j = 170;
double mNm_to_mA = 1000.0/8.4;
double mA_to_duty = 80.0/1800.0;
double pwm_duty;
double offset = 50;
double pwm_duty2;
uint32_t pwm_duty3;
static int i = 0;
int k;
char buf[32];
uint16_t blocks;     
double conversion;  

void loop() 
{
//BEGIN PIXY INTEGRATION

  digitalWrite(47,LOW);    
  blocks = pixy.getBlocks();    // grab blocks!
  
  // If there are detect blocks, print them!
  if (blocks)
  {
      //0.28 for coarse 0.00398 for fine;
      conversion = 0.2748;//0.2748 for coarse 0.0398 for fine;
      //Get input and adjust to degrees. Note that the 0.0398 value is for the fine lens
    Input = (pixy.blocks[0].x)*conversion;
//      //Set center to 0
    Input -= 160*conversion;
//      // Convert to radians 
    Input *= 3.1415926535897932384626433832795/180;
      // Use PID LAW
       myPID.Compute();
       pwm_duty = (Output*30*mNm_to_mA*mA_to_duty+offset)/100.0*256.0;
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
      Serial.print(Output,5);
      Serial.print(",");
      Serial.print(pwm_duty,5);
      Serial.print(",");
      Serial.print(pwm_duty2,5);
      Serial.print(",");
      Serial.print(pwm_duty3);
      Serial.print("\n");
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
//    for (k=0; k<blocks; k++)
//      {
//        sprintf(buf, "  block %d: ", j);
//        Serial.print(buf); 
//        pixy.blocks[k].print();
      }
//    }
  }  




  
  
}
