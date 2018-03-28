#include <Servo.h> 
Servo myservo;

#include <SPI.h>  
#include <Pixy.h>
#include <PID_v1.h>

Pixy pixy;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.4193213777, Ki=0.003150323227, Kd=12.61957147, N=0.155;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,N, DIRECT);

void setup() 
{ 

  Serial.begin(9600);
  Serial.print("Starting...\n");
  myservo.attach(9);
  pinMode(9,OUTPUT);
  pixy.init();
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  
} 

double j = 170;
static int i = 0;
int k;
uint16_t blocks;
char buf[32];


//TESTING DATA COMMENT OUT IF USING PIXY
double vec[] = {0,
    0.0007,
    0.0027,
    0.0060,
    0.0106,
    0.0166,
    0.0239,
    0.0325,
    0.0425,
    0.0537,
    0.0662,
    0.0801,
    0.0952,
    0.1116,
    0.1293,
    0.1482,
    0.1685,
    0.1899,
    0.2127,
    0.2366,
    0.2618,
    0.2882,
    0.3158,
    0.3446,
    0.3747,
    0.4058,
    0.4382,
    0.4717,
    0.5064,
    0.5422,
    0.5792,
    0.6172,
    0.6564,
    0.6966,
    0.7379,
    0.7803,
    0.8237,
    0.8682,
    0.9137,
    0.9602,
    1.0077,
    1.0562,
    1.1057,
    1.1561,
    1.2075,
    1.2598,
    1.3130,
    1.3671,
    1.4221,
    1.4779,
   58.8304,
   58.8783,
   58.9080,
   58.9197,
   58.9136,
   58.8901,
   58.8494,
   58.7917,
   58.7174,
   58.6266,
   58.5197,
   58.3969,
   58.2586,
   58.1048,
   57.9361,
   57.7525,
   57.5544,
   57.3421,
   57.1159,
   56.8759,
   56.6226,
   56.3561,
   56.0768,
   55.7849,
   55.4807,
   55.1645,
   54.8365,
   54.4971,
   54.1465,
   53.7850,
   53.4129,
   53.0304,
   52.6379,
   52.2355,
   51.8236,
   51.4025,
   50.9723,
   50.5335,
   50.0862,
   49.6308,
   49.1674,
   48.6964,
   48.2180,
   47.7325,
   47.2402,
   46.7413,
   46.2361,
   45.7247,
   45.2076,
   44.6850,
   44.1570,
   43.6240,
   43.0861,
   42.5437,
   41.9970,
   41.4463,
   40.8917,
   40.3335,
   39.7720,
   39.2073,
   38.6398,
   38.0696,
   37.4970,
   36.9222,
   36.3454,
   35.7669,
   35.1868,
   34.6054,
   34.0228,
   33.4394,
   32.8553,
   32.2706,
   31.6857,
   31.1007,
   30.5158,
   29.9312,
   29.3471,
   28.7637,
   28.1812,
   27.5997,
   27.0194,
   26.4405,
   25.8632,
   25.2877,
   24.7140,
   24.1425,
   23.5732,
   23.0063,
   22.4420,
   21.8803,
   21.3216,
   20.7658,
   20.2133,
   19.6640,
   19.1181,
   18.5759,
   18.0373,
   17.5025,
   16.9718,
   16.4451,
   15.9226,
   15.4044,
   14.8907,
   14.3815,
   13.8770,
   13.3772,
   12.8823,
   12.3924,
   11.9075,
   11.4278,
   10.9533,
   10.4842,
   10.0204,
    9.5622,
    9.1096,
    8.6626,
    8.2213,
    7.7858,
    7.3563,
    6.9326,
    6.5149,
    6.1033,
    5.6978,
    5.2985,
    4.9054,
    4.5185,
    4.1380,
    3.7638,
    3.3961,
    3.0347,
    2.6798,
    2.3315,
    1.9896,
    1.6544,
    1.3257,
    1.0036,
    0.6882,
    0.3794,
    0.0772,
   -0.2182,
   -0.5069,
   -0.7890,
   -1.0643,
   -1.3330,
   -1.5949,
   -1.8501,
   -2.0985,
   -2.3403,
   -2.5754,
   -2.8038,
   -3.0254,
   -3.2404,
   -3.4488,
   -3.6505,
   -3.8456,
   -4.0340,
   -4.2159,
   -4.3912,
   -4.5600,
   -4.7223,
   -4.8781,
   -5.0274,
   -5.1703,
   -5.3069,
   -5.4370,
   -5.5609,
   -5.6785,
   -5.7898,
   -5.8950,
   -5.9940,
   -6.0868,
   -6.1736,
   -6.2544,
   -6.3293,
   -6.3981,
   -6.4612,
   -6.5184,
   -6.5698,
   -6.6155,
   -6.6555,
   -6.6900,
   -6.7189,
   -6.7422,
   -6.7602,
   -6.7728,
   -6.7801,
   -6.7821,
   -6.7789,
   -6.7706,
   -6.7572,
   -6.7389,
   -6.7156,
   -6.6874,
   -6.6544,
   -6.6167,
   -6.5743,
   -6.5273,
   -6.4758
   -6.4199,
   -6.3595};

void loop() 
{
  //TESTING LOOP COMMENT OUT IF USING PIXY
 for (int i=0;i<249;i++)
 {
      Input = vec[i]*3.1415926535897932384626433832795/180;
      myPID.Compute();
      //Serial.print(Input);
      //Serial.print("\n");
//      Serial.print(Output,5);
//      Serial.print("\n");
      delay(100);
      
 }
 delay(100000);
  //END TESING LOOP

  //BEGIN PIXY INTEGRATION
           
 // blocks = pixy.getBlocks();    // grab blocks!
  
  // If there are detect blocks, print them!
//  if (blocks)
//  {
      //Get input and adjust to degrees. Note that the 0.0398 value is for the fine lens
//    Input = (pixy.blocks[0].x)*0.0398;
      //Set center to 0
//    Input -= 160*0.0398;
      // Convert to radians 
//    Input *= 3.1415926535897932384626433832795/180;
      // Use PID LAW
//    myPID.Compute();
      //control servo (if using)
//    //myservo.write(xOut*170/320);
//    
//    
//    i++;
//    //Serial print of Pixy info
//    // do this (print) every 50 frames because printing every
//    // frame would bog down the Arduino
//    if (i%50==0)
//    {
//    Serial.print(Input);
//    Serial.print("\n");
//    Serial.print(Output);
//    Serial.print("\n");  
//      sprintf(buf, "Detected %d:\n", blocks);
//      Serial.print(buf);
//      for (k=0; k<blocks; k++)
//      {
//        sprintf(buf, "  block %d: ", j);
//        Serial.print(buf); 
//        pixy.blocks[k].print();
//      }
//    }
//  }  




  
  
}
