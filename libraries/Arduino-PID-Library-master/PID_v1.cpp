/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, double N, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(-100000, 10000);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, N, POn);

    lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, double N, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, N, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
	   

	//timeChange = 100;
	//lastTimeChange = 100;
	double input      = *myInput;      // current input
	double error      = input - *mySetpoint; // current error
	double pState  = kp  * (error - preError); // proportional post gain  state
	double iState  = ki  * (error)*timeChange/1000; // integral post gain  state (deltaT already in ki)
	double dState  = ( kd * ( (error - preError)/timeChange*1000 - (preError - prePreError)/lastTimeChange*1000) - predState)*n; // derivative post gain  state (deltaT already in kd)
    //double dState  = kd * ( (error - preError)/timeChange*1000  - (preError - prePreError)/lastTimeChange*1000);
	double output;
	output = preTau + (pState + iState + dState);
	lastTimeChange = timeChange;

	prePreError = preError;
	preError    = error;
	preTau      = output;
    predState   = dState;

	//Print to arduino terminal
		// Serial.print(input,4);
		// Serial.print(",");
		// Serial.print(error,4);
		// Serial.print(",");
		// Serial.print(pState,4);
		// Serial.print(",");
		// Serial.print(iState,8);
		// Serial.print(",");
		// Serial.print(dState,8);
		// Serial.print(",");
		//Serial.print(output,4);
		//Serial.print("\n");



		//   outputSum+= (ki * error);
		
		//   /*Add Proportional on Measurement, if P_ON_M is specified*/
		//   if(!pOnE) outputSum-= kp * dInput;

		//   if(outputSum > outMax) outputSum= outMax;
		//   else if(outputSum < outMin) outputSum= outMin;

		//   /*Add Proportional on Error, if P_ON_E is specified*/
		   // double output;
		//   if(pOnE) output = kp * error;
		//   else output = 0;

		//   /*Compute Rest of PID Output*/
		//   output += outputSum - kd * dInput;

			if(output > outMax) output = outMax;
		  else if(output < outMin) output = outMin;
			*myOutput = output;

		  /*Remember some variables for next time*/
		  lastInput = input;
		  lastTime = now;
			return true;
	   }
	   else return false;
	}

	/* SetTunings(...)*************************************************************
	 * This function allows the controller's dynamic performance to be adjusted.
	 * it's called automatically from the constructor, but tunings can also
	 * be adjusted on the fly during normal operation
	 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, double N, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki;
   kd = Kd;
   n = N;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, double N){
    SetTunings(Kp, Ki, Kd, N, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   prePreError = 0;
   preError = 0;
   preOutput = 0;
   lastTimeChange = 0.1;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

