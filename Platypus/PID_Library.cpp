/*
  MIT License
  ---
  Arduino Waveshare Alphabot Robot Project
  Program purpose: PID Control
  ---
  @author: Krzysztof Stezala
  ---
  Provided by CybAiR Science Club at 
  Institute of Control, Robotics and Information Engineering of
  Poznan University of Technology  
*/

#include "PID_Library.h"

// Constructor /////////////////////////////////////////////////////////////////
PID_Controller::PID_Controller()
{
  SampleTime = 0;
  Kp = 0;
  Ki = 0;
  Kd = 0;
  Input = 0;
  Output = 0;
  Setpoint = 0;
  lastTime = 0;
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available publicly
/* setSetPoint(int) sets the reference value for the PID controller */
void PID_Controller::setSetPoint(int setpoint)
{
  Setpoint = setpoint;
}

/* setInput(int,int) sets value of the input from sensors as percentage value */
void PID_Controller::setInput(int weight, int number)
{
  Input = (float)weight / (float)number;
}

/* setTunings(float,float,float) sets proper values to PID coefficients */
void PID_Controller::setTunings(float kp, float ki, float kd)
{
  float SampleTimeInSec = ((float)SampleTime) / 1000;
  Kp = kp;
  Ki = ki * SampleTimeInSec;
  Kd = kd / SampleTimeInSec;
}

/* setSampleTime(int) sets new sample time for PID controller */
void PID_Controller::setSampleTime(int newsampletime)
{
  if (newsampletime > 0)
  {
    float ratio = (float)newsampletime / (float)SampleTime;

    Ki *= ratio;
    Kd /= ratio;

    SampleTime = (unsigned long)newsampletime;
  }
}

/* Compute(void) calculates the output for speed regulation of motors */
int PID_Controller::Compute(void)
{
  unsigned long now = millis();

  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    //Compute error variables
    float error = Setpoint - Input;
    errSum += error;
    float dErr = (error - lastErr);

    //Compute PID Output
    Output = Kp * error + Ki * errSum + Kd * dErr;

    lastErr = error;
    lastTime = now;
  }
  return Output;
}
