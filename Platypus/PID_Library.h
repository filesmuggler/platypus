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
#ifndef PID_LIBRARY
#define PID_LIBRARY

#include "Arduino.h"

class PID_Controller
{
    public:
        PID_Controller();
        void setSetPoint(int);
        void setInput(int,int);
        void setTunings(float kp,float ki,float kd);
        void setSampleTime(int);
        int Compute(void);

    private:
        unsigned long lastTime;
        int Output, Setpoint;
        float Input;
        double errSum, lastErr;
        double Kp, Ki, Kd;
        int SampleTime;
};


#endif