/*
  MIT License
  ---
  Arduino Waveshare Alphabot Robot Project
  Program purpose: Main program
  ---
  @author: Krzysztof Stezala
  ---
  Provided by CybAiR Science Club at 
  Institute of Control, Robotics and Information Engineering of
  Poznan University of Technology  
*/

#include "PID_Library.h"

PID_Controller pid_c;

void setup(){

    pid_c.setTunings(1.0,1.0,1.0);
    pid_c.Compute();
    //create PID controller
    //create array of sensors
    //initialize pins
    //in case of auto control: delay after power up
    //in case of remote control: wait for comm
}

void loop(){
    //read analog values
    //sort values
    //compute PID algorithm
    //correct path and speed
}
