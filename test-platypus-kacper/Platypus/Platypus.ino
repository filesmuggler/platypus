/*
    IR codes for rc device:

    0 - 16738455
    1 - 16724175
    2 - 16718055
    3 - 16743045
    4 - 16716015
    5 - 16726215
    6 - 16734885
    7 - 16728765
    8 - 16730805
    9 - 16732845
    100+ - 16750695
    200+ - 16756815
    - - 16769055
    + - 16754775
    eq - 16748655
 */

#include "TRSensors.h"
#include "IRremote.h"

/* DEBUG MODE: 1 - enabled, 0 - disabled */
#define DEBUG 0

/* remote control */
#define RCV_PIN 4
#define BUTTON_0 16738455
#define BUTTON_1 16724175
#define BUTTON_2 16718055
#define BUTTON_3 16743045
#define BUTTON_4 16716015
#define BUTTON_5 16726215
#define BUTTON_6 16734885
#define BUTTON_7 16728765
#define BUTTON_8 16730805
#define BUTTON_9 16732845
#define BUTTON_100 16750695
#define BUTTON_200 16756815
#define BUTTON_MINUS 16769055
#define BUTTON_PLUS 16754775
#define BUTTON_EQ 16748655
IRrecv irrecv(RCV_PIN);
decode_results results;

/* motors */
#define ML 5
#define MR 6
#define ML_IN1 A1
#define ML_IN2 A0
#define MR_IN3 A2
#define MR_IN4 A3

/* sensors */
#define NUM_SENSORS 5
TRSensors sensors = TRSensors();
unsigned int sensorsValues[NUM_SENSORS];

/* global variables for robot start-stop control */
bool if_start = false;
bool if_stop = true;

/* PID parameters */
int position; // position obtained from sensors (2000 is neutral)
unsigned long lastTime;
int Setpoint;
int lastOutput;
double errSum, lastErr;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
String params_set;
    

const int maximum = 150;


int Compute(int input){
    unsigned long now = millis();
    int timeChange = (now - lastTime);
    if(timeChange >= SampleTime){
        /*Compute all the working error variables*/
        double error = Setpoint - input; //P part
        errSum += error;                 //I part
        double dErr = (error - lastErr); //D part

        /* Compute PID output */
        int output = kp * error;// + ki * errSum + kd * dErr;
        //int output = kp * error;
        /* save some values for next time */
        lastErr = error;
        lastTime = now;
        lastOutput = output;
    }
    return lastOutput;
}

void SetTunings(double Kp, double Ki, double Kd){
    double SampleTimeInSec = ((double)SampleTime)/1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime){
    if(NewSampleTime > 0){
        double ratio = (double)NewSampleTime/(double)SampleTime;

        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void setup(){
    Serial.begin(9600);

    /* IR RC enabled */
    irrecv.enableIRIn();

    /* setting up motors */
    pinMode(ML, OUTPUT);
    pinMode(MR, OUTPUT);
    pinMode(ML_IN1, OUTPUT);
    pinMode(ML_IN2, OUTPUT);
    pinMode(MR_IN3, OUTPUT);
    pinMode(MR_IN4, OUTPUT);
    digitalWrite(ML_IN1, HIGH);
    digitalWrite(ML_IN2, LOW);
    digitalWrite(MR_IN3, HIGH);
    digitalWrite(MR_IN4, LOW);
    analogWrite(ML, 0);
    analogWrite(MR, 0);

    /* Setting PID params */
    Setpoint = 2000;
    SetSampleTime(10);
    SetTunings(12,7,0);

    /* sensors calibration */
    for (int i = 0; i < 200; i++) {
        sensors.calibrate();
        /* DEBUG ONLY */
        /* display progress */
        if(DEBUG){
            Serial.print("Calibrating...");
            Serial.print((float)i / 200.0 * 100);
            Serial.println("% complete.");
        }
        
    }
    /* DEBUG ONLY */
    if(DEBUG){
        Serial.println("Calibration finished.");
    }
    
    delay(1000);
}

void loop(){
    if (!if_start) {
    /* wait for IR control to start the race */
    if (irrecv.decode(&results)) {
      if (results.value == BUTTON_1) {
        if_start = true;
        if_stop = false;
      }
      irrecv.resume();
    }
    
    Stop();
  }
  else if (!if_stop) {
    /* check for IR control to stop the car */
    if (irrecv.decode(&results)) {
      if (results.value == BUTTON_2) {
        if_start = false;
        if_stop = true;
      }
      else if(results.value == BUTTON_4){
          //increase kp
          kp += 1;
      }
      else if(results.value == BUTTON_5){
          //decrease kp
          kp -= 1;
      }
      else if(results.value == BUTTON_7){
          // increase ki
          ki += 1;
      }
      else if(results.value == BUTTON_8){
          // decrease ki
          ki -= 1;
      }
      else if(results.value == BUTTON_EQ){
          // reset kp,ki,kd
          SetTunings(0,0,0);
      }
      irrecv.resume();
    }

    params_set = String(kp) + " " + String(ki) + " " + String(kd) + "\n";
    Serial.print(params_set);
    Drive();
    
  }
}

void Drive(){
    /* read sensors */
    position = sensors.readLine(sensorsValues);

    /* DEBUG ONLY */
    /* display sensors values */
    if(DEBUG){
        for (unsigned char i = 0; i < NUM_SENSORS; i++)
        {
            Serial.print(sensorsValues[i]);
            Serial.print('\t');
        }
        Serial.println(position); 
    }
    

    int power_difference = Compute(position);
    if (power_difference > maximum)
        power_difference = maximum;
    if (power_difference < - maximum)
        power_difference = - maximum;
    if (power_difference < 0)
    {
        analogWrite(ML,maximum + power_difference);
        analogWrite(MR,maximum);
    }
    else
    {
        analogWrite(ML,maximum);
        analogWrite(MR,maximum - power_difference);
    }

}

void Stop(){
    /* stop all motors */
    analogWrite(ML, 0);
    analogWrite(MR, 0);
}
