#include "TRSensors.h"
#include "IRremote.h"

/* DEBUG MODE: 1 - enabled, 0 - disabled */
#define DEBUG 0

/* remote control */
#define RCV_PIN 4
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
int SampleTime = 10; //1 sec

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
        int output = kp * error + ki * errSum;//+ kd * dErr;
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
    SetTunings(12,25,3);

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
      if (results.value == 16724175) {
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
      if (results.value == 16718055) {
        if_start = false;
        if_stop = true;
      }
      irrecv.resume();
    }
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
