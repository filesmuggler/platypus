#include "TRSensors.h"
#include "IRremote.h"

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


    /* sensors calibration */
    for (int i = 0; i < 200; i++) {
        sensors.calibrate();
        /* DEBUG ONLY */
        /* display progress */
        Serial.print("Calibrating...");
        Serial.print((float)i / 200.0 * 100);
        Serial.println("% complete.");
    }
    Serial.println("Calibration finished.");
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
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorsValues[i]);
        Serial.print('\t');
    }
    Serial.println(position); 
    
}

void Stop(){
    /* stop all motors */
    analogWrite(ML, 0);
    analogWrite(MR, 0);
}

void Compute(){

}

void SetTunings(){

}