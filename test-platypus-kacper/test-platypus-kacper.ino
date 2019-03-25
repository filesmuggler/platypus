#include "TRSensors.h"
#include "IRremote.h"

#define RCV_PIN 4
IRrecv irrecv(RCV_PIN);
decode_results results;

#define Kp 0;
#define Ki 0;
#define Kd 0;

#define MAX_SPEED 255;

#define ML 5
#define MR 6

#define ML_IN1 A1
#define ML_IN2 A0
#define MR_IN3 A2
#define MR_IN4 A3

#define NUM_SENSORS 5
TRSensors sensors = TRSensors();
unsigned int sensorsValues[NUM_SENSORS];

bool if_start = false;
bool if_stop = true;

long sensor_average;
int sensor_sum, position;
int integral, derivative;

void setup() {
  irrecv.enableIRIn();

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

  analogWrite(ML,0);
  analogWrite(MR,0);

  for(int i = 0; i<400;i++){
      sensors.calibrate();
  }

  
}


void loop() {
  if (!if_start) {
    // wait for IR control to start the race
    if (irrecv.decode(&results)) {
      if (results.value == 16724175) {
        if_start = true;
        if_stop = false;
      }
      irrecv.resume();
    }
  }

  else if (!if_stop) {
    // check for IR control to stop the car
    if (irrecv.decode(&results)) {
      if (results.value == 16718055) {
        if_start = false;
        if_stop = true;
      }
      irrecv.resume();
    }

    //Simple PID control
    unsigned int position = trs.readLine(sensorValues);

    int proportional = (int)position - 2000;
    integral = integral + proportional;
    derivative = proportional - last_proportional;
    int last_proportional = proportional;

    error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
    

    if (error_value < -MAX_SPEED) {
        error_value = -MAX_SPEED;
    }
    if (error_value > MAX_SPEED) {
        error_value = MAX_SPEED;
    }

    if (error_value < 0) {
      analogWrite(MR,MAX_SPEED + error_value);
      analogWrite(ML,MAX_SPEED);
    }
    else {
      analogWrite(ML,MAX_SPEED - error_value);
      analogWrite(MR,MAX_SPEED);
    }
    
  }
 
}
