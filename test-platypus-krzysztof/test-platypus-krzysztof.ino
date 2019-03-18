#include "TRSensors.h"
#include "IRremote.h"

#define RCV_PIN 4
IRrecv irrecv(RCV_PIN);
decode_results results;

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

void setup() {
  irrecv.enableIRIn();
  pinMode(ML, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  if (!if_start) {
    // wait for IR control to start the race
    if (irrecv.decode(&results)) {
      if (results.value == 16724175) {
        if_start = true;
        if_stop = false;
        digitalWrite(13, HIGH);
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
        digitalWrite(13, LOW);
      }
      irrecv.resume();
    }
    
    // drive the car
    
  }
}
