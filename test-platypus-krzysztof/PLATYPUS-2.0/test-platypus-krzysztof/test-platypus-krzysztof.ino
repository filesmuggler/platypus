#include "TRSensors.h"
#include "IRremote.h"

//HEHE NOOBZ

/* ir remote control */
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

unsigned int position;

void setup() {

  Serial.begin(9600);
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
      Serial.print("Calibrating...");
      Serial.println(i/400*100);
  }

  
  delay(1000);  
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
    Stop();
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
    Drive();    
  }
}

void Stop(){
  analogWrite(ML,0);
  analogWrite(MR,0);
}

void Drive(){
  //read sensors
  position = sensors.readLine(sensorsValues);
  for (unsigned char i = 1; i < NUM_SENSORS - 1; i++)
  {
    Serial.print(sensorsValues[i]);
    Serial.print('\t');
  }

  Serial.println(position); // comment this line out if you are using raw values
  
  
  //PID control
  // The "proportional" term should be 0 when we are on the line.
  int proportional = 2000 - (int)position;
  // improve performance.
  int power_difference = proportional/15; //+derivative;  

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int maximum =100;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < - maximum)
    power_difference = - maximum;
//    Serial.println(power_difference);
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
  //encoder learning the track
  //

  
}
