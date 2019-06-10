/*
    MIT License 2019
    ---
    Alphabot Linefollower Arduino Robot with PID control
    version: 1.0
    Purpose: Linefollower programmed for PID understanding
    File: Platypus.ino (main file)
    ---
    @author: Krzysztof Stezala & Kacper Karczewski
    ---
    Provided by CybAiR Science Club at
    Institute of Control, Robotics and Information Engineering of
    Poznan University of Technology
*/

#include "src/TRSensors/TRSensors.h"

/* Bluetooth variables */
char bt_received_state;
String receivedData = "";
unsigned long previous_millis = 0;
short interval = 10;
int handy_counter = 0;
unsigned long current_millis;

/*  
    Motors pin definitions:
        ML - enable left motor
        MR - enable right motor
        ML_IN1, ML_IN2 - rotation direction of the left motor
        ML_IN3, ML_IN4 - rotation direction of the right motor
 */
#define ML 5
#define MR 6
#define ML_IN1 A1
#define ML_IN2 A0
#define MR_IN3 A2
#define MR_IN4 A3

/* Alphabot sensors bar */
// define how many sensors to use
#define NUM_SENSORS 5
TRSensors sensors = TRSensors();
// hold values for sensors
unsigned int sensorsValues[NUM_SENSORS];

/* Global variables for robot start-stop control */
bool if_start = false;
bool if_stop = true;

/* PID parameters */
unsigned int position; // position obtained from sensors (2000 is neutral)
unsigned long lastTime = 0;
int Setpoint;
int lastOutput = 100;
double errSum = 0;
double lastErr = 0;
double kp, ki, kd;
double kp_r, ki_r, kd_r;
int SampleTime = 1000; //1 sec

/* Optimal default speed */
const int maximum = 150;

void setup()
{
    /* Setting up bluetooth */
    Serial.begin(9600);

    /* Setting up motors */
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

    /* Setting initial PID params */
    Setpoint = 2000;
    SetSampleTime(1);

    /* Sensors calibration */
    for (int i = 0; i < 400; i++)
    {
        sensors.calibrate();
    }

    /* Reading Bluetooth for PID paramteres for particular run */
    while (!if_start)
    {
        // read data until BT is available
        while (Serial.available())
        {
            // look up if communication is not too fast
            current_millis = millis();
            if (current_millis - previous_millis > interval)
            {
                // read data
                previous_millis = current_millis;
                bt_received_state = Serial.read();
                // look for data and delimiters
                if (bt_received_state == '|')
                {
                    // convert received state to int
                    int temp_data = receivedData.toInt();
                    // assign k's
                    if (handy_counter == 0)
                    {
                        kp_r = temp_data;
                        receivedData = "";
                        handy_counter++;
                    }
                    else if (handy_counter == 1)
                    {
                        ki_r = temp_data;
                        receivedData = "";
                        handy_counter++;
                    }
                    else if (handy_counter == 2)
                    {
                        kd_r = temp_data;
                        receivedData = "";
                        SetTunings(kp_r, ki_r, kd_r);

                        if (kp_r == 0 && ki_r == 0 && kd_r == 0)
                        {
                            // stop
                            if_start = false;
                            if_stop = true;
                        }
                        else
                        {
                            // start
                            if_start = true;
                            if_stop = false;
                        }
                        handy_counter = 0;
                    }
                }
                else
                {
                    receivedData += bt_received_state;
                }
            }
        }
    }
    // safety delay, why not
    delay(1000);
}

/* main loop */
void loop()
{
    Drive();
}

/* Computing output for motors */
int Compute(unsigned int input)
{
    unsigned long now = millis();
    int timeChange = (now - lastTime);
    if (timeChange >= SampleTime)
    {
        /* Compute all the working error variables */
        // calculating error
        double error = Setpoint - input; 
        
        // I part
        if (lastOutput >= maximum && error > 0)
        {
            // preventing wind-up when I-term is too high
            errSum = 0;
        }
        else if (lastOutput <= -maximum && error < 0)
        {
            // preventing wind-up when I-term is too low
            errSum = 0;
        }
        else
        {
            // increasing I-term
            errSum += error * (double)timeChange / 1000.0;
        }

        // D part
        double dErr = (error - lastErr); 

        /* Compute PID output */
        int output = kp * error + ki * errSum + kd * dErr;
        
        /* save some values for next time */
        lastErr = error;
        lastTime = now;
        lastOutput = output;
    }
    return lastOutput;
}

/* Setting kp, ki, kd params */
void SetTunings(double Kp, double Ki, double Kd)
{
    double SampleTimeInSec = ((double)SampleTime) / 1000.0;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

/* Adjusting sampling time */
void SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)SampleTime;

        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

/* Go baby go */
void Drive()
{
    // read sensors
    position = sensors.readLine(sensorsValues);

    
    if (position >= 1900 && position <= 2100)
    {
        /* Boost on straight lines */
        analogWrite(ML, 255);
        analogWrite(MR, 255);
    }
    else
    {
        /* Normal operation */
        int power_difference = Compute(position);
        if (power_difference > maximum)
            power_difference = maximum;
        if (power_difference < -maximum)
            power_difference = -maximum;
        if (power_difference < 0)
        {
            analogWrite(ML, maximum + power_difference);
            analogWrite(MR, maximum);
        }
        else
        {
            analogWrite(ML, maximum);
            analogWrite(MR, maximum - power_difference);
        }
    }
}
