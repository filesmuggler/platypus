#include "TRSensors.h"

// Holds received data from bluetooth
char bt_received_state;
String receivedData = "";
unsigned long previous_millis = 0;
short interval = 10;
int handy_counter = 0;
unsigned long current_millis;

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
unsigned int position; // position obtained from sensors (2000 is neutral)
unsigned long lastTime = 0;
int Setpoint;
int lastOutput = 100;
double errSum = 0;
double lastErr = 0;
double kp, ki, kd;
double kp_r, ki_r, kd_r;
int SampleTime = 1000; //1 sec

const int maximum = 150;

void setup()
{

    Serial.begin(9600);
    //Serial.println("hello");
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
    SetSampleTime(1);
    //SetTunings(12, 6, 0);

    /* sensors calibration */
    for (int i = 0; i < 400; i++)
    {
        sensors.calibrate();
        /* DEBUG ONLY */
        /* display progress */
    }

    while(!if_start){
        while (Serial.available())
    {
        current_millis = millis();
        if (current_millis - previous_millis > interval)
        {
            //read data
            previous_millis = current_millis;
            bt_received_state = Serial.read();
            //Serial.println(bt_received_state);
            if (bt_received_state == '|')
            {
                //convert receivedstate to int
                int temp_data = receivedData.toInt();
                //Serial.println(temp_data);
                //assign k
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
                    //SetTunings(12,6,0);
                    //Serial.print(kp_r);
                    //Serial.print(' ');
                    //Serial.print(ki_r);
                    //Serial.print(' ');
                    //Serial.print(kd_r);
                    //Serial.print('\n');

                    if (kp_r == 0 && ki_r == 0 && kd_r == 0)
                    {
                        //stop
                        if_start = false;
                        if_stop = true;
                    }
                    else
                    {
                        //start
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

    delay(1000);
}

void loop()
{
    
    
    if (!if_start)
    {
        Stop();
    }
    else if (!if_stop)
    {
        Drive();
    }
}

int Compute(unsigned int input)
{
    unsigned long now = millis();
    int timeChange = (now - lastTime);
    if (timeChange >= SampleTime)
    {
        /*Compute all the working error variables*/
        double error = Setpoint - input; //P part

        //I part
        if(lastOutput >= maximum && error >0){
            errSum = 0;
        }
        else if(lastOutput <= -maximum && error <0){
            errSum = 0;
        }
        else{
            errSum += error * (double)timeChange / 1000.0; 
        }

        double dErr = (error - lastErr); //D part

        /* Compute PID output */
        int output = kp * error + ki*errSum + kd * dErr;
        //int output = kp * error;
        /* save some values for next time */
        lastErr = error;
        lastTime = now;
        lastOutput = output;
    }
    return lastOutput;
}

void SetTunings(double Kp, double Ki, double Kd)
{
    double SampleTimeInSec = ((double)SampleTime) / 1000.0;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

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

void Drive()
{
    /* read sensors */
    position = sensors.readLine(sensorsValues);

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

void Stop()
{
    /* stop all motors */
    analogWrite(ML, 0);
    analogWrite(MR, 0);
}
