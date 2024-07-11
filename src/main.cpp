#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32Time.h>
#include "Gyro.hpp"
#include "Accel.hpp"
#include "Mag.hpp"


int16_t mx,my,mz;
// double ax,ay,az;
// int16_t gx,gy,gz;

Servo right_prop;
Servo left_prop;

const int MIN_PWM = 1050;
const int MAX_PWM = 1200;


float Acceleration_angle[2] = {0,0};
float Gyro_angle[2] = {0,0};
float Total_angle[2] = {0,0};


float elapsedTime;
unsigned long curr_time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=0.1;//3.55
double ki=0.08;//0.003
double kd=0.05;//2.05
///////////////////////////////////////////////

double throttle=1000;
float desired_angle = 0;

// int changeLeftMotorScale(int pwm);

// int changeRightMotorScale(int pwm);


void setup(){
    Serial.begin(9600);
    Wire.begin();
    curr_time = millis();
    setupMag();
    readMag(mx,my,mz);
    setupGyro(2000);
    setupAccel();
    right_prop.attach(18);
    left_prop.attach(19);
    left_prop.writeMicroseconds(1000); 
    right_prop.writeMicroseconds(1000);
    desired_angle = my;
    delay(7000);
}

void loop(){
    timePrev = curr_time;
    curr_time = millis();
    elapsedTime = (curr_time - timePrev)/1000.f; 

    readMag(mx,my,mz);
    // readAccel(ax,ay,az);
    // readGyro(gx,gy,gz);

    // Acceleration_angle[0] = atan((ay/16384.0)/sqrt(pow((ax/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;
    // Acceleration_angle[1] = atan(-1*(ax/16384.0)/sqrt(pow((ay/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;

    // Gyro_angle[0] = gx/131.0; 
    // Gyro_angle[1] = gy/131.0;

    // Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    // Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   

    // error = Total_angle[1] - desired_angle;
    error = my - desired_angle;
        
    pid_p = kp*error;

    if(-3 < error < 3){
        pid_i = pid_i+(ki*error);  
    }

    pid_d = kd*((error - previous_error)/elapsedTime);

    PID = pid_p + pid_i + pid_d;

    if(PID < -1000){
        PID=-1000;
    }
    if(PID > 1000){
        PID=1000;
    }

    pwmLeft = throttle + PID;
    pwmRight = throttle - PID;

    //Right
    if(pwmRight < MIN_PWM){
        pwmRight = MIN_PWM;
    }
    if(pwmRight > MAX_PWM){
        pwmRight = MAX_PWM;
    }
    //Left
    if(pwmLeft < MIN_PWM){
        pwmLeft = MIN_PWM;
    }
    if(pwmLeft > MAX_PWM){
        pwmLeft = MAX_PWM;
    }

    /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
    left_prop.writeMicroseconds(pwmLeft);
    right_prop.writeMicroseconds(pwmRight);
    previous_error = error; //Remember to store the previous error.

    // Serial.print("Left Prop pwm: ");
    // Serial.println(pwmLeft);
    // Serial.print("Right Prop pwm: ");
    // Serial.println(pwmRight);
    // Serial.print("Error ");
    // Serial.println(error);
    // Serial.print("Angle Read ");
    // Serial.println(my);
    // Serial.print("PID Signal");
    // Serial.println(PID);
    // delay(250);
}
