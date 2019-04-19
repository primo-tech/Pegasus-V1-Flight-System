#ifndef _MOTORS_
#define _MOTORS_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>         // import wire library for i2c, servo library for motors
#include <MPU6050.h>           // BME library for barometer, MPU library for IMU

class Motors                  // create motor control class
{
  public:
    int Setpoint;             // variable for altitute hold setpoint
    float prevError  = 0;
    float prevSetPoint = 0;
    float InputErrorTotal = 0;
    float p=0, i=0, d=0, cont=0;
    float y = 0, yT = 0, N = 0.15;  

                              // PID variables and gains
                              float RKp = 8,RKi = 0.001,RKd = 10;
                              float PKp = 8,PKi = 0.001,PKd = 10;
                                   
    float error(float a, float b);  // error for feedback calculations
    float pid(double InputError,double SetPoint,unsigned long timeBetFrames,float Kp, float Ki, float Kd);  // PID controller
    
    void RunMotors(Servo* Motor,int Gain);   // transmits commands to motors
    void MotorMix(Servo x, int y, int lower, int upper);  // motor mixing algorithm for roll, pitch and yaw motion
    void FlightControl(double v,double x,double y,double z);   // transmitts mixed values to motors
    void AltitudeControl(int al,double x);         // altitude hold algorithm
    void FullStop();                               // all motors set to lowest command value or off position
    void StartUp();                                // Right an intial value to all motors                                      
};

#endif _MOTORS_
