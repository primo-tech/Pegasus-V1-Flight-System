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

    int error(int a, int b);  // error for feedback calculations
    double pid(int InputError,int InputErrorTotal,unsigned long timeBetFrames);  // PID controller
    void RunMotors(Servo* Motor,int Gain);   // transmits commands to motors
    void MotorMix(Servo x, int y, int lower, int upper);  // motor mixing algorithm for roll, pitch and yaw motion
    void FlightControl(int v,int x,int y,int z);   // transmitts mixed values to motors
    void AltitudeControl(int al,double x);         // altitude hold algorithm
    void FullStop();                               // all motors set to lowest command value or off position

   private:
    int prevError  = 0;
    double p=0,i=0,d=0,cont=0;                     // PID variables and gains
                                  double Kp = 150,Ki = 10,Kd = 100;                   
};

#endif _MOTORS_
