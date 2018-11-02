#ifndef _MOTORS_
#define _MOTORS_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include <MPU6050.h>

class Motors
{
  public:
    int Setpoint1, Input1;
    
    void RunMotors(Servo* Motor,int Gain);
    double pid(int InputError,int InputErrorTotal,unsigned long timeBetFrames);
    void AltitudeControl(int al,double x);
    void FlightControl(int v,int x,int y,int z);
    void MotorMix(Servo x, int y);
    void FullStop();
    int error(int a, int b);

   private:
    int prevError  = 0;
    double p=0,i=0,d=0,cont=0;
                                  double Kp = 150,Ki = 10,Kd = 100;                   
};

#endif _MOTORS_
