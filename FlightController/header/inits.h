#ifndef _INITS_
#define _INITS_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>         // import wire library for i2c, servo library for motors
#include <MPU6050.h>           // BME library for barometer, MPU library for IMU
#include <HMC5883L.h>          // Reference the HMC5883L Compass Library
#include <FIR.h>               // Finite impulse response filter library

class Inits          // create initialisation class
{
  private:
    int M1 = 3;     // top left
    int M2 = 5;     // bottom left
    int M3 = 6;     // top right
    int M4 = 9;     // bottom right                 Motor Pin assignments
    int M5 = 10;    // top rear
    int M6 = 11;    // bottom rear
  
  public:
    void initSensors();          // initialise the sensors
    void initMotors();           // initialise the motors
};

#endif
