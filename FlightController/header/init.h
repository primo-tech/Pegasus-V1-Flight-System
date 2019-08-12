#ifndef _INIT_
#define _INIT_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>         // import wire library for i2c, servo library for motors
#include <MPU6050.h>           // BME library for barometer, MPU library for IMU

class Initialise              // create initialisation class
{
  public:
    int M1 = 3;     // Top Left
    int M2 = 5;     // Bottom Left
    int M3 = 6;     // Top Right
    int M4 = 9;     // Bottom Right                 Motor Pin assignments
    int M5 = 10;    // Top Rear
    int M6 = 11;    // Bottom Rear

    void init_sensors();          // initialise the sensors
    void init_motors();           // initialise the motors
};

#endif
