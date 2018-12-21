#ifndef _SENSORREAD_
#define _SENSORREAD_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <BME280I2C.h>         // import wire library for i2c
#include <MPU6050.h>           // BME library for barometer, MPU library for IMU

class Sensor
{
  public:
    double Altitude();      // calculated the current altitude above sea level
    float *Axis_xyz();        // calculate the current pose roll, pitch, yaw(x,y,z)
    
  private:
    float timeStep = 0.01;   // time step for consistent cycle time and thus frequency
    /*
    * BAROMETER VALUES
    */
    double R = 8.3144598;
    double g = 9.80665;
    double M = 0.0289644;        
    double Pb = 101325;
    double num=0,dnum=0,h=0,hb = 0;

    /*
     * IMU VALUES
    */
    float gpitch = 0, groll = 0, gyaw = 0;
    float angle_pitch_output, angle_roll_output;
    boolean set_gyro_angles;
    
};

#endif _SENSORREAD_

