#ifndef _SENSORREAD_
#define _SENSORREAD_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <BME280I2C.h>
#include <MPU6050.h>

class Sensor
{
  public:
    double Altitude();
    int *Axis_xyz();
    
  private:
    float timeStep = 0.01;
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

