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
#include <HMC5883L.h>          // reference the HMC5883L compass library
#include <FIR.h>               // finite impulse response filter library

#define FILTERTAPS 5

class Sensors
{
  private:
    float timeStep = 0.01;   // time step for consistent cycle time and thus frequency
    /*
     * FILTER VALUES
     */
    float coef[FILTERTAPS] = { 0.021, 0.096, 0.146, 0.096, 0.021};
    float gain = 1;        // set to 1 and input unity to see what this needs to be
    /*
    * BAROMETER VALUES
    */
    double R = 8.3144598;
    double g = 9.80665;
    double M = 0.0289644;        
    double Pb = 101325;
    double num=0,dnum=0,h=0,hb = 0;
    double T;
    /*
     * IMU VALUES
    */
    float apitch = 0,aroll = 0;
    float gpitch = 0, groll = 0, gyaw = 0;
    float angle_pitch_output, angle_roll_output;
    boolean set_gyro_angles; 
    /*
     * COMPASS VALUES
    */
    int error = 0;
    float declinationAngle = 0.0457;
    float heading;
    float headingDegrees;
    
  public:
    Sensors();
    
    double ALT();     // calculated the current altitude above sea level
    double *IMU();    // calculate the current pose roll, pitch, yaw(x,y,z)
    double MAG();     // calculate the current heading  yaw(z)

    double aveFilter(int sampleNumber, double value);  // simple averaging filter
    double compFilter(int sampleNumber, double value); // simple complimentary filter 
};

#endif
