#ifndef _MOTORS_
#define _MOTORS_

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Servo.h>

class Motors                  // create motor control class
{
  private:
    double Setpoint;             // variable for altitute hold setpoint
    
  public:
    double AltitudeControl(double input,double sensorVal);       // altitude hold algorithm
    
    void FullStop();                               // all motors set to lowest command value or off position
    void StartUp();                                // Right an intial value to all motors 
    
    void RunMotors(Servo* Motor,int Gain);   // transmits commands to motors
    void MotorMix(Servo x, int y, int lower, int upper);  // motor mixing algorithm for roll, pitch and yaw motion
    void FlightControl(double v,double x,double y,double z);   // transmitts mixed values to motors                                      
};

#endif
