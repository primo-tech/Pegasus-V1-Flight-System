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
    double Setpoint;          // variable for altitute hold setpoint
    double Signal[6];         // array to hold motor input signal
       
  public:
    double ALTControl(double input,double sensorVal);       // altitude hold algorithm
    
    void FullStop();                               // all motors set to lowest command value or off position
    void StartUp();                                // Right an intial value to all motors 
    
    void RunMotors(Servo* Motor,int Gain);   // transmits commands to motors
    void Saturate(Servo Motor, int Signal, int lower, int upper);  // motor mixing algorithm for roll, pitch and yaw motion
    void MotorMix(double Th,double P,double R,double Y);   // transmitts mixed values to motors                                      
};

#endif
