#include "motors.h"

#include <Servo.h>

extern Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

void Motors::RunMotors(Servo* Motor,int Gain)
{
  int x = 0;
    
    if(Gain > 2000)
    {
        x = 2000;                      // Actuator Limit Saturation 
    }
    if(Gain < 1000)
    {
        x = 1000;                      // Actuator Limit Saturation 
    }
    else
    {
        x = Gain;              
    }
    Motor->writeMicroseconds(x);       // write gain to motors
}

void Motors::AltitudeControl(int al,double x)
{
  double b = x;
                         // checks commanded value, compare that with actual value and returns compensated value
   if( al > 1750)
   {
     b = b + 1.5;
     Setpoint = b;
   }
   else if( al > 1650 &&  al < 1750)
   {
     b = b + 1;
     Setpoint = b;
   }
   else if( al > 1550 &&  al < 1650)
   {
     b = b + 0.5;
     Setpoint = b;
   }
   else if( al < 1450 && al > 1350)
   {
     b = b - 0.5;
      Setpoint = b;
   }
   else if( al < 1350 && al > 1250)
   {
     b = b - 1;
     Setpoint = b;
   }
   else if( al < 1250)
   {
     b = b - 1.5;
     Setpoint = b;
   }
   else if( al > 1450 && al < 1550)
   {
     Setpoint = b;
   }
}

void Motors::MotorMix(Servo x, int y, int lower, int upper)
{
  if (y > upper)
  {
    y = upper;
  }
  else if(y < lower)     // set upper and lower motor limits as each motor is different
  {                      // lower limit = just when the motor starts moving
    y = lower;
  }
  else
  {
    y = y;
  }
  RunMotors(&x,y);
}

void Motors::FlightControl(double v,double x,double y,double z)
{
  double Run1 = v-x+y-z;     // Top Left
  double Run2 = v-x+y+z;     // Bottom Left
  double Run3 = v-x-y+z;     // Top Right
  double Run4 = v-x-y-z;     // Bottom Right
  double Run5 = v+x-(y*0.5)-z;     // Top Rear
  double Run6 = v+x-(y*0.5)+z;     // Bottom Rear
  
  MotorMix(Motor1,Run1,1300,2000);
  MotorMix(Motor2,Run2,1300,2000);
  MotorMix(Motor3,Run3,1300,2000);
  MotorMix(Motor4,Run4,1300,2000);
  MotorMix(Motor5,Run5,1100,2000);
  MotorMix(Motor6,Run6,1100,2000);
}

void Motors::StartUp()
{
  RunMotors(&Motor1,1200);
  RunMotors(&Motor2,1200);
  RunMotors(&Motor3,1200);
  RunMotors(&Motor4,1200);
  RunMotors(&Motor5,1200);
  RunMotors(&Motor6,1200);
}

void Motors::FullStop()
{
  RunMotors(&Motor1,1000);
  RunMotors(&Motor2,1000);
  RunMotors(&Motor3,1000);
  RunMotors(&Motor4,1000);
  RunMotors(&Motor5,1000);
  RunMotors(&Motor6,1000);
}
