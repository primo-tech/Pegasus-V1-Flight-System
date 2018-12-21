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

float Motors::error(float a, float b)
{
  float c;
  c = a - b;                  // calculate the difference between a and b return as error
  return(c);
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

void Motors::FlightControl(float v,float x,float y,float z)
{
  float Run1 = v-x+y-z;     // Top Left
  float Run2 = v-x+y+z;     // Bottom Left
  float Run3 = v-x-y+z;     // Top Right
  float Run4 = v-x-y-z;     // Bottom Right
  float Run5 = v+x-y-z;     // Top Rear
  float Run6 = v+x-y+z;     // Bottom Rear
  
  MotorMix(Motor1,Run1,1300,2000);
  MotorMix(Motor2,Run2,1300,2000);
  MotorMix(Motor3,Run3,1300,2000);
  MotorMix(Motor4,Run4,1300,2000);
  MotorMix(Motor5,Run5,1050,2000);
  MotorMix(Motor6,Run6,1050,2000);
}

void Motors::StartUp()
{
  RunMotors(&Motor1,1200);
  RunMotors(&Motor2,1100);
  RunMotors(&Motor3,1100);
  RunMotors(&Motor4,1100);
  RunMotors(&Motor5,1100);
  RunMotors(&Motor6,1100);
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

float Motors::pid(float InputError,unsigned long timeBetFrames)
{
  InputErrorTotal += InputError;
  yT += y;
  
  p = InputError*Kp;
  i = InputErrorTotal*Ki*timeBetFrames;
  //d = (Kd*(InputError-prevError))/timeBetFrames;
  y = N*(InputError - (yT*timeBetFrames));   // Derivative Prefilter
  d = y*Kd;
  cont = p + i + d;
  prevError = InputError;
  
  // integral anti-windup
  if(cont > 200 )
  {
    cont = 200;
    if (InputError > 0)
    {
      InputErrorTotal = 0;
    }
  }
  else if(cont < -200 )
  {
    cont = -200;
    if (InputError < 0)
    {
      InputErrorTotal = 0;
    }
  }
  
  //Derivative Setpoint Weighting
  if(InputError < 1 && InputError > -1)
  {
    d  = 0;
  }

  return(cont);
}
