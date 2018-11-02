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
        x = Gain;              // add the PID gain to the initial velocity
    }
    Motor->writeMicroseconds(x);
}

void Motors::AltitudeControl(int al,double x)
{
  double b = x;
    
   if( al > 1750)
   {
     b = b + 1.5;
     Setpoint1 = b;
   }
   else if( al > 1650 &&  al < 1750)
   {
     b = b + 1;
     Setpoint1 = b;
   }
   else if( al > 1550 &&  al < 1650)
   {
     b = b + 0.5;
     Setpoint1 = b;
   }
   else if( al < 1450 && al > 1350)
   {
     b = b - 0.5;
      Setpoint1 = b;
   }
   else if( al < 1350 && al > 1250)
   {
     b = b - 1;
     Setpoint1 = b;
   }
   else if( al < 1250)
   {
     b = b - 1.5;
     Setpoint1 = b;
   }
   else if( al > 1450 && al < 1550)
   {
     Setpoint1 = b;
   }
}

int Motors::error(int a, int b)
{
  int c;
  c = a - b;
  return(c);
}

void Motors::FlightControl(int v,int x,int y,int z)
{
  int Run1 = v+x+y+z;     // Top Left
  int Run2 = v+x+y-z;     // Bottom Left
  int Run3 = v+x-y-z;     // Top Right
  int Run4 = v+x-y+z;     // Bottom Right
  int Run5 = v-x+y+z;     // Top Rear
  int Run6 = v-x-y-z;     // Bottom Rear
  
  MotorMix(Motor1,Run1);
  MotorMix(Motor2,Run2);
  MotorMix(Motor3,Run3);
  MotorMix(Motor4,Run4);
  MotorMix(Motor5,Run5);
  MotorMix(Motor6,Run6);
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

double Motors::pid(int InputError,int InputErrorTotal,unsigned long timeBetFrames)
{
  bool intOn;
  double new_i;

  new_i = i+InputErrorTotal;
  
  p = InputError*Kp;
  i = InputErrorTotal*Ki*timeBetFrames;
  d = (Kd*(InputError-prevError))/timeBetFrames;

  prevError = InputError;
    
  cont = p + i + d;

  intOn = true;
  
  if(cont > 100 )
  {
    cont = 100;
    if (InputError > 0)
    {
      intOn = false;
    }
  }
  else if(cont < -100 )
  {
    cont = -100;
    
    if (InputError < 0)
    {
      intOn = false;
    }
  }
  
  if(intOn)
  {
    i = new_i;
  }
  
  return(cont);
}

void Motors::MotorMix(Servo x, int y)
{
  if (y > 2000)
  {
    y = 2000;
  }
  else if(y < 1050)
  {
    y = 1050;
  }
  else
  {
    y = y;
  }
  RunMotors(&x,y);
}                                
