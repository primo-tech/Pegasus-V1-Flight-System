#include "init.h"
#include "motors.h"
#include "sensors.h"

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

double Motors::ALTControl(double input,double sensorVal)
{
   Setpoint = sensorVal; // start from zero;
   
   if( input > 1750)
   {
     Setpoint += 1.5;
   }
   else if( input > 1650 &&  input < 1750)
   {
     Setpoint += 1;
   }
   else if( input > 1550 &&  input < 1650)
   {
     Setpoint += 0.5;
   }
   else if( input < 1450 && input > 1350)
   {
     Setpoint -= 0.5;
   }
   else if( input < 1350 && input > 1250)
   {
     Setpoint -= 1;
   }
   else if( input < 1250)
   {
     Setpoint -= 1.5;
   }
   else if( input > 1450 && input < 1550)
   {
     Setpoint += 0;
   }
   return(Setpoint);
}

void Motors::Saturate(Servo Motor, int Signal, int lower, int upper)
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

void Motors::MotorMix(double Th,double P,double R,double Y)
{
  Signal[0] = Th-p+R+z;     // Top Left
  Signal[1] = Th-p+R-z;     // Bottom Left
  Signal[2] = Th-p-R-z;     // Top Right
  Signal[3] = Th-p-R+z;     // Bottom Right
  Signal[4] = Th+p+0+z;     // Top Rear
  Signal[5] = Th+p+0-z;     // Bottom Rear
  
  Saturate(Motor1,Signal[0],1300,2000);
  Saturate(Motor2,Signal[1],1300,2000);
  Saturate(Motor3,Signal[2],1300,2000);
  Saturate(Motor4,Signal[3],1300,2000);
  Saturate(Motor5,Signal[4],1300,2000);
  Saturate(Motor6,Signal[5],1300,2000);
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
