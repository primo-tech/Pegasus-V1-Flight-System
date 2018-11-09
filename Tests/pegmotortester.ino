#include <Servo.h>

int M1 = 3;
int M2 = 5;
int M3 = 6;
int M4 = 9;
int M5 = 10;
int M6 = 11;

Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

void setup() 
{
  Serial.begin(9600);
  init_motors();
}

void loop() 
{
  int x = map(analogRead(A0),130,820,1000,2000);
  Serial.println(x);
  
  RunMotors(&Motor1,1000);
  RunMotors(&Motor2,1000);
  RunMotors(&Motor3,1000);
  RunMotors(&Motor4,1000);
  RunMotors(&Motor5,x);
  RunMotors(&Motor6,1000);
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
/*
 *                                   CALCULATING THE ERROR FOR BOTH POSITION AND ANGLE                     
 */
void RunMotors(Servo* Motor,int Gain)
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
/*
 *                                    INITIALISING THE MOTORS
 */
void init_motors()
{
  Motor1.attach(M1);
  Motor2.attach(M2);
  Motor3.attach(M3);
  Motor4.attach(M4);
  Motor5.attach(M5);
  Motor6.attach(M6);
  
  RunMotors(&Motor1,1000);
  RunMotors(&Motor2,1000);
  RunMotors(&Motor3,1000);
  RunMotors(&Motor4,1000);
  RunMotors(&Motor5,1000);
  RunMotors(&Motor6,1000);
  delay(5000);
}
/*
 *   RUN MOTORS
 */
void FlightControl(int v,int x,int y,int z)
{
  RunMotors(&Motor1,v+x+y+z);
  RunMotors(&Motor2,v+x-y-z);
  RunMotors(&Motor3,v-x-y+z);
  RunMotors(&Motor4,v-x+y-z);
}
void PitchControl(int x,int y)
{
  RunMotors(&Motor1,y+x);
  RunMotors(&Motor2,y+x);
  RunMotors(&Motor3,y-x);
  RunMotors(&Motor4,y-x);
}
void RollControl(int x,int y)
{
  RunMotors(&Motor1,y+x);
  RunMotors(&Motor2,y-x);
  RunMotors(&Motor3,y-x);
  RunMotors(&Motor4,y+x);
}
void YawControl(int x,int y)
{
  RunMotors(&Motor1,y+x);
  RunMotors(&Motor2,y-x);
  RunMotors(&Motor3,y+x);
  RunMotors(&Motor4,y-x);
}
