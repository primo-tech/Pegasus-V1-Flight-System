//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------

#include "init.h"
#include "motors.h"
#include "sensorRead.h"

//---------------------------------------------------------------------------------------------------------------
/*
 *                                    VARIABLE/CONSTANT DEFINITIONS
 */
//---------------------------------------------------------------------------------------------------------------
/*
 * TIMERS
 */
long loop_timer;
unsigned long timer = 0;
unsigned long shutdowntime;
float timeStep = 0.01;
bool breakout = 0;
/*
 * RECEIVER VALUES
 */
unsigned long int aa,bb,cc;
int x[15],ch1[15],ch[7],ii; //specifing arrays and variables to store values

unsigned long timeBetFrames = 0;

long  MP,MR,MY;

int *xA = 0;
int *yA = 0;
//int *zA = 0;
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Motors motor;                   // Instantiate motor control class
Initialise inital;              // Instantiate initialisation class
Sensor readIn;                  // Instantiate Sensor class
//--------------------------------------------------------------------------------------------------------------
/*
 *                                   COMPONENT INITIALISATION LOOP 
 */
//--------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Serial.println("Initialize BME280");
  Serial.println("Initialize MPU6050");
  
  Wire.begin();                // join i2c bus with address #1
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING); // enabling interrupt at pin 2
}
//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
void loop()
{ 
  motor.FullStop();  // otherwise set all motor values to 0
  breakout = 0;
  read_rc();             // read receiver values 
     
  if(ch[1]< 1100 && ch[2] > 1800 && ch[3] < 1300 & ch[4] < 1100)
  {
    inital.init_sensors();          // intialise IMU and Barometer
    inital.init_motors();           // intialise motors and calibrate IMU

    while(breakout != 1)
    {
      motor.StartUp();
      read_rc();
      
      if(ch[1] > 1300)
      { 
        MainLoop();            // run main flight controll loop        
      }
    }
  }
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
/*
 *   MAIN FLIGHT FUNCTIONALITY                             
 */                                   
void MainLoop()
{
  /*
 * PID VARIABLES
 */
int a  = 0;
int b  = 0;
//int c  = 0;

int ThrottleSetPoint = 0;
int PitchSetPoint = 0;
int RollSetPoint = 0;
//int YawSetPoint = 0;

  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();                         // begin decoding PPM values
   
    xA = readIn.Axis_xyz();
    yA = readIn.Axis_xyz()+1;          // read in roll pitch and yaw IMU values
    //zA = readIn.Axis_xyz()+2;

    //Serial.print(*xA+3);
    //Serial.print("\t");
    //Serial.println(*yA+1);
    
    ThrottleSetPoint =  map(ch[1],1040,2020,1000,1800);            // read in throttle setpoint
    if(ThrottleSetPoint > 1050)
    {
        PitchSetPoint = map(ch[4],1000,1900,30,-30);
        RollSetPoint = map(ch[3],1200,2020,30,-30);   // read in roll pitch and yaw setpoint values from receiver
        //YawSetPoint = map(ch[2],1076,1936,-30,30);    // and map to between 0 and 30 degrees 
        shutdowntime = 0;                             // keep a running count of time within loop
    }
    else
    {
        PitchSetPoint = 0;
        RollSetPoint =0;
        //YawSetPoint =0;
      
        shutdowntime += (millis()- timer)*10;           

        if( shutdowntime > 4000)                  // if running count exceed 2 seconds break out of main loop
        {                                         // and reset all setpoints to zero 
          breakout = 1;
        }
    }

    a = motor.error(PitchSetPoint,*xA+3);
    b = motor.error(RollSetPoint,*yA+1);    // calculated error from setpoints
    //c = motor.error(YawSetPoint,*zA);        // error = setpoint - sensorValue
    
    MP = motor.pid(a,timeBetFrames);
    MR = motor.pid(b,timeBetFrames);       // Calculate roll Ptich and yaw PID values
    //MY = motor.pid(c,timeBetFrames);
    MY = map(ch[2],1070,1930,-200,200);    // non feedback rate control for yaw
   
    motor.FlightControl(ThrottleSetPoint,MP,MR,MY);  // Send PID values to Motor Mixing algorithm
    
    timeBetFrames = millis() - timer;
    delay((timeStep*2000) - timeBetFrames); //Run at 100Hz
  }
}                    
/*
 *   READ PPM VALUES FROM PIN 2
 */
  //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
  //this code gives channel values from 0-1000 values 
  //    -: ABHILASH :-    //
void read_me() 
{
  int j;
  
  aa=micros();   //store time value a when pin value falling
  cc=aa-bb;      //calculating time inbetween two peaks
  bb=aa;         
  x[ii]=cc;      //storing 15 value in array
  ii=ii+1;       

  if(ii==15)
  {
    for(j=0;j<15;j++) 
    {
      ch1[j]=x[j];
    }
    ii=0;
  }
}//copy store all values from temporary array another array after 15 reading 

void read_rc()
{
  int i,j,k=0;
  
  for(k=14;k>-1;k--)
  {
    if(ch1[k]>10000)
    {
      j=k;
    }
  } //detecting separation space 10000us in that another array
                    
  for(i=1;i<=6;i++)
  {
    ch[i]=(ch1[i+j]);
  }
}     //assign 6 channel values after separation space
