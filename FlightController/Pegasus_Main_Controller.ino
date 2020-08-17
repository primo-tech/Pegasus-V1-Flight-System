//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------
#include "inits.h"
#include "motors.h"
#include "sensors.h"
#include <PID_v1.h>
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
unsigned long timeBetFrames = 0;
unsigned long shutdowntime;
float timeStep = 0.01;
bool breakout = 0;
/*
 * RECEIVER VARIABLES
 */
unsigned long int aa,bb,cc;
int x[15],ch1[15],ch[7],ii; // specifing arrays and variables to store values
/*
 *  CONTROL VARIABLES
 */
double MA,MP,MR,MY;

double alt;
double *xA;
double *yA;

double Ainput,Rinput,Pinput;
double initialAlt,initialRoll,initialPitch;

double ThrottleSetPoint;
double AltitudeSetPoint;
double PitchSetPoint;
double RollSetPoint;

double Throttle = 1000;
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Motors motor;           // instantiate motor control class
Inits initi;            // instantiate initialisation class
Sensors sensor;         // instantiate Sensor class

PID PIDAlt(&Ainput,  &MA, &AltitudeSetPoint,10,0.2,0, DIRECT);  // altitude PID controller
PID PIDPitch(&Pinput,&MP, &PitchSetPoint,8,0.2,10, DIRECT);     // pitch PID controller
PID PIDRoll(&Rinput, &MR, &RollSetPoint,8,0.2,10, DIRECT);      // roll PID controller
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
  
  PIDAlt.SetOutputLimits(-200,200);
  PIDPitch.SetOutputLimits(-200,200);
  PIDRoll.SetOutputLimits(-200,200);

  PIDAlt.SetMode(AUTOMATIC);
  PIDPitch.SetMode(AUTOMATIC);
  PIDRoll.SetMode(AUTOMATIC);
  
  PIDAlt.SetSampleTime(10);
  PIDPitch.SetSampleTime(10);
  PIDRoll.SetSampleTime(10);
  
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
  motor.FullStop();  // set all motor values to 0
  breakout = 0;
  read_rc();         // read receiver values 
  digitalWrite(4,1);   
  //if(ch[1]< 1100 && ch[2] > 1800 && ch[3] < 1300 && ch[4] < 1100)
  //{
    digitalWrite(4,0);
    initi.initSensors();          // intialise IMU and Barometer
    initi.initMotors();           // intialise motors and calibrate IMU

    while(breakout != 1)
    {
      motor.StartUp();
      read_rc();
      
      //if(ch[1] > 1200)
      //{
        MainLoop();                 // run main flight controll loop        
      //}
    }
  //}
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
  ThrottleSetPoint = 0;
  AltitudeSetPoint = 0;
  PitchSetPoint = 0;
  RollSetPoint = 0;
  
  MA = 0;
  MP = 0;
  MR = 0;
  MY = 0;
  
  initialAlt = 0;
  initialRoll = 0;
  initialPitch = 0;
  
  for(int counter = 0; counter < 100; counter++)
  {
    initialAlt += sensor.ALT();
    initialPitch += *sensor.IMU();    // calibrate sensor offsets 
    initialRoll += *sensor.IMU()+1;
  }
  initialAlt = initialAlt/100;
  initialPitch = initialPitch/100;
  initialRoll = initialRoll/100;
  
  Serial.println(initialAlt);
  Serial.println(initialPitch);
  Serial.println(initialRoll);

  delay(5000);
  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();                          // begin decoding PPM values
    
    Ainput = sensor.ALT() - initialAlt; // read in current altitude value
    
    xA = (double *)sensor.IMU();
    yA = (double *)sensor.IMU()+1;      // read in roll and pitch IMU values
    
    Pinput = *xA - initialPitch;        // set the roll and pitch value to PID inputs
    Rinput = *yA;
    
    ThrottleSetPoint =  map(ch[1],1000,1930,1000,1800);  // read in throttle setpoint
    Serial.println(Ainput);
    if(ThrottleSetPoint > 1050)
    {
        AltitudeSetPoint += motor.ALTControl(ThrottleSetPoint,Ainput); // calcute the altitude setpoint from throttle commands
        PitchSetPoint = map(ch[4],1000,2000,10,-10);
        RollSetPoint = map(ch[3],1000,2000,-10,10);   // read in roll pitch and yaw setpoint values from receiver
                                                      // and map to between 0 and 10 degrees 
        shutdowntime = 0;                             // keep a running count of time within loop
    }
    else
    {
        AltitudeSetPoint = 0;
        PitchSetPoint = 0;
        RollSetPoint = 0;
       
        shutdowntime += (millis() - timer)*10;           
        
        if( shutdowntime > 4000)              // if running count exceeds 4000 counts break out of main loop
        {                                     // and reset all setpoints to zero
          digitalWrite(12,0);
          digitalWrite(13,0);
          breakout = 1;
        }
    }

    PIDAlt.Compute();
    PIDPitch.Compute();                       // calculate PID values
    PIDRoll.Compute();
    
    MY = map(ch[2],1070,1930,-200,200);       // non feedback rate control for yaw
    
    Throttle = MA;
    if (Throttle < 1200)
    {
      Throttle = 1200;                        // saturate throttle value to actuator limits
    }
    else if (Throttle > 1800)
    {
      Throttle = 1800;
    }

    //Serial.println(MP);
    motor.MotorMix(Throttle,MP,MR,MY);        // send PID values to Motor Mixing algorithm
    
    timeBetFrames = millis() - timer;
    delay((timeStep*2000) - timeBetFrames);   // run Loop at 100Hz
  }
}                    
/*
 *   READ PPM VALUES FROM PIN 2
 *   this code reads value from RC reciever from PPM pin (Pin 2 or 3)
 *   this code gives channel values from 0-1000 values    
 *   : ABHILASH :
 */
void read_me() 
{
  int j;
  
  aa = micros();   // store time value a when pin value falling
  cc = aa - bb;    // calculating time inbetween two peaks
  bb = aa;         
  x[ii] = cc;      // storing 15 value in array
  ii = ii + 1;       

  if(ii == 15)
  {
    for(j = 0;j < 15;j++)   // copy store all values from temporary array another array after 15 reading
    {
      ch1[j] = x[j];
    }
    ii = 0;
  }
}

void read_rc()
{
  int i,j,k = 0;
  
  for(k = 14;k > - 1;k--)   // detecting separation space 10000us in that another array
  {
    if(ch1[k] > 10000)
    {
      j = k;
    }
  } 
                    
  for(i = 1;i <= 6;i++)
  {
    ch[i] = (ch1[i+j]);    // assign 6 channel values after separation space
  }
}
