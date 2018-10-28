//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include <MPU6050.h>

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
 * IMU VALUES
 */
float gpitch = 0, groll = 0, gyaw = 0;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;
/*
 * BAROMETER VALUES
 */
double R = 8.3144598;
double g = 9.80665;
double M = 0.0289644;
double Pb = 101325;
double num=0,dnum=0,h=0,hb = 0;
/*
 * RECEIVER VALUES
 */
unsigned long int aa,bb,cc;
int x[15],ch1[15],ch[7],ii; //specifing arrays and variables to store values
/*
 * MOTOR CONNECTIONS
 */
int M1 = 3;     // Top Left
int M2 = 5;     // Bottom Left
int M3 = 6;     // Top Right
int M4 = 9;     // Bottom Right
int M5 = 10;    // Top Rear
int M6 = 11;    // Bottom Rear
/*
 * PID VARIABLES
 */
int prevError  = 0;
int a  = 0,aT  = 0;
int b  = 0,bT  = 0;
int c  = 0,cT  = 0;
int Setpoint1, Input1;
int PitchSetPoint = 0;
int RollSetPoint = 0;
int YawSetPoint = 0;

double p=0,i=0,d=0,cont=0;

unsigned long timeBetFrames = 0;

long  MP,MR,MY;

int *xA;
int *yA;
int *zA;
int *ThrottleSetPoint;
int *ThrottleSetPoint1;
int *ThrottleSetPoint2;
int *ThrottleSetPoint3;
int *ThrottleSetPoint4;
//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------
Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
MPU6050 mpu;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
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

  init_sensors();
  init_motors();
}
//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
void loop()
{ 
  timer = millis();

  RunMotors(&Motor1,1500);
  RunMotors(&Motor2,1500);
  RunMotors(&Motor3,1500);
  RunMotors(&Motor4,1500);
  RunMotors(&Motor5,1200);
  RunMotors(&Motor6,1200);
  
  xA = Axis_xyz();
  yA = Axis_xyz()+1;
  zA = Axis_xyz()+2;

  Serial.print(*xA+3);
  Serial.print("\t");
  Serial.print(*yA+1);
  Serial.print("\t");
  Serial.print(*zA);
  Serial.print("\t");
  Serial.println(Altitude());
  
  /*
  MainLoop();
  
  read_rc();
  
  if (ch[2] < 1100)
  {
    breakout = 0;
    delay(1000);
    MainLoop();
  }
  else
  {
    FullStop();
  }
  */
  
  timeBetFrames = millis() - timer;
  delay((timeStep*2000) - timeBetFrames); //Run at 50Hz
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
/*
 *   MAIN FLIGHT FUNCTIONALITY                        PID GAINS             
 */
                                            double prop = 6,inte = 1,deriv = 6;
void MainLoop()
{
  while(breakout != 1)
  {
    timer = millis();
  
    read_rc();
   
    xA = Axis_xyz();
    yA = Axis_xyz()+1;
    zA = Axis_xyz()+2;
    
    ThrottleSetPoint =  ThrottleControl();
    if(*ThrottleSetPoint > 1050)
    {
        PitchSetPoint = map(ch[3],1008,2008,30,-30);
        RollSetPoint = map(ch[4],1008,2008,30,-30);
        YawSetPoint = map(ch[2],1076,1936,-30,30);
        shutdowntime = 0;
    }
    else
    {
        PitchSetPoint = 0;
        RollSetPoint =0;
        YawSetPoint =0;
      
        shutdowntime += (millis()- timer)*10;

        if( shutdowntime > 2000)
        {
          breakout = 1;
        }
    }

    a = error(*xA+4,PitchSetPoint);
    b = error(*yA+1,RollSetPoint);
    c = error(*zA,YawSetPoint);
   
    aT += a;
    bT += b;
    cT += c;
   
    MP = pid(a,aT,prop,inte,deriv,timeBetFrames);
    MR = pid(b,bT,prop,inte,deriv,timeBetFrames);
    MY = pid(c,cT,prop,inte,deriv,timeBetFrames);
   
    FlightControl(*ThrottleSetPoint,MP,MR,MY);
    
    timeBetFrames = millis() - timer;
    delay((timeStep*1000) - timeBetFrames); //Run at 100Hz
  }
}
/*
 *  CALCULATING THE ERROR                     
 */
int error(int a, int b)
{
    int c;
    
    c = a - b;
    return(c);
}
/*
 *   CALCULATING THE PID GAIN VALUES
 */
double pid(int InputError,int InputErrorTotal,double Kp,double Ki,double Kd,unsigned long timeBetFrames)
{ 
    p = InputError*Kp;
    i = InputErrorTotal*Ki*timeBetFrames;
    d = (Kd*(InputError-prevError))/timeBetFrames;
    
    prevError = InputError;
    
    cont = p + i + d;
    if(cont > 250 )
    {
      cont = 250;
      return(cont);
    }
    else if(cont < -250)
    {
      cont = -250;
      return(cont);
    }
    else
    {
      return(cont);
    }
}
/*
 * CONTROLLING THE MOTORS
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
 * INITIALISING THE MOTORS
 */
void init_motors()
{
  mpu.calibrateGyro();
  mpu.setThreshold(10);
  
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
int *ThrottleControl()
{
  static int val[5];
  val[0] =  map(ch[1],1080,1970,1000,2000);
  val[1] =  map(ch[1],1080,1970,1050,2000);
  val[2] =  map(ch[1],1080,1970,1050,2000);
  val[3] =  map(ch[1],1080,1970,1060,2000);
  val[4] =  map(ch[1],1080,1970,1050,2000);
  return(val);
}
void FullStop()
{
  RunMotors(&Motor1,1000);
  RunMotors(&Motor2,1000);
  RunMotors(&Motor3,1000);
  RunMotors(&Motor4,1000);
  RunMotors(&Motor5,1000);
  RunMotors(&Motor6,1000);
}
void MotorMix(Servo x, int y)
{
  if (y > 2000)
  {
    y = 2000;
    RunMotors(&x,y);
  }
  else if(y < 1050)
  {
    y = 1050;
    RunMotors(&x,y);
  }
  else
  {
    RunMotors(&x,y);
  }
}
void FlightControl(int v,int x,int y,int z)
{
  int Run1 = v+x+y+z;     // Top Left
  int Run2 = v+x+y-z;     // Bottom Left
  int Run3 = v+x-y-z;     // Top Right
  int Run4 = v+x-y+z;     // Bottom Right
  int Run5 = v-x-y+z;     // Top Rear
  int Run6 = v-x+y-z;     // Bottom Rear
  
  MotorMix(Motor1,Run1);
  MotorMix(Motor2,Run2);
  MotorMix(Motor3,Run3);
  MotorMix(Motor4,Run4);
  MotorMix(Motor5,Run5);
  MotorMix(Motor6,Run6);
}
/*
 *   ALTITUDE HOLD ALGORITHM
 */
void AltitudeControl(int al,double x)
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
/*
 *   CALCULATING THE ALTITUDE FROM BAROMETER
 */
double Altitude()
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);
   double T = temp + 273;
   
   num = log(pres/Pb) * T * R;
   dnum = g * M * -1;
   h = (num/dnum)+ hb;
   
   return(h);
}
/*
 *   CALCULATING THE VALUES FROM IMU
 */
int *Axis_xyz()
{
   static int Axis[3];
   Vector norm = mpu.readNormalizeGyro();
   Vector normAccel = mpu.readNormalizeAccel();
   
   gpitch = gpitch + norm.YAxis * timeStep;
   groll = groll + norm.XAxis * timeStep;
   gyaw = gyaw + norm.ZAxis * timeStep;
   
   int Pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
   int Roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
   
   if(set_gyro_angles)
   {                                                 //If the IMU is already started
     gpitch = gpitch * 0.985 + Pitch * 0.015;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
     groll = groll * 0.985 + Roll * 0.015;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
   }
   else
   {                                                                //At first start
     gpitch = Pitch;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
     groll = Roll;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
     set_gyro_angles = true;                                            //Set the IMU started flag
   }
   
   //To dampen the pitch and roll angles a complementary filter is used
   angle_pitch_output = angle_pitch_output * 0.9 +  gpitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
   angle_roll_output = angle_roll_output * 0.9 +  groll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
   
   Axis[0] = gpitch;
   Axis[1] = groll;
   Axis[2] = gyaw;
   
   return(Axis); 
}
/*
 *   INITIALISING THE SENSORS
 */
void init_sensors()
{
  while(!Serial) {} // Wait
  Serial.println("");
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
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
