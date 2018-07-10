#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include <MPU6050.h>

long loop_timer;
unsigned long timer = 0;
float timeStep = 0.01;

float gpitch = 0, groll = 0, gyaw = 0;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;

double R = 8.3144598;
double g = 9.80665;
double M = 0.0289644;
double Pb = 101325;
double num=0,dnum=0,h=0,hb = 0;

unsigned long int aa,bb,cc;
int x[15],ch1[15],ch[7],ii; //specifing arrays and variables to store values

// Motor connection pins
int M1 = 3;
int M2 = 5;
int M3 = 6;
int M4 = 9;
int M5 = 10;
int M6 = 11;

int prevError  = 0;
int a  = 0,aT  = 0,b  = 0,bT  = 0,c  = 0,cT  = 0;
int Setpoint1, Input1;
double p=0,i=0,d=0,cont=0;
unsigned long timeBetFrames = 0;

int ThrottleSetPoint = 0;
int PitchSetPoint = 0;
int RollSetPoint = 0;
int YawSetPoint = 0;
   
int *xA;
int *yA;
int *zA;

int Throttle; 

long  MP,MR,MY;


Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
MPU6050 mpu;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

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
               double prop = 6,inte = 0,deriv = 3;
void loop()
{ 
   timer = millis();
  
   read_rc();
   
   xA = Axis_xyz();
   yA = Axis_xyz()+1;
   zA = Axis_xyz()+2;
   
   Serial.print("Pitch = \t");
   Serial.print(*xA);
   Serial.print("\tRoll = \t");
   Serial.print(*yA);
   Serial.print("\tYaw = \t");
   Serial.print(*zA);
   Serial.println("");
   
   a = error(*xA,PitchSetPoint);
   b = error(*yA,RollSetPoint);
   c = error(*zA,YawSetPoint);
   
   aT += a;
   bT += b;
   cT += c;
   
  MP = pid(a,aT,prop,inte,deriv,timeBetFrames);
  MR = pid(b,bT,prop,inte,deriv,timeBetFrames);
  MY = pid(c,cT,prop,inte,deriv,timeBetFrames);
  Throttle = ThrottleSetPoint;
  //ThrottleControl();
  FlightControl(Throttle,MP,MR,MY);
  //PitchControl(MP,Throttle);
  //RollControl(MR,Throttle);
  //YawControl(MY,Throttle);
  
  timeBetFrames = millis() - timer;
  delay((timeStep*4000) - timeBetFrames); 
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
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
    return(cont);
}
/*
 *                                    CONTROLLING THE MOTORS
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
  RunMotors(&Motor6,1000);
  RunMotors(&Motor6,1000);
  delay(5000);
  RunMotors(&Motor3,1100);
  mpu.calibrateGyro();
  delay(200);
}
/*
 *   RUN MOTORS
 */
int ThrottleControl()
{
  return(ch1[0]);
}
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
 *   CALCULATING THE AXIS VALUES FROM MPU
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
     gpitch = gpitch * 0.9996 + Pitch * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
     groll = groll * 0.9996 + Roll * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
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
   
   Axis[0] = gpitch*3.143;
   Axis[1] = groll*3.143;
   Axis[2] = gyaw*3.143;
   
   return(Axis); 
}
/*
 *   CALCULATING THE ALTITUDE FROM BAROMETER
 */
double Alitude()
{
   float temp(NAN), hum(NAN), pres(NAN);
   double T = temp + 273;

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);
   
   num = log(pres/Pb) * T * R;
   dnum = g * M * -1;
   h = (num/dnum)+ hb;
   
   return(h);
}
/*
 *   INITIALISING THE SENSORS
 */
void init_sensors()
{
  while(!Serial) {} // Wait
  
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
void read_me() 
{
  //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
  //this code gives channel values from 0-1000 values 
  //    -: ABHILASH :-    //
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
