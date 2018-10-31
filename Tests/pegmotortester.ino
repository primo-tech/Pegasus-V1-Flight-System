#include <Servo.h>
#include <MPU6050.h>

long loop_timer;
unsigned long timer = 0;
float timeStep = 0.01;
float gpitch = 0, groll = 0, gyaw = 0;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;
int M1 = 3;
int M2 = 5;
int M3 = 10;
int M4 = 9;
int M5 = 6;
int M6 = 11;
int prevError  = 0;
int a  = 0,aT  = 0,b  = 0,bT  = 0,c  = 0,cT  = 0;
double p=0,i=0,d=0,cont=0;
unsigned long timeBetFrames = 0;
double PitchPID;

Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
MPU6050 mpu;

void setup() 
{
  Serial.begin(9600);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  init_motors();
}

     double prop = 6,inte = 1,deriv = 3;

void loop() 
{
   timer = millis();
   int *x;
   int *y;
   int *z;
   x = Axis_xyz();
   y = Axis_xyz()+1;
   z = Axis_xyz()+2;
   Serial.print("Pitch = \t");
   Serial.print(*x);
   Serial.print("\tRoll = \t");
   Serial.print(*y);
   Serial.print("\tYaw = \t");
   Serial.print(*z);
   Serial.println("");
   a = error(*x,0);
   b = error(*y,0);
   c = error(*z,0);
   aT += a;
   bT += b;
   cT += c;
  long  MP = pid(a,aT,prop,inte,deriv,timeBetFrames);
  long  MR = pid(b,bT,prop,inte,deriv,timeBetFrames);
  long  MY = pid(c,cT,prop,inte,deriv,timeBetFrames);
  //Serial.println(MP);
  int Throttle = 1300;
  FlightControl(Throttle,MP,MR,MY);
  //RunMotors(&Motor3,1100);
  timeBetFrames = millis() - timer;
  delay((timeStep*4000) - timeBetFrames); 
}
//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------
/*
 *                                   CALCULATING THE ERROR FOR BOTH POSITION AND ANGLE                     
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
    //i = InputErrorTotal*Ki*timeBetFrames;
    i = 0;
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
 *   CALCULATING THE AXIS VALUES
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
   Axis[0] = gpitch*2 + 15;
   Axis[1] = groll*2;
   Axis[2] = gyaw*2;
   return(Axis); 
}

