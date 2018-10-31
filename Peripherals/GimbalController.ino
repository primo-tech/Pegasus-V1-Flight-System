//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include <MPU6050.h>

/*
 * TIMERS
 */
unsigned long timer = 0;
float timeStep = 0.01;
unsigned long timeBetFrames = 0;
/*
 * IMU VALUES
 */
float gpitch = 0, groll = 0, gyaw = 0;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;

int *About_Z, *About_X;

Servo Z_Axis,X_Axis;
MPU6050 mpu;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Initialize MPU6050");
  
  Wire.begin();                // join i2c bus with address #1
  
  Z_Axis.attach(3);
  X_Axis.attach(5);
  init_sensors();
  Z_Axis.write(90);
  X_Axis.write(90);
}

void loop() 
{
  timer = millis();
  
  About_X = Axis_xyz();
  About_Z = Axis_xyz()+2;
  
  
  int VVal = map(*About_Z,-90,90,50,130); // 50 t0 130
  int WVal = map(*About_X,-90,90,50,125); // 50 t0 125
  
  Serial.println(VVal);

  int steps = 5, i;
  int Val,Val2, newVal,newWVal; 
  for(i = 0; i < 10;i++)
  {
    Val += (VVal/steps)*5;
    Val2 += (WVal/steps)*5;
    //delay(1);
  }
  
  newVal = Val/10;
  newWVal = Val2/10;
  
  Z_Axis.write(newVal);
  X_Axis.write(newWVal);
  
  timeBetFrames = millis() - timer;
  delay((timeStep*1000) - timeBetFrames); //Run at 100Hz
}

void init_sensors()
{
  while(!Serial) {} // Wait
  Serial.println("");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}
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
