#include "sensorRead.h"

#include <Wire.h>
#include <BME280I2C.h>
#include <MPU6050.h>

extern MPU6050 mpu;        // extern allows other files to use these values
extern BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
                  
double Sensor::Altitude()
{
  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa); 
          
  bme.read(pres, temp, hum, tempUnit, presUnit);  //read the current tempurature and atmospheric pressure values

  double T = temp + 273;
   
  num = log(pres/Pb) * T * R;
  dnum = g * M * -1;                   // used predefined constants to calculated altitude.
  h = (num/dnum)+ hb;
   
  return(h);                    // return altitude value
}

int *Sensor::Axis_xyz()
{
  static int Axis[3];        // created a static array to hold output
  
  Vector norm = mpu.readNormalizeGyro();         // read in gyroscope values and accelerometer values
  Vector normAccel = mpu.readNormalizeAccel();   // as 1x3 vectors
   
  gpitch = gpitch + norm.YAxis * timeStep;
  groll = groll + norm.XAxis * timeStep;        // extract xyz rate values and calcutate position values
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
  Axis[1] = groll;                    // save pitch roll and yaw values to array
  Axis[2] = gyaw;
   
  return(Axis);                        // return array
}

