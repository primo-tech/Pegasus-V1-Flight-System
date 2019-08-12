#include "sensorRead.h"

#include <Wire.h>
#include <BME280I2C.h>
#include <MPU6050.h>

extern MPU6050 mpu;        // extern allows other files to use these values
extern BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
extern HMC5883L compass;  

Sensors::Sensors()
{
  compass = HMC5883L(); // Construct a new HMC5883 compass.
  error = compass.SetScale(1.3); // Set the scale of the compass.
   if(error != 0) // If there is an error, print it out.
   {
    Serial.println(compass.GetErrorText(error));
   }
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
  {
    Serial.println(compass.GetErrorText(error));   
  }        
}

double Sensors::Altitude()
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

float *Sensors::Axis_xyz()
{
  static float Axis[3];        // created a static array to hold output
  
  Vector norm = mpu.readNormalizeGyro();         // read in gyroscope values and accelerometer values
  Vector normAccel = mpu.readNormalizeAccel();   // as 1x3 vectors
   
  gpitch = gpitch + norm.YAxis * timeStep;
  groll = groll + norm.XAxis * timeStep;        // extract xyz rate values and calcutate position values
  gyaw = gyaw + norm.ZAxis * timeStep;
   
  int Pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int Roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
   
  if(set_gyro_angles)
  {                                                 //If the IMU is already started
    gpitch = gpitch * 0.995 + Pitch * 0.005;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    groll = groll * 0.995 + Roll * 0.005;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
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
   
  Axis[0] = angle_pitch_output;
  Axis[1] = angle_roll_output;                    // save pitch roll and yaw values to array
  Axis[2] = gyaw;
   
  return(Axis);                        // return array
}

float Sensor::Axis_z()
{
  MagnetometerScaled scaled = compass.ReadScaledAxis(); // Retrived the scaled values from the compass (scaled to the configured scale).
  
  heading = atan2(scaled.YAxis, scaled.XAxis); // Calculate heading when the magnetometer is level, then correct for signs of axis.
  
  // Once you have your heading, add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  heading += declinationAngle;
  
  if(heading < 0)   // Correct for when signs are reversed.
  {
    heading += 2*PI;
  }
  
  if(heading > 2*PI) // Check for wrap due to addition of declination.
  {
    heading -= 2*PI;
  }
  
  headingDegrees = heading * 180/M_PI;  // Convert radians to degrees
  
  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L). delay(66);
  
   return(headingDegrees);
}
