#include "inits.h"
#include "motors.h"
#include "sensors.h"

#include <Wire.h>
#include <BME280I2C.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <FIR.h>

extern MPU6050 mpu;      // extern allows other files to use these values
extern BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                         // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
extern HMC5883L compass;  
extern FIR fir;

Sensors::Sensors()
{
  fir.setCoefficients(coef);
  fir.setGain(gain);
}

double Sensors::ALT()
{
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa); 
          
  bme.read(pres, temp, hum, tempUnit, presUnit);  //read the current tempurature and atmospheric pressure values

  T = temp + 273;
   
  num = log(pres/Pb) * T * R;
  dnum = g * M * -1;                  // used predefined constants to calculated altitude.
  h = fir.process((num/dnum)+ hb);    // here we call the fir routine with the input. The value 'fir' spits out is stored in the output variable.
  return(h);             // return altitude value
}

double *Sensors::IMU()
{
  static double Axis[3];        // created a static array to hold output
  
  Vector normGyro = mpu.readNormalizeGyro();        // read in gyroscope values and accelerometer values
  Vector normAccel = mpu.readNormalizeAccel();  // as 1x3 vectors
   
  gpitch = gpitch + normGyro.YAxis * timeStep;
  groll = groll + normGyro.XAxis * timeStep;        // extract xyz rate values and calcutate position values
  gyaw = normGyro.ZAxis;
   
  apitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  aroll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
   
  if(set_gyro_angles)
  {                                               // if the IMU is already started
    gpitch = gpitch * 0.995 + apitch * 0.005;     // correct the drift of the gyro pitch angle with the accelerometer pitch angle
    groll = groll * 0.995 + aroll * 0.005;        // correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {                                               // at first start
    gpitch = apitch;                              // set the gyro pitch angle equal to the accelerometer pitch angle 
    groll = aroll;                                // set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                       // set the IMU started flag
  }
   
                                                                   // to dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 +  gpitch * 0.1;   // take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 +  groll * 0.1;      // take 90% of the output roll value and add 10% of the raw roll value
   
  Axis[0] = angle_pitch_output;
  Axis[1] = angle_roll_output;         // save pitch roll and yaw values to array
  Axis[2] = gyaw;
   
  return(Axis);                        // return array
}

double Sensors::MAG()
{
}
