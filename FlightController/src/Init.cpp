#include "init.h"
#include "motors.h"

#include <Wire.h>
#include <Servo.h>
#include <BME280I2C.h>
#include <MPU6050.h>

Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
MPU6050 mpu;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
                  
void Initialise::init_sensors()
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

void Initialise::init_motors()
{
  Motors motor;
  mpu.calibrateGyro();
  mpu.setThreshold(10);
  
  Motor1.attach(M1);
  Motor2.attach(M2);
  Motor3.attach(M3);
  Motor4.attach(M4);
  Motor5.attach(M5);
  Motor6.attach(M6);
  
  motor.RunMotors(&Motor1,1000);
  motor.RunMotors(&Motor2,1000);
  motor.RunMotors(&Motor3,1000);
  motor.RunMotors(&Motor4,1000);
  motor.RunMotors(&Motor5,1000);
  motor.RunMotors(&Motor6,1000);
  delay(5000);
}
