#ifndef _ACCELGYRO_MPU6050_H_
#define _ACCELGYRO_MPU6050_H_

#include "accelgyro.h"
//#include <Wire.h>      // lets check this
#include <MPU6050J.h>   // lets check this

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
/*
void checkSettings()
{
  Serial.println();
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  Serial.println(mpu.getClockSource());
  Serial.print(" * Accelerometer:         ");
  Serial.println(mpu.getRange());
  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}
*/

bool initMPU6050()
{
	  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
      {
         Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
          delay(500);
	    }    
      
	if (mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
       {
            mpu.calibrateGyro();
         // Set threshold sensivty. Default 3.
         // If you don't want use threshold, comment this line or set 0.
            mpu.setThreshold(3);
        //    checkSettings();
          return true;
       }
       else
         {
		  return false; 
         }
}

geometry_msgs::Vector3 readIMUaccelerometer()
{
    Vector normAccel = mpu.readNormalizeAccel();
    acceleration.x = normAccel.XAxis;
    acceleration.y = normAccel.YAxis;
    acceleration.z = normAccel.ZAxis;
    return acceleration;
}


geometry_msgs::Vector3 readIMUgyroscope()
{
  Vector norm = mpu.readNormalizeGyro();
  
  timer = millis();
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  // Output raw
  angular_velocity.x= pitch;
  angular_velocity.y =roll;  
  angular_velocity.z=yaw;
  
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
  return angular_velocity;
}

geometry_msgs::Vector3 readIMUmagnetometer()
{
      magnetic_field.x = 0.00; 
      magnetic_field.y = 0.00; 
      magnetic_field.z = 0.00;
     
    return magnetic_field;
}



#endif  // _ACCELGYRO_MPU6050_H_
