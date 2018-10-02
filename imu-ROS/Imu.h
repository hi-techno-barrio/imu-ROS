#ifndef _IMU_H_
#define _IMU_H_

#ifdef GY85_IMU
    #include "GY85/gy85_configuration.h"
#endif

#ifdef MPU6050_IMU
   // #include "MP6050/test.h"
    #include "MPU6050/mpu6050_configuration.h"
#endif

bool initIMU()
{
    Wire.begin();
    delay(5);
    bool accel, gyro, mag, mpu;
    #ifdef GY85_IMU
      accel = initAccelerometer();
      gyro  = initGyroscope();   
      mag   = initMagnetometer();
          if(accel && gyro && mag)
            return true;
           else
             return false;
     #endif
    
     #ifdef MPU6050_IMU
          mpu = initMPU6050();
            if(mpu )
              return true;
            else
                return false;
      #endif

}

geometry_msgs::Vector3 readAccelerometer()
{
    return readIMUaccelerometer();
}

geometry_msgs::Vector3 readGyroscope()
{
    return readIMUgyroscope();
}


geometry_msgs::Vector3 readMagnetometer()
{
    return readIMUmagnetometer();
}


#endif
