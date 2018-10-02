#ifndef _ACCELGYRO_H_
#define _ACCELGYRO_H_

bool initMPU6050();

geometry_msgs::Vector3 readIMUaccelerometer();
geometry_msgs::Vector3 acceleration;

geometry_msgs::Vector3 readIMUgyroscope();
geometry_msgs::Vector3 angular_velocity;

geometry_msgs::Vector3 readIMUmagnetometer();
geometry_msgs::Vector3 magnetic_field;



#include "accelgyro_MPU6050.h"

#endif  // _ACCELGYRO_H_
