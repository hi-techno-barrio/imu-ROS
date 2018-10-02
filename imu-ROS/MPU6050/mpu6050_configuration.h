#ifndef _MPU6050_CONFIGURATION_H_
#define _MPU6050_CONFIGURATION_H_

#include "imu_wire.h"

// Set the baud rate
#define BAUD  57600

// Use advanced teensy i2c library from https://github.com/nox771/i2c_t3
// You can't have both
// #define WIRE_T3

// IMU Configuration
// Select your IMU board from the list below.
// If not avaliable, define a custom one but feel free to added it.
//#define SEN10724
//#define GY85
//#define MPU6050
//#define GY80
//#define OTHER


#include "accelgyro.h"


#endif  // _MPU6050_CONFIGURATION_H_

