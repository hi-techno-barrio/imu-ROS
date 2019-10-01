
/*
 IMU:MPU6050 on WiringPi
 Christopher  Coballes
 Hi-Techno Barrio

 */ 


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <string>
using namespace std;

/*
 Declares variables For ROS
 */ 
float theta = 0 ;
float dt;
float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0 ;
float gyro_z_offset = 0.0 ;
float gyro_x,gyro_y,gyro_z ;
int   pub_freq = 10 ;
int   alpha = 0.9  ;
int   counter ;
int   tuningIMU_loop = 60;
bool DEBUG  = 0 ;
ros::Time last_vel_time,current_time;
 float q[3]; 
/*
 Declares variables For MPU6050
 */
const int I2C_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;
/*
 Functions to read 2 bytes = word
 */ 
float read_word_2c(int fd, int addr) {
   int high = wiringPiI2CReadReg8(fd, addr);
   int low = wiringPiI2CReadReg8(fd, addr+1);
   int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

int main(int argc, char **argv) {
  counter = 0;
  // Connect to device.
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

  // Start ROS node stuff.
  ros::init(argc, argv, "IMU_Publisher");
  ros::NodeHandle node;
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Publisher gyro_pub = node.advertise<geometry_msgs::Vector3>("gyro", 10);
 
 
  ros::Rate rate(10);  // hz
  current_time = ros::Time::now();
  last_vel_time = ros::Time::now();
  // Publish in loop.
  while(ros::ok()) {
    sensor_msgs::Imu imu_msgs;
    geometry_msgs::Vector3 gyro_msgs;
    
	/* Read gyroscope values.
	  At default sensitivity of 250deg/s we need to scale by 131.
	*/
	gyro_x = read_word_2c(fd, 0x43) / 131;
	gyro_y = read_word_2c(fd, 0x45) / 131;
	gyro_z = read_word_2c(fd, 0x47) / 131;
	
	/*
	 Read accelerometer values.
	 At default sensitivity of 2g we need to scale by 16384.
	 Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
	  But! Imu msg docs say acceleration should be in m/2 so need to *9.807
	*/
	const float la_rescale = 16384.0 / 9.807;
	imu_msgs.linear_acceleration.x = read_word_2c(fd, 0x3b) / la_rescale;
	imu_msgs.linear_acceleration.y = read_word_2c(fd, 0x3d) / la_rescale;
	imu_msgs.linear_acceleration.z = read_word_2c(fd, 0x3f) / la_rescale;
	
     if (counter < tuningIMU_loop){
                gyro_x_offset += gyro_x;
                gyro_y_offset += gyro_y;
                gyro_z_offset += gyro_z;
                counter += 1;
             } else if ((counter == tuningIMU_loop) && (tuningIMU_loop != 0))
               {
                gyro_x_offset /= tuningIMU_loop ;
                gyro_y_offset /= tuningIMU_loop ;
                gyro_z_offset /= tuningIMU_loop ;
               // ROS_INFO("finished callibrating yaw") ;
                counter+= 1 ;
                }            
      else{   // #publish ros Imu message
                gyro_x -= gyro_x_offset ;
                gyro_y -= gyro_y_offset ;
                gyro_z -= gyro_z_offset ;
                if (DEBUG){
               // ROS_INFO('x %s y %s z %s', gyro_x, gyro_y, gyro_z);
                   }  
                gyro_msgs.x = gyro_x;
                gyro_msgs.y = gyro_y;
                gyro_msgs.z = gyro_z;
                gyro_pub.publish(gyro_msgs);
 
                dt = current_time.toSec() - last_vel_time.toSec();
                theta += dt*gyro_z ;
                
                imu_msgs.header.stamp = ros::Time::now();
                imu_msgs.header.frame_id = '/base_link'  ;             
              //  q = tf::transformations.quaternion_from_euler(0.0, 0.0, theta);
                imu_msgs.orientation.x = q[0];
                imu_msgs.orientation.y = q[1]; 
                imu_msgs.orientation.z = q[2];
                imu_msgs.orientation.w = q[3];
               // imu_msgs.orientation_covariance = [1e6,0,0,0,1e6,0,0,0,1e-6];
                imu_msgs.angular_velocity_covariance[0] = -1;  
                imu_msgs.linear_acceleration_covariance[0] = -1 ;
                imu_pub.publish(imu_msgs) ;
                last_vel_time = current_time;
               
			    }  
					
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

