

//#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
//header file for imu
#include "lino_msgs/Imu.h"
#include <ros/ros.h>

using namespace std;

const int I2C_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;

float read_word_2c(int fd, int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}


int main(int argc, char **argv) {

  // Connect to device.
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

  // Start ROS node stuff.
  ros::init(argc, argv, "imu6050");
  ros::NodeHandle node;
 // ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu", 10);
  lino_msgs::Imu raw_imu_msg;
  ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
  // nh.advertise(raw_imu_pub);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {

//this block publishes the IMU static unsigned long prev_control_time = 0;
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;
    
    //data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }
    // Pub & sleep.
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

void publishIMU()
{
    // pass accelerometer data to imu object
    // raw_imu_msg.linear_acceleration = readAccelerometer(); // Read gyroscope values.
    // At default sensitivity of 250deg/s we need to scale by 131.
    raw_imu_msg.angular_velocity.x = read_word_2c(fd, 0x43) / 131;
    raw_imu_msg.angular_velocity.y = read_word_2c(fd, 0x45) / 131;
    raw_imu_msg.angular_velocity.z = read_word_2c(fd, 0x47) / 131;

    // pass gyroscope data to imu object
    // raw_imu_msg.angular_velocity = readGyroscope();
    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
    const float la_rescale = 16384.0 / 9.807;
    raw_imu_msg.linear_acceleration.x = read_word_2c(fd, 0x3b) / la_rescale;
    raw_imu_msg.linear_acceleration.y = read_word_2c(fd, 0x3d) / la_rescale;
    raw_imu_msg.linear_acceleration.z = read_word_2c(fd, 0x3f) / la_rescale;

    //pass accelerometer data to imu object  must be zero!
    //raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}


