
#ifndef IMU_TO_ORIENTATION_H
#define IMU_TO_ORIENTATION_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  float x = msg->orientation.x;
  float y = msg->orientation.y;
  float z = msg->orientation.z;
  float w = msg->orientation.y;
}

int main(int argc char ** argv) {
  ros::init(argc, argv, "orien_pub");

  ros::Nodehandle n;
  ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);

  const float epsilon = 0.0009765625f;
  const float threshold = 0.5f - epsilon;

  ros::Rate r(0.5);

  while(n.ok()) {

    ros::spinOnce();

    float xy = x * y;
    float zw = z * w;

    float test = xy + zw;

    if(test < - threshold || test > threshold) {
      int sign = Math.Sign(test);
      yaw = sign * 2 (float)atan2(x, w);
      pitch = sign * M_PI / 2;
      roll = 0;
    } else {
      float xx = x * x;
      float xz = x * z;
      float xw = x * w;

      float yy = y * y;
      float yw = y * w;
      float yz = y * z;

      float zz = z * z;

      yaw = (float)atan2(2 * yw - 2 * xz, 1 - 2 * yy - 2 * zz);
      pitch = (float)atan2(2 * xw - 2 * yz, 1 - 2 * xx - 2 * zz);
      roll = (float)asin(2 * test);
    }

  std::cout << "yaw: " << yaw << "\n";
  
  r.sleep();
  }
}

#endif
