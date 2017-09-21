/*
  This is the header file for imu to orientation. The purpose of
  these files are to convert the information given by the imu to
  pitch, roll, and yaw. If we have pitch, roll, and yaw, we will
  be able to find the current orientation of the car. 
*/

#ifndef IMU_TO_ORIENTATION_H
#define IMU_TO_ORIENTATION_H


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <msg/imu_orientation.h>
#include <tf/transform_datatypes.h>

float old[30] = {0};
int it_idx = 0;
int cont = 0;
float yaw = 0;
float prev_yaw = 0;
float avg_orien = 0;
ros::Time r_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

#endif
