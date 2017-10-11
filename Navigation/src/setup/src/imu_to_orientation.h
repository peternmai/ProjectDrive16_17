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
#include <msg/vehicle_status.h>
#include <tf/transform_datatypes.h>
#include <list>

float yaw = 0;
float prev_yaw = 0;
float cont = 0;
float avg_orien = 0;
float old_avg = 0;
std::list<float> o;
std::list<float> scaled_o;
//float old[30];
//int it_idx = 0;
ros::Time r_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

void modeCallback(const msg::vehicle_status::ConstPtr & msg);

#endif
