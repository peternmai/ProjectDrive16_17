#ifndef DRIVE_H
#define DRIVE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <msg/imu_orientation.h>

#include <string.h>
#include <math.h>

#include "CruiseControl.h"
#include "QuickTurning.h"
#include "WallDetection.h"
#include "util.h"
#include "beam.h"

enum DriveMode {
  cruise,
  reorientate,
  obstacle_avoidance
};

// Constant Variables
#define PUBLISH_RATE 10

static const float MAX_STEERING_LEFT  =  M_PI / 2;
static const float MAX_STEERING_RIGHT = -M_PI / 2;

static const float MIN_RADIUS_LASER_SCAN_FILTER = 0.3;

#endif
