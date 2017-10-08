#ifndef DRIVE_H
#define DRIVE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <msg/imu_orientation.h>
#include <msg/vehicle_status.h>

#include <string.h>
#include <math.h>

#include "CruiseControl.h"
#include "QuickTurning.h"
#include "WallDetection.h"
#include "UTurnDetection.h"
#include "ObstacleAvoidance.h"
#include "util.h"


enum DriveMode {
  cruise,
  reorientate,
  obstacle_avoidance
};

// Constant Variables
#define PUBLISH_RATE 10

#define CRUISE_FRONT_AREA_THRESHOLD 100
#define MIN_BACKUP_TIME 1.75 //seconds

static const float MIN_RADIUS_LASER_SCAN_FILTER = 0.2;

static const BoxCoordinates rightBuffer = {0.1, 0.25, -0.15, 0.3};
static const BoxCoordinates leftBuffer  = {-0.25, 0.1, -0.15, 0.3};
static const BoxCoordinates frontBuffer = {-0.14, 0.14, 0.1, 0.4};

#endif
