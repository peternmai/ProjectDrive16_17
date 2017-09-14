/** This is the header file for the centering portion of our
    navigation algorithm. Our approach to centering the
    vehicle starts with measuring the area from 0 to pi/2
    radians and the area from pi/2 to pi radians. After
    comparing the left and right areas in front of the
    vehicle, it makes a suggestion to move towards the side
    that has a greater area which would abstractly represent a
    space that is more open.

    Requirements: subscription to the Scan Messages using the
    laserCallback function

    Main functions: findLArea() and findRArea()
    Helper functions: findArea() findRStart() findREnd()
                      findLStart() findLEnd()

    Outputs: the area to the upper left and upper right quadrant in
    front of the vehicle.

 */
#ifndef LANEKEEPING_ASSISTANCE_H
#define LANEKEEPING_ASSISTANCE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include <iostream>
#include <stdlib.h>

#define MAX_LENGTH 5
#define PUBLISH_RATE 3
#define LASER_FREQUENCY 1000
#define MAX_READINGS long(LASER_FREQUENCY / PUBLISH_RATE)

static double angle_inc;
static int num_reads;

static float ranges[MAX_READINGS] = {0};
static float angles[MAX_READINGS] = {0};

/* The laserCallback is the data parser for the scan message. These
   functions require the ranges, angles, and angle increment
   variables from the message.
   This function also mods the angles to limit them to 2 * PI
 
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  num_reads = msg->ranges.size();
  for(int i = 0; i < num_reads; i++) {
    ranges[i] = msg->ranges[i] > 5 ? 5 : msg->ranges[i];
    angles[i] = msg->angle_min + msg->angle_increment * i;
    angle_inc = msg->angle_increment;
  }

  for(int i = 0; i < num_reads; i++) {
    double curr = angles[i];
    while(curr > 2 * M_PI) {
     curr -= 2 * M_PI;
    }
    angles[i] = curr;
  }
}
*/

/* This function finds the area of the triangle by calculating the
   base * height. The base can be chosen but the height may not be
   already given, so we find the height with a*sin(C) where a and b
   are the ranges we took from the scan message earlier and C being
   the angle_increment.
 */
double findArea(int startIdx, int endIdx, double angleInc) {
  double a, b;
  double C;
  double area = 0;
  C = angleInc;

  for(int i = startIdx; i < endIdx; i++) {
    a = ranges[i];
    b = ranges[i+1];

    if(a == 0 && i != 0)
      a = ranges[i-1];

    if(b == 0 && i+2 < endIdx)
      b = ranges[i+2];

    area += (b * a * sin(C) / 2);
  } 
}

/* The next collection of functions are to help find the indicies of
   the start and finish of the left and right areas. We need these 
   indicies to find the left and right areas. So, to find complete
   areas, we take the latest end indicies and find their cooresponding
   start indicies. This ensures that the area is nearly a complete 
   90 degrees before taking the area.

 */
int findREnd() {
  int REnd = 0;
  for(int i = 0; i < sizeof(angles); i++) {
    if(angles[i] < 5.85 && angles[i] > 5.7)
     REnd =  i; 
  }
  return REnd;
}

int findRStart(int REnd) {
  int RStart = 0;
  for(int i = 0; i < REnd; i++) {
    if(angles[i] > 3 * M_PI / 2 && angles[i] < 4.8)
      RStart = i;
  }
  return RStart;
}

int findLEnd() {
  int LEnd = 0;
  for(int i = 0; i < sizeof(angles); i++) {
    if(angles[i] < M_PI / 2 && angles[i] > 1.5)
      LEnd = i;
  }
  return LEnd;
}

int findLStart(int LEnd) {
  int LStart = 0;
  for(int i = 0; i < LEnd; i++) {
    if(angles[i] < 0.65 && angles[i] > 0.5)
      LStart = i;
  }
  return LStart;
}

/* These functions are the main functions of this file. They find the
   left and right area by calling the other helper functions
 */
double findLArea() {
  int LEnd = findLEnd();
  int LStart = findLStart(LEnd);

  return findArea(LStart, LEnd, angle_inc);
}

double findRArea() {
  int REnd = findREnd();
  int RStart = findRStart(REnd);

  return findArea(RStart, REnd, angle_inc);
}

double LaneKeepingAssistance(const 
       sensor_msgs::LaserScan::ConstPtr& msg) {
  num_reads = msg->ranges.size();

  for(int i = 0; i < num_reads; i++) {
    ranges[i] = msg->ranges[i] > 5 ? 5 : msg->ranges[i];
    angles[i] = msg->angle_min + msg->angle_increment * i;
    angle_inc = msg->angle_increment;
  }

  for(int i = 0; i < num_reads; i++) {
    double curr = angles[i];
    while(curr > 2 * M_PI) {
      curr -= 2 * M_PI;
    }
  angles[i] = curr;
  }

  double L_Area = findLArea();
  double R_Area = findRArea();
  double T_Area = L_Area + R_Area;

  double diff = (L_Area - R_Area) / T_Area;

  std::cout << "L_AREA: " << L_Area << " R_AREA: " << R_Area;
  std::cout << " T_AREA: " << T_Area << " diff: " << diff << "\n";
  std::cout << "Output: " << M_PI / 2 * diff << "\n";
  
  return M_PI / 2 * diff;
}

#endif
