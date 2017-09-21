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
#include <string>

#define MAX_LENGTH 5
#define PUBLISH_RATE 3
#define LASER_FREQUENCY 1000
#define MAX_READINGS long(LASER_FREQUENCY / PUBLISH_RATE)

//these are the start and end angles for the area we are looking for
#define lf_start 0.0
#define lf_end M_PI / 2.0
#define lr_start M_PI / 2.0
#define lr_end M_PI

//the first letter represents the side of the area and the second is 
//for forward or reverse 
#define rf_start 3.0 * M_PI / 2.0
#define rf_end 2.0 * M_PI
#define rr_start M_PI
#define rr_end 3.0 * M_PI / 2.0

static double angle_inc;
static int num_reads;
static bool forw = false;

float ranges[MAX_READINGS] = {0};
float angles[MAX_READINGS] = {0};



/* The laserCallback is the data parser for the scan message. These
   functions require the ranges, angles, and angle increment
   variables from the message.
   This function also mods the angles to limit them to 2 * PI */
 
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
  int angle_count = 1;
  C = angleInc;

  for(int i = startIdx; i < endIdx; i++) {
    angle_count = 1;
    a = ranges[i];
    b = ranges[i+1];

    if(a == 0)
      continue;

    while(b == 0) {
      b = ranges[i+angle_count+1];
      angle_count++;
    }

    area += (b * a * sin(C * angle_count) / 2);
  }
  return area;
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
  double str_angle = 0, end_angle = 0;

  if(forw) {
    str_angle = rf_end - 0.15;
    end_angle = rf_end;
  } else {
    str_angle = rr_end - 0.15;
    end_angle = rr_end;
  }

  for(int i = num_reads; i > 0; i--) {
      if(angles[i] > str_angle && angles[i] < end_angle)
        return i;
  }
  return REnd;
}

int findRStart(int REnd) {
  int RStart = 0;
  double str_angle = 0, end_angle = 0;

  if(forw) {
    str_angle = rf_start ;
    end_angle = rf_start + 0.1;
  } else {
    str_angle = rr_start;
    end_angle = rr_start + 0.1;
  }

  for(int i = 0; i < REnd; i++) {
    if(angles[i] > str_angle && angles[i] < end_angle)
      RStart = i;
  }
  return RStart;
}

int findLEnd() {
  int LEnd = 0;
  double str_angle = 0, end_angle = 0;

  if(forw) {
    str_angle = lf_end - 0.15;
    end_angle = lf_end;
  } else {
    str_angle = lr_end - 0.15;
    end_angle = lr_end;
  }

  for(int i = num_reads; i > 0; i--) {
    if(angles[i] > str_angle && angles[i] < end_angle)
      return i;
  }
  return LEnd;
}

int findLStart(int LEnd) {
  int LStart = 0;
  double str_angle = 0, end_angle = 0;

  if(forw) {
    str_angle = lf_start;
    end_angle = lf_start + 0.1;
  } else {
    str_angle = lr_start;
    end_angle = lr_start + 0.1;
  }
  for(int i = 0; i < LEnd; i++) {
    if(angles[i] > str_angle && angles[i] < end_angle)
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

/*
  This function returns the results of all the calculations performed
  in these functions. start and end indicies, the area, and the
  suggested steering angle are the results of this funciton.
*/
void displayResults(double left, double right, double total, double diff) {
  std::string dir;
  double l_start = 0, l_end = 0;
  double r_start = 0, r_end = 0;

  if(forw)
    dir = "forward";
  else
    dir = "reverse";

  if(forw) {
    l_start = lf_start;
    l_end = lf_end;
    r_start = rf_start;
    r_end = rf_end;
  } else {
    l_start = lr_start;
    l_end = lr_end;
    r_start = rr_start;
    r_end = rr_end;
  }

  std::cout << "The car is currently going: " << dir << "\n";
  std::cout << "Measuring the left area from: " << l_start << " to: ";
  std::cout << l_end << " has an area of: " << left << "\n";
  std::cout << "l_start: " << findLStart(findLEnd());
  std::cout << " l_end: " << findLEnd() << "\n";
  std::cout << "Measuring the right area from: " << r_start << " to: ";
  std::cout << r_end << " has an area of: " << right << "\n";
  std::cout << "r_start: " << findRStart(findREnd());
  std::cout << " r_end: " << findREnd() << "\n";
  std::cout << "There is a total area of: " << total << "\n";
  std::cout << "The suggested steering angle is: " << M_PI / 2.0 * diff;
  std::cout << "\n";
}

/*
  This is the main function. It requires a laserscan message.
  It parses the laser scan message for ranges, angles,
  angle incremenets, and the direction the vehicle is heading.
  Lastly, it returns a recommended steering angle
*/
double LaneKeepingAssistance(const 
  sensor_msgs::LaserScan::ConstPtr& msg, bool forward) {
  
  num_reads = msg->ranges.size();
  forw = forward;

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

//  displayResults(L_Area, R_Area, T_Area, diff);

  double steering_angle = 0;

  if(forward)
   steering_angle = M_PI / 2.0 * diff;
  else
   steering_angle = M_PI / 2.0 * diff * -1;

  return steering_angle;
}

#endif
