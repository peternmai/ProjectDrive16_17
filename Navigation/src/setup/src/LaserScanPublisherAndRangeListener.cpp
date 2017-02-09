#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

const unsigned int NUM_READINGS = 100;
float terarangerone_ranges[NUM_READINGS] = {0};
int callback_instance = 0;

void terarangeroneCallback( const sensor_msgs::Range::ConstPtr& msg) {
  terarangerone_ranges[ callback_instance ] = msg->range;
  callback_instance = (callback_instance + 1) % NUM_READINGS;
}

int main (int argc, char **argv) {
  std::cout << "Starting\n";
  ros::init(argc, argv, "laser_scan_publisher_and_range_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("terarangerone", NUM_READINGS, terarangeroneCallback);

  std::cout << "Publishing Laser Scan Information..." << std::endl;
  //ros::init(argc, argv, "laser_scan_publisher");
  ros::Publisher scan_pub =
  n.advertise<sensor_msgs::LaserScan>("laser_scan", NUM_READINGS);

  double laser_frequency = 40;
  double intensities[NUM_READINGS];
  int count = 0;

  ros::Rate r (1.0);

  while (n.ok()) {
    for (unsigned int i = 0; i < NUM_READINGS; i++) {
      intensities[i] = 100 + count;
    }

    ros::spinOnce();
    ros::Time scan_time = ros::Time::now();
    
    std::cout << scan_time << std::endl;
    
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.range_min = 0.0;
    scan.range_max = 100.0;
    
    
    scan.angle_increment = 3.14 / NUM_READINGS;
    scan.time_increment = (1 / laser_frequency) / (NUM_READINGS);
    scan.ranges.resize(NUM_READINGS);
    scan.intensities.resize(NUM_READINGS);
    for(unsigned int i = 0; i < NUM_READINGS; i++) {
      scan.ranges[i] = terarangerone_ranges[i];
      scan.intensities[i] = intensities[i];
    }
    
    scan_pub.publish(scan);
    ++count;
    r.sleep();  
  }
  return 0;
}
