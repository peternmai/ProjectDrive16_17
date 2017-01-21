/**
 * Name: LaserScanPublisher.cpp
 * Description: This program is meant to translate input information retrieved
 *              from our sensor into the format accepted by ROS Navigation Stack
 *              and send it back to ROS. The input has to fit the LaserScan
 *              Message which could be found in <sensor_msgs/LaserScan.h>.
 *
 * Prerequisite:
 *
 * Compilation: g++ PublishSensorStream.cpp -o laser_scan_publisher
 * -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -W
 * Run:
 *
 * Effect/Outcome
 *
 * Return:
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main( int argc, char ** argv ) {

  // Announce to ROS this program as a node called "laser_scan_publisher"
  ros::init(argc, argv, "laser_scan_publisher");

  // Create publisher to be used later and send LaserScan to ROS
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  // These informations are pulled from laser driver
  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();
  
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;
   
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }
  
    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}
