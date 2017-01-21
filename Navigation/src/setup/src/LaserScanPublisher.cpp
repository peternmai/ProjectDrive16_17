/**
 * Name: LaserScanPublisher.cpp
 * Description: This program is meant to translate input information retrieved
 *              from our sensor into the format accepted by ROS Navigation Stack
 *              and send it back to ROS. The input has to fit the LaserScan
 *              Message which could be found in <sensor_msgs/LaserScan.h>.
 *
 * Prerequisite: Ensure ROS is running.
 *
 * Compilation:  Add executable dependencies to ../CMakeLists.txt
 *               cd /<path_to_...>/ProjectDrive16_17/Navigation
 *               catkin_make
 *
 * Run:          rosrun setup publish_laser_scan
 *
 * Effect/Outcome: The program will continuously publish new sensor information
 *                 to ROS environment once every 1/rate second(s).
 *
 * Return: None - Program will run until ROS is terminated.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>

int main( int argc, char ** argv ) {

  std::cout << "Publishing Laser Scan Information..." << std::endl;

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

  // Rate of publish. Publish once per 1/rate second(s).
  ros::Rate r(1.0);

  // Continuously publish new sensor data
  while(n.ok()){

    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    std::cout << scan_time << std::endl;
  
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;            // Laser scan object
    scan.header.stamp = scan_time;          // Specify current time [sec]
    scan.header.frame_id = "laser_frame";   // Specify frame_id
    scan.angle_min = -1.57;                 // Start angle of scan [rad]
    scan.angle_max = 1.57;                  // End angle of scan [rad]
    scan.range_min = 0.0;                   // Minimum range value [m]
    scan.range_max = 100.0;                 // Maximum range value [m]

    // Distance between measurement [rad]
    scan.angle_increment = 3.14 / num_readings;

    // Time between measurements [seconds]
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    
    // Range and intensity data during that one second of collection
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }
  
    // Publish Laser Scan message back to ROS environment
    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}
