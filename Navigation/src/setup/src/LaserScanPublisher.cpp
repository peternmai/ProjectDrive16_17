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
#include <sensor_msgs/Range.h>

#include <iostream>

// Constant Variables
#define NUM_READINGS    100
#define LASER_FREQUENCY  40

// Shared Static Variables
static float         sensor_ranges[ NUM_READINGS ] = {0};
static unsigned int  callback_instance = 0;

// Retrieve information from TeraRanger One Sensor
static void terarangeroneCallback( const sensor_msgs::Range::ConstPtr& msg ) {
  sensor_ranges[ callback_instance ] = msg->range;
  callback_instance = ( callback_instance + 1 ) % NUM_READINGS;
}

int main( int argc, char ** argv ) {

  ROS_INFO("Publishing Laser Scan Information...");

  // Announce to ROS this program as a node called "laser_scan_publisher"
  ros::init(argc, argv, "laser_scan_publisher");

  // Create publisher and subscribers to be used later and send LaserScan to ROS
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser_scan", 50);
  ros::Subscriber teraranger_sub = n.subscribe("terarangerone", NUM_READINGS,
    terarangeroneCallback );


  // Rate of publish. Publish once per 1/rate second(s).
  ros::Rate r(1.0);

  // Continuously publish new sensor data
  while(n.ok()){

    // Retrive data from sensors being used
    ros::Time scan_time = ros::Time::now();
    callback_instance = 0;
    ros::spinOnce();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;            // Laser scan object
    scan.header.stamp = scan_time;          // Specify current time [sec]
    scan.header.frame_id = "laser_frame";   // Specify frame_id
    scan.angle_min = -1.57;                 // Start angle of scan [rad]
    scan.angle_max = 1.57;                  // End angle of scan [rad]
    scan.range_min = 0.0;                   // Minimum range value [m]
    scan.range_max = 100.0;                 // Maximum range value [m]

    // Distance between measurement [rad]
    scan.angle_increment = 3.14 / NUM_READINGS;

    // Time between measurements [seconds]
    scan.time_increment = (1 / LASER_FREQUENCY) / (NUM_READINGS);
    
    // Range and intensity data during that one second of collection
    scan.ranges.resize(NUM_READINGS);
    scan.intensities.resize(NUM_READINGS);
    for(unsigned int i = 0; i < NUM_READINGS; ++i)
      scan.ranges[i] = sensor_ranges[i];
  
    // Publish Laser Scan message back to ROS environment
    scan_pub.publish(scan);
    r.sleep();
  }
}
