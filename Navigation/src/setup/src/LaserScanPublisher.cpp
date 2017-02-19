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
#include <msg/optical_encoder.h>

#include <iostream>

// Constant Variables
#define NUM_READINGS    600
#define LASER_FREQUENCY 600
#define PUBLISH_RATE    1

// Shared Static Variables
static float         sensor_ranges[ NUM_READINGS ] = {0};
static float         sensor_range_min = 0;
static float         sensor_range_max = 100;

static ros::Time     teraranger_start_time = (ros::Time) 0;
static ros::Time     teraranger_end_time = (ros::Time) 0;

static ros::Time     encoder_start_time = (ros::Time) 0;
static ros::Time     encoder_end_time = (ros::Time) 0;

static float         encoder_angle_start = 0;
static float         encoder_angle_end = 0;
static float         encoder_avg_angular_velocity = 0;

static unsigned int  callback_instance = 0;
static unsigned int  optical_encoder_instance = 0;

// Retrieve information from optical encoder to get sensor angle information
static void opticalEncoderCallback( const msg::optical_encoder::ConstPtr& msg ) {

  if( optical_encoder_instance == 0 ) {
    encoder_angle_start = msg->angle;
    encoder_start_time = msg->time;
    encoder_avg_angular_velocity = msg->avg_angular_velocity;
  }

  encoder_angle_end = msg->angle;
  encoder_end_time = msg->time;
  optical_encoder_instance++;
}

// Retrieve information from TeraRanger One Sensor
static void terarangeroneCallback( const sensor_msgs::Range::ConstPtr& msg ) {

  //std::cout << callback_instance << ") ";
  //std::cout << "Actual Time: " << ros::Time::now() << " | Sensor Time: " <<
  //msg->header.stamp << " | Range: " << msg->range << std::endl;

  if( callback_instance == 0 )
     teraranger_start_time = msg->header.stamp;
  else
     teraranger_end_time = msg->header.stamp;
  
  sensor_ranges[ callback_instance ] = msg->range;
  sensor_range_min = msg->min_range;
  sensor_range_max = msg->max_range;
  callback_instance = ( callback_instance + 1 ) % NUM_READINGS;
}

static float calculateStartAngle() {

  ros::Duration timeDiff = teraranger_start_time - encoder_start_time;
  float timeDiffSec = timeDiff.toSec();
  float angleDiff = encoder_avg_angular_velocity * timeDiffSec;

  return (encoder_angle_start + angleDiff);
}

static float calculateEndAngle() {

  float startAngle = calculateStartAngle();
  float timeDiffSec = (encoder_end_time.toSec() - encoder_start_time.toSec() );
  return startAngle + encoder_avg_angular_velocity * timeDiffSec;
}

int main( int argc, char ** argv ) {

  ROS_INFO("Publishing Laser Scan Information...");

  // Announce to ROS this program as a node called "laser_scan_publisher"
  ros::init(argc, argv, "laser_scan_publisher");

  // Create publisher and subscribers to be used later and send LaserScan to ROS
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("laser_scan", 1);
  ros::Subscriber teraranger_sub = n.subscribe("terarangerone", NUM_READINGS,
    terarangeroneCallback );
  ros::Subscriber optical_encoder_sub = n.subscribe("optical_encoder",
    NUM_READINGS, opticalEncoderCallback );


  // Rate of publish. Publish once per 1/rate second(s).
  ros::Rate r( PUBLISH_RATE );

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
    scan.angle_min = calculateStartAngle(); // Start angle of scan [rad]
    scan.angle_max = calculateEndAngle();   // End angle of scan [rad]
    scan.range_min = sensor_range_min;      // Minimum range value [m]
    scan.range_max = sensor_range_max;      // Maximum range value [m]

    // Distance between measurement [rad]
    scan.angle_increment = (scan.angle_max - scan.angle_min) / NUM_READINGS;

    // Time between measurements [seconds]
    //scan.time_increment = (1 / LASER_FREQUENCY) / (NUM_READINGS);
    scan.time_increment = (teraranger_end_time.toSec() - 
                          teraranger_start_time.toSec()) / NUM_READINGS;
    
    // Range and intensity data during that one second of collection
    scan.ranges.resize(NUM_READINGS);
    for(unsigned int i = 0; i < NUM_READINGS; ++i)
      scan.ranges[i] = sensor_ranges[i];
    
    std::cout << "Num optical encoder broadcast: " << optical_encoder_instance
    << std::endl;
    optical_encoder_instance = 0;
  
    // Publish Laser Scan message back to ROS environment
    scan_pub.publish(scan);
    r.sleep();
  }
}
