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
#include <math.h>

#include <iostream>
#include <stdlib.h>

// Constant Variables
#define PUBLISH_RATE      3
#define LASER_FREQUENCY   1000
#define MAX_READINGS      long(LASER_FREQUENCY / PUBLISH_RATE)

// Shared Static Variables
static float         sensor_ranges[ MAX_READINGS ] = {0};
static float         sensor_angles[ MAX_READINGS ] = {0};
static ros::Time     sensor_times[  MAX_READINGS ] = {(ros::Time) 0 };

static float         sensor_range_min = 0;
static float         sensor_range_max = 100;

static ros::Time     teraranger_start_time = (ros::Time) 0;
static ros::Time     teraranger_latest_time = (ros::Time) 0;

static ros::Time     encoder_start_time = (ros::Time) 0;
static ros::Time     encoder_latest_time = (ros::Time) 0;

static float         encoder_start_angle = 0;
static float         encoder_latest_angle = 0;

static float         encoder_start_angular_velocity = 0;
static float         encoder_latest_angular_velocity = 0;

static unsigned int  teraranger_callback_instance = 0;
static unsigned int  optical_encoder_callback_instance = 0;

static float         min_angular_velocity = 12;
static float         max_angular_velocity = 18;

static float         angle_offset = M_PI;

// Since teraranger's distance readings arent broadcasted at a consistent
// rate, this array will hold the range readings with respect to time. Each
// increment in the array has an adjusted time increment of scan duration
// divided by the number of scans.
static float         sensor_adjusted_ranges[ MAX_READINGS ] = {0};

// Calcualte the angle at the specified teraranger time
static float calculateAngle( const ros::Time & teraranger_time, const ros::Time
& optical_encoder_time, const float & encoder_angle, const float &
angular_velocity ) {

  ros::Duration timeDiff = teraranger_time - optical_encoder_time;
  float angleDiff = angular_velocity * timeDiff.toSec();

  if(encoder_angle + angleDiff < 0)
    return encoder_angle + angleDiff + 2 * M_PI;
  return (encoder_angle + angleDiff);
}

// Retrieve information from optical encoder to get sensor angle information
static void opticalEncoderCallback( const msg::optical_encoder::ConstPtr& msg ) {

  // Calculate the current angle with the offset
  float cur_angle = msg->angle + angle_offset;
  if( cur_angle >= (2 * M_PI))
    cur_angle -= 2 * M_PI;

  if( optical_encoder_callback_instance == 0 ) {
    encoder_start_angular_velocity = msg->avg_angular_velocity;
    encoder_start_angle = cur_angle;
    encoder_start_time  = msg->time;
  }

  encoder_latest_angular_velocity = msg->avg_angular_velocity;
  encoder_latest_angle = cur_angle;

  // std::cout << encoder_latest_angular_velocity << std::endl;

  encoder_latest_time  = msg->time;
  optical_encoder_callback_instance++;
}

// Retrieve information from TeraRanger One Sensor
static void terarangeroneCallback( const sensor_msgs::Range::ConstPtr& msg ) {
  
  // Discard newer readings if receiving more than expected max readings
  if( teraranger_callback_instance >= MAX_READINGS )
    return;

  if( teraranger_callback_instance == 0 ) {
    teraranger_start_time = msg->header.stamp;
  }
  else
    teraranger_latest_time = msg->header.stamp;

  sensor_times[  teraranger_callback_instance ] = msg->header.stamp;
  sensor_ranges[ teraranger_callback_instance ] = msg->range;
  sensor_angles[ teraranger_callback_instance ] = calculateAngle( teraranger_latest_time,
    encoder_latest_time, encoder_latest_angle, encoder_latest_angular_velocity);
 
  sensor_range_min = msg->min_range;
  sensor_range_max = msg->max_range;
  teraranger_callback_instance++;
  
}

static float calculateStartAngle() {
  if ( teraranger_callback_instance > 0 )
    return sensor_angles[0];
  else
    return 0;
}

static float calculateEndAngle() {

  // Count restart in rotation
  int rotation_restart_counter = 0;
  for( int i = 1; i < teraranger_callback_instance; i++ )
    if( sensor_angles[i] + M_PI < sensor_angles[i-1] )
      rotation_restart_counter++;

  if ( teraranger_callback_instance > 0 )
    return sensor_angles[ teraranger_callback_instance - 1 ] +
    rotation_restart_counter * M_PI * 2;
  else
    return 0;
}

static void generateAdjustedRanges() {

  int index, i;

  /**
  for( i = 0; i < teraranger_callback_instance; i++ )
    if( sensor_angles[i] < 0.0873 || sensor_angles[i] > 6.196)
      sensor_ranges[i] = 10;
  **/

  if( teraranger_callback_instance > 0 ) {
    // Calculate the seconds between each range reading  
    float timeBetweenRange = ( sensor_times[ teraranger_callback_instance - 1 ] -
                  sensor_times[0] ).toSec() / MAX_READINGS;
    
    //std::cout << "Time between range: " << timeBetweenRange << std::endl;
    // Clear data in previous array
    for( i = 0; i < MAX_READINGS; i++ )
      sensor_adjusted_ranges[i] = 0;

    // Go through each range reading and set to adjust ranges
    for( i = 0; i < teraranger_callback_instance; i++ ) {
      index = (sensor_times[i] - sensor_times[0]).toSec() / timeBetweenRange;
      if( index >= 0 && index < MAX_READINGS )
        sensor_adjusted_ranges[index] = sensor_ranges[i];
      //else
      //  std::cout << "Bad attempt to write tera reading " << i <<" to adjusted "
      //          << "range array index " << index << std::endl;
    }
  }
}

static void displayData( const sensor_msgs::LaserScan & data ) {
  
/**
  // Clear Screen
  for( int i = 0; i < 5; i++ )
    std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
**/
  system("clear");

  // Figure out max time difference between scan
  ros::Duration maxTimeDiff = ros::Time::now() - ros::Time::now();
  for( int i = 1; i < teraranger_callback_instance; i++ )
    if( sensor_times[i] - sensor_times[i-1] > maxTimeDiff )
      maxTimeDiff = sensor_times[i] - sensor_times[i-1];

  std::cout << "Publishing Laser Scan Information..." << std::endl;
  std::cout << data.header.stamp << " | Total Scan Duration: " << 
  data.scan_time << " seconds" << std::endl << std::endl;

  std::cout << "\tStart Angle:\t\t" << data.angle_min << " rad\n";
  std::cout << "\tEnd Angle:\t\t"   << data.angle_max << " rad\n"  <<std::endl;

  std::cout << "\tAngle Increment:\t" << data.angle_increment << " rad\n";
  std::cout << "\tAvg Increment Time:\t" << data.time_increment << " sec\n";
  std::cout << "\tMax Increment Time:\t" << maxTimeDiff.toSec() << " sec\n";
  std::cout << "\tAngular Velocity:\t" <<
    (data.angle_max - data.angle_min) * PUBLISH_RATE << " rad/sec\n" << std::endl;

  std::cout << "\t# of Distance Readings:\t" << teraranger_callback_instance << std::endl;
  std::cout << "\t# of Angle Readings:\t" << optical_encoder_callback_instance << std::endl;

  std::cout << "\nDisplaying scan data..." << std::endl;
  std::cout << "\tINDEX\t\tTIME\t\t\tANGLE\t\tDISTANCE" << std::endl;
  if( teraranger_callback_instance > 0 ) {
    //TODO change back to printing every 10
    for( int i = 0; i < 1; i++) {//(teraranger_callback_instance-1); i += 1){
      std::cout << "\t" << i << "\t\t" << sensor_times[i] << "\t";
      std::cout << sensor_angles[i] << "\t\t" << sensor_ranges[i] << std::endl;
    }
    
    // Print last reading
    std::cout << "\t" << (teraranger_callback_instance-1) << "\t\t" <<
      sensor_times[ teraranger_callback_instance-1 ] << "\t" <<
      sensor_angles[ teraranger_callback_instance-1 ] << "\t\t" <<
      sensor_ranges[ teraranger_callback_instance-1 ] << std::endl;
  }
  else
    std::cout << "\tNO DATA\t\tNO DATA\t\t\tNO DATA\t\tNO DATA" << std::endl;
}

int main( int argc, char ** argv ) {

  // Announce to ROS this program as a node called "laser_scan_publisher"
  ros::init(argc, argv, "laser_scan_publisher");

  // Create publisher and subscribers to be used later and send LaserScan to ROS
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Subscriber teraranger_sub = n.subscribe("terarangerone", MAX_READINGS,
    terarangeroneCallback );
  ros::Subscriber optical_encoder_sub = n.subscribe("optical_encoder",
    MAX_READINGS, opticalEncoderCallback );


  // Rate of publish. Publish once per 1/rate second(s).
  ros::Rate r( PUBLISH_RATE );

  // Continuously publish new sensor data
  while(n.ok()){

    // Retrive data from sensors being used
    ros::Time scan_time = ros::Time::now();
    ros::spinOnce();

    // Adjust angle ranges so that it's time increment is consistent
    generateAdjustedRanges();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;            // Laser scan object
    scan.header.stamp = sensor_times[0];    // Specify first reading time [sec]
    scan.header.frame_id = "laser_frame";   // Specify frame_id
    scan.angle_min = calculateStartAngle(); // Start angle of scan [rad]
    scan.angle_max = calculateEndAngle();   // End angle of scan [rad]
    scan.range_min = sensor_range_min;      // Minimum range value [m]
    scan.range_max = sensor_range_max;      // Maximum range value [m]

    // Distance between measurement [rad]
    scan.angle_increment = (scan.angle_max - scan.angle_min) / MAX_READINGS;

    // Total Scan Time [seconds]
    scan.scan_time = teraranger_latest_time.toSec() - teraranger_start_time.toSec();

    // Time between measurements [seconds]
    scan.time_increment = scan.scan_time / MAX_READINGS;
    
    scan.ranges.resize( MAX_READINGS );
    for(unsigned int i = 0; i < MAX_READINGS; ++i)
      scan.ranges[i] = sensor_adjusted_ranges[i];

    //std::cout << (scan.angle_max - scan.angle_min) << " | " << encoder_latest_angular_velocity << std::endl;

    for(unsigned int i = 0; i < MAX_READINGS; i++ )
      if( sensor_angles[i] < 0.08 || sensor_angles[i] > 6.19 )
        scan.ranges[i] = 5;

    displayData( scan );
    teraranger_callback_instance = 0;
    optical_encoder_callback_instance = 0;
  
    // Publish Laser Scan message back to ROS environment
    scan_pub.publish(scan);
    r.sleep();
  }
}
