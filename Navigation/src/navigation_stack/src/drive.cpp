/**
 * Name: Navigation.cpp
 * Description: This is the main driver for the autonomous RC vehicle. This
 *              program will subscribe to <sensor_msgs/LaserScan>. It will
 *              process the data, and guide the vehicle in the forward
 *              direction, avoiding any obstacles that are lying ahead.
 *
 * Effect/Outcome: Publish AckermannDrive commands that will set the vehicle's
 *                 throttle and steering controls.
 *
 * Return: None - Program will run until ROS is terminated.
 */

#include "drive.h"

static float speed = 0;
static float steering = 0;
static long  pubCount = 0;

static void printProgressBar( std::string title, float percentage ) {
  if( percentage < 0 )
    std::cout << title << ": [" << std::string( 50 - (int) (-50 * percentage), ' ' )
              << std::string( (int) (-50 * percentage) + 1, '=' )
              << std::string( 50, ' ') << "] " << percentage * 100 << "%" << std::endl;
  else
    std::cout << title << ": [" << std::string( 50, ' ')
              << std::string( (int) (50 * percentage) + 1, '=' )
              << std::string( 50 - (int) (50 * percentage), ' ') << "] " 
              << percentage * 100 << "%" << std::endl;
}

static void VisualizeAckermannDrive( float throttle, float steering ) {
  printProgressBar( "THROTTLE", speed / 1 );
  printProgressBar( "STEERING", steering / (M_PI/2));
}


static void LaserScanCallback( const sensor_msgs::LaserScan::ConstPtr& scan ) {
  speed = GetCruiseSpeed( scan, 1, 5 );
  steering = LaneKeepingAssistance( scan );
  std::cout << "Steering: " << steering << std::endl;
}

int main( int argc, char ** argv ) {

  // Announce the name of this node to ROS
  ros::init(argc, argv, "APPOLO_DRIVER");

  // Create publisher and subscribers to be used later
  ros::NodeHandle nh;
  ros::Publisher AckermannDrivePublisher =  
    nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1);
  ros::Subscriber LaserScanSubscriber = 
    nh.subscribe("scan", 1, LaserScanCallback );

  ros::Rate r( PUBLISH_RATE );

  // Continuously publish new sensor data
  while(nh.ok()) {
    system("clear");
    ros::spinOnce();

    ackermann_msgs::AckermannDrive AckermannDrive;
    AckermannDrive.steering_angle = steering;
    AckermannDrive.steering_angle_velocity = 0;
    AckermannDrive.speed = speed;
    AckermannDrive.acceleration = 0;
    AckermannDrive.jerk = 0;

    ackermann_msgs::AckermannDriveStamped AckermannDriveStamped;
    AckermannDriveStamped.header.stamp = ros::Time::now();
    AckermannDriveStamped.header.frame_id = std::to_string( pubCount++ );
    AckermannDriveStamped.drive = AckermannDrive;

    AckermannDrivePublisher.publish( AckermannDriveStamped );

    VisualizeAckermannDrive( speed, steering );
    
    r.sleep();
  }

  return 0;
}
