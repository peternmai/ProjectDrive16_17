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

static DriveMode curMode  = DriveMode::cruise;
static DriveMode prevMode = DriveMode::cruise;
static DriveMode tmpMode  = DriveMode::cruise;

static float speed = 0;
static float steering = 0;
static long  frame_id = 0;

static float currentOrientation         = -1;
static float desiredOrientation         = -1;
static float wallDetectionOrientation   = -1;

static bool reorientateCarForward = true;
static bool obstacleAvoidanceReorientateRequest = false;
static bool requestNewDriveMode = false;

static void VisualizeAckermannDrive( float throttle, float steering ) {
  std::cout << std::endl << "Mode: " << curMode << std::endl << std::endl;
  printProgressBar( "THROTTLE", speed / MAX_SPEED_FORWARD );
  printProgressBar( "STEERING", -steering / (M_PI/2));
}


static void LaserScanCallback( const sensor_msgs::LaserScan::ConstPtr& scan ) {
  
  // Make sure laser scan has valid data points
  if( scan->ranges.size() == 0 || scan->angle_increment == 0 ) {
    speed = 0;
    steering = 0;
    return;
  }

   // Convert LaserScan to Cartesian Map
  std::vector<CartesianCoordinate> CartesianMap = LaserScanToCartesianMap( scan );

  // Get cruise control information for both forward and backward
  CruiseControl forwardCruiseControl  = GetCruiseControl( CartesianMap, scan, VehicleGear::forward );
  CruiseControl backwardCruiseControl = GetCruiseControl( CartesianMap, scan, VehicleGear::backward );


  // Other Variables
  TurningCommands turningCommands;

  //wallDetectionOrientation = WallDetection( CartesianMap, currentOrientation );

  if( requestNewDriveMode ) {

    std::cout << "Switching drive mode..." << std::endl;
  
  switch( curMode ) {
      case DriveMode::cruise:
        
        desiredOrientation = fmod(currentOrientation + M_PI, 2 * M_PI);
        prevMode           = curMode;
	curMode            = DriveMode::reorientate;

        break;

      case DriveMode::reorientate:

        tmpMode  = curMode;
	curMode  = prevMode;
	prevMode = tmpMode;

        break;

      case DriveMode::obstacle_avoidance:

        if( obstacleAvoidanceReorientateRequest ) {
	  prevMode = curMode;
	  curMode  = DriveMode::reorientate;
	  obstacleAvoidanceReorientateRequest = false;
	}
	else {
	  prevMode = curMode;
	  curMode  = DriveMode::cruise;
	}
     
        break;
    }

    requestNewDriveMode = false;
  }
  

  // Determine throttle and steering base on current drive mode
  switch( curMode ) {
    case DriveMode::cruise:
/**
      // If car can go forward, use forward cruise control
      if( forwardCruiseControl.proposed_speed > 0 ) {
        speed    = std::max(MIN_SPEED_FORWARD, forwardCruiseControl.proposed_speed);
        steering = forwardCruiseControl.proposed_steering_angle;
      }

      // Car has reached obstacles
      else {
        requestNewDriveMode = true;
        speed               = 0;
	steering            = 0;
      }
**/

      
      // If can go forward, use forward cruise control
      if( forwardCruiseControl.proposed_speed >= 0 ) {
        speed    = std::max(MIN_SPEED_FORWARD, forwardCruiseControl.proposed_speed);
	steering = forwardCruiseControl.proposed_steering_angle;
      }

      // Cannot go forward, use backward cruise control
      else {
        if( backwardCruiseControl.proposed_speed < 0 ) {
          speed    = std::min(MIN_SPEED_BACKWARD, backwardCruiseControl.proposed_speed);
	  steering = backwardCruiseControl.proposed_steering_angle * -1;
	}
	else {
	  speed    = 0;
	  steering = 0;
	}
      }
      

      break;

    case DriveMode::reorientate:
      
      // Get suggested turning commands to get vehicle in right orientation
      turningCommands = TurnVehicle( CartesianMap, M_PI, currentOrientation, reorientateCarForward );
      std::cout << "Current Orientation: " << currentOrientation << std::endl;
      std::cout << "Gear: " << turningCommands.gear << std::endl;
      std::cout << "Direction: " << turningCommands.direction << std::endl;

      // Check to see if commands are valid
      if( turningCommands.gear == VehicleGear::forward && 
        turningCommands.direction == Direction::front ) {

	// Commands not valid - Make car go the opposite direction
	std::cout << "Changing Direction" << std::endl;
	reorientateCarForward = !reorientateCarForward;
	speed    = 0;
	steering = 0;

        // Exit the rest of the code
	break;
      }

      // Set speed based on forward or backward motion
      if( turningCommands.gear == VehicleGear::forward )
        speed = 2;//forwardCruiseControl.proposed_speed;
      else if( turningCommands.gear == VehicleGear::backward )
        speed = -2;//backwardCruiseControl.proposed_speed;
      else
        speed = 0;

      // Set steering accordingly
      if( turningCommands.direction == Direction::left )
        steering = MAX_STEERING_LEFT;
      else
        steering = MAX_STEERING_RIGHT;
      
      break;

    case DriveMode::obstacle_avoidance:
      speed = 0;
      steering = 0;
      break;

  }
}

static void OrientationCallback( const msg::imu_orientation::ConstPtr & orientation ) {
  currentOrientation = orientation->orientation;
}

int main( int argc, char ** argv ) {

  // Announce the name of this node to ROS
  ros::init(argc, argv, "APPOLO_DRIVER");

  // Create publisher and subscribers to be used later
  ros::NodeHandle nh;
  ros::Publisher AckermannDrivePublisher =  
    nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1);
  ros::Subscriber LaserScanSubscriber = 
    nh.subscribe("filtered_scan", 1, LaserScanCallback );
  ros::Subscriber Orientation = 
    nh.subscribe("orientation", 1, OrientationCallback );

  ros::Rate r( PUBLISH_RATE );

  // Continuously publish new sensor data
  while(nh.ok()) {
    system("clear");
    ros::spinOnce();

    speed = 0;

    // Generate AckermannDrive message for steering / throttle
    ackermann_msgs::AckermannDrive AckermannDrive;
    AckermannDrive.steering_angle = steering;
    AckermannDrive.steering_angle_velocity = 0;
    AckermannDrive.speed = speed;
    AckermannDrive.acceleration = 0;
    AckermannDrive.jerk = 0;

    // Attach AckermannDrive message to AckermannDriveStamped message
    ackermann_msgs::AckermannDriveStamped AckermannDriveStamped;
    AckermannDriveStamped.header.stamp = ros::Time::now();
    AckermannDriveStamped.header.frame_id = std::to_string( frame_id++ );
    AckermannDriveStamped.drive = AckermannDrive;

    // Publish drive commands
    AckermannDrivePublisher.publish( AckermannDriveStamped );

    VisualizeAckermannDrive( speed, steering );
    
    r.sleep();
  }

  return 0;
}

static void OptimizeAckermannDrive() {
  speed = (0.2 + 0.8 * (MAX_STEERING_LEFT - std::abs(steering) /
          MAX_STEERING_LEFT)) * speed;
}
