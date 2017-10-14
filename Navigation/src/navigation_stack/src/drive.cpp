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
#include <ctime>
#include <ros/master.h>

static DriveMode curMode  = DriveMode::obstacle_avoidance;
static DriveMode prevMode = DriveMode::cruise;
static DriveMode tmpMode  = DriveMode::cruise;

static float refreshRate = 0;

static float speed = 0;
static float steering = 0;
static long  frame_id = 0;

static float currentOrientation         = -1;
static float averageOrientation         = -1;
static float oldAvgOrientation          = -1;
static float desiredOrientation         = -1;
static float wallDetectionOrientation   = -1;
static float pitch                      = -1;
static float avgPitch                   = -1;

static float reorientateOrientation     = -1;

static float numIterationBackward       = 0;

static bool reorientateCarForward = true;
static bool obstacleAvoidanceReorientateRequest = false;
static bool requestNewDriveMode = false;

//Jason Test variables - obstacle avoidance
static bool leftBlocked = false;
static bool rightBlocked = false;
static bool frontBlocked = false;
static float navPointX = 0;
static float navPointY = 0;

static float frontWidth = 0.3;

static float bestAngle = 0;

static CartesianCoordinate p1_front{frontWidth / -2, 1};
static CartesianCoordinate p2_front{frontWidth / 2, 1};
static CartesianCoordinate p3_front{frontWidth / -2, 0.1};
static CartesianCoordinate p4_front{frontWidth / 2, 0.1};

static CartesianCoordinate p1_left{-0.7, 0.15};
static CartesianCoordinate p2_left{-0.1, 0.15};
static CartesianCoordinate p3_left{-0.7, -0.15};
static CartesianCoordinate p4_left{-0.1, -0.15};

static CartesianCoordinate p1_right{0.1, 0.15};
static CartesianCoordinate p2_right{0.7, 0.15};
static CartesianCoordinate p3_right{0.1, -0.15};
static CartesianCoordinate p4_right{0.7, -0.15};


static void VisualizeAckermannDrive( float throttle, float steering ) {
  std::cout << "Mode: " << curMode << std::endl << std::endl;
  printProgressBar( "THROTTLE", speed / MAX_CRUISE_SPEED );
  printProgressBar( "STEERING", -steering / (M_PI/2));
}


static void LaserScanCallback( const sensor_msgs::LaserScan::ConstPtr& scan ) {

  clock_t startTime = std::clock();
  
  // Make sure laser scan has valid data points
  if( scan->ranges.size() == 0 || scan->angle_increment == 0 ) {
    speed = 0;
    steering = 0;
    return;
  }

   // Convert LaserScan to Cartesian Map
  std::vector<CartesianCoordinate> CartesianMap = LaserScanToCartesianMap( scan );

  // Get cruise control information for both forward and backward
  CruiseControl forwardCruiseControl  = 
    GetCruiseControl( CartesianMap, scan, VehicleGear::forward );
  CruiseControl backwardCruiseControl = 
    GetCruiseControl( CartesianMap, scan, VehicleGear::backward );

  // Other Variables
  TurningCommands turningCommands;
  
  // Calculate which direction the wall is pointing towards
  wallDetectionOrientation = WallDetection( CartesianMap, currentOrientation );
  float straightOrientation = 
    orientationDiff( currentOrientation, wallDetectionOrientation );

  // Print out orientation of car
  std::cout << "Current Orientation: " << currentOrientation << std::endl;
  std::cout << "Average Orientation: " << averageOrientation << std::endl;
  std::cout << "Wall    Orientation: " << wallDetectionOrientation << std::endl;
  std::cout << "Orientation Diff From Wall: " << straightOrientation << std::endl;  

  // Check if car needs to perform a U-TURN
  if((curMode != DriveMode::reorientate ) && (checkUTurn(averageOrientation,
    currentOrientation, wallDetectionOrientation))) {
    std::cout << "Trigger UTURN" << std::endl;
    reorientateOrientation = oldAvgOrientation;
    prevMode = curMode;
    curMode  = DriveMode::reorientate;
  }

  /**
  float areaInFront = t_area(scan);
  if( curMode != DriveMode::reorientate ) {
    if( areaInFront > CRUISE_FRONT_AREA_THRESHOLD && curMode ) {
      prevMode = curMode;
      curMode  = DriveMode::cruise;
    }
    else {
      prevMode = curMode;
      curMode  = DriveMode::obstacle_avoidance;
    }
  }
  **/
  

  // One of the driving mode requested a new mode
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
        steering = (forwardCruiseControl.proposed_steering_angle + 
                    straightOrientation) / 2;
      }

      // Car has reached obstacles
      else {
        requestNewDriveMode = true;
        speed               = 0;
        steering            = 0;
      }
      **/

      /***
      // If can go forward, use forward cruise control
      if( forwardCruiseControl.proposed_speed >= 0 ) {
        speed    = std::max(MIN_SPEED_FORWARD, forwardCruiseControl.proposed_speed);
        steering = (forwardCruiseControl.proposed_steering_angle +
                    straightOrientation);

      }

      // Cannot go forward, use backward cruise control
      else {
        if( backwardCruiseControl.proposed_speed < 0 ) {
          speed    = std::min(MIN_SPEED_BACKWARD, backwardCruiseControl.proposed_speed);
          steering = (backwardCruiseControl.proposed_steering_angle -
                      straightOrientation);
        }
        else {
          speed    = 0;
          steering = 0;
        }
      }
      **/
      speed = forwardCruiseControl.proposed_speed;
      steering = (obstacleAvoidance( CartesianMap, straightOrientation, 0.5, 30)
                 + straightOrientation) / 2;
      
      

      break;

    case DriveMode::reorientate:
      
      // Get suggested turning commands to get vehicle in right orientation
      turningCommands = TurnVehicle( CartesianMap, reorientateOrientation, 
                                     currentOrientation, reorientateCarForward);

      std::cout << "Desired Orientation: " << reorientateOrientation << std::endl;

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
        speed = std::max((float)0.1, forwardCruiseControl.proposed_speed);
      else if( turningCommands.gear == VehicleGear::backward )
        speed = std::min((float)-0.1, backwardCruiseControl.proposed_speed);
      else {
        speed = 0;
        curMode = prevMode;
        prevMode = DriveMode::reorientate;
      }

      // Set steering accordingly
      if( turningCommands.direction == Direction::left )
        steering = MAX_STEERING_LEFT;
      else
        steering = MAX_STEERING_RIGHT;
      
      break;

    case DriveMode::obstacle_avoidance:
      speed = forwardCruiseControl.proposed_speed;
      steering = obstacleAvoidance(CartesianMap, straightOrientation, 0.45, 40);

      // Check if can go straight
      if( CartesianMapBoxFilter( CartesianMap, frontBuffer ).size() > 0 || speed <= MIN_SPEED_BACKWARD) {
        speed = backwardCruiseControl.proposed_speed;
        steering *= -1;
        numIterationBackward = refreshRate * MIN_BACKUP_TIME;
      }
      else
        speed = std::max(MIN_SPEED_FORWARD, speed);
      
      if( numIterationBackward > 0 ) {
        speed = backwardCruiseControl.proposed_speed;
        steering *= -0.5;
        numIterationBackward--;
      }


      /*
      //if front is blocked or sides become unblocked, run beampath
      int numPointsTop, numPointsLeft, numPointsRight;
      
      numPointsTop = totalPointsInPolygon(p1_front, p2_front, p3_front, p4_front, CartesianMap);
      numPointsLeft = totalPointsInPolygon(p1_left, p2_left, p3_left, p4_left, CartesianMap);
      numPointsRight = totalPointsInPolygon(p1_right, p2_right, p3_right, p4_right, CartesianMap);
      
      if((numPointsTop > 0 && !frontBlocked) || (numPointsLeft == 0 && leftBlocked) || (numPointsRight == 0 && rightBlocked)) {
        bestAngle = beamPath(straightOrientation, 0.6, 20, CartesianMap);
        //navPointX = cos(bestAngle + 3.141593 / 2) * 1000;
        //navPointY = sin(bestAngle + 3.141593 / 2) * 1000;
      
        std::cout << "BeamPath: " << bestAngle << std::endl; 
        //reorientateOrientation = bestAngle + currentOrientation;

        //if(reorientateOrientation < 0) reorientateOrientation += 2 * 3.141593;
        //prevMode = curMode;
        //curMode  = DriveMode::reorientate;
      }
        
      if(numPointsTop > 0) frontBlocked = true;
      else frontBlocked = false;

      if(numPointsLeft > 0) leftBlocked = true;
      else leftBlocked = false;

      if(numPointsRight > 0) rightBlocked = true;
      else rightBlocked = false;
      
      //navPointX
      //navPointY
      //leftBlocked
      //rightBlocked
      //frontBlocked

      if(currentOrientation < bestAngle)
        steering = MAX_STEERING_LEFT * (bestAngle - currentOrientation) / 1.5;
      else if(currentOrientation > bestAngle)
        steering = MAX_STEERING_RIGHT * (currentOrientation - bestAngle) / 1.5;
      else
        steering = 0;

      std::cout << "F: " << frontBlocked << " L: " << leftBlocked << " R: " << rightBlocked << " N_X: " << navPointX << " N_Y: " << navPointY << " Steering: " << steering << std::endl;
      */
      break;

  }

  //if( curMode == DriveMode::cruise ) {
  //  speed *= (float) 1 - (std::abs(steering) * 0.5 )/ MAX_STEERING_LEFT;
  //}

  if( speed > 0 ) {
    speed = std::max(MIN_SPEED_FORWARD, speed);
  }

  if( speed < 0 ) {
    speed = std::min(MIN_SPEED_BACKWARD, speed);
  }

  int obstaclesToRight = CartesianMapBoxFilter( CartesianMap, rightBuffer ).size();
  int obstaclesToLeft  = CartesianMapBoxFilter( CartesianMap, leftBuffer  ).size();

  if( obstaclesToRight > 0 )
    steering = std::max((float) 0.07, steering);
  if( obstaclesToLeft > 0 )
    steering = std::min((float) -0.07, steering);

  // Uneven slope surface override
  if(((pitch > 0.15) && (avgPitch > 0.15)) || ((pitch < -0.15) && (avgPitch < -0.15))) {

    // If head is facing down
    if( pitch > 0 ) {
      speed = MIN_SPEED_FORWARD;
      steering = 0;
    }
    // If head is facing up and ahead is clear
    else if( pitch < 0 && speed > 0 )
      speed = 999;

    // Else go back
    else {
      speed = speed;
      steering = 0;
    }
  }

  clock_t endTime   = std::clock();
  double  duration  = double(endTime - startTime) / CLOCKS_PER_SEC;
  std::cout << "Total calculation time: " << duration << std::endl;
}

static void OrientationCallback( const msg::imu_orientation::ConstPtr & orientation ) {
  currentOrientation = orientation->orientation;
  averageOrientation = orientation->avg_orientation;
  oldAvgOrientation  = orientation->old_avg;
  pitch              = orientation->pitch;
  avgPitch           = orientation->avg_pitch;

}

int main( int argc, char ** argv ) {

  // Announce the name of this node to ROS
  ros::init(argc, argv, "APPOLO_DRIVER");

  // Create publisher and subscribers to be used later
  ros::NodeHandle nh;
  ros::Publisher AckermannDrivePublisher =  
    nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 1);
  ros::Publisher VehicleStatePublisher = 
    nh.advertise<msg::vehicle_status>("vehicle_status", 1);
  ros::Subscriber LaserScanSubscriber = 
    nh.subscribe("filtered_scan", 1, LaserScanCallback );
  ros::Subscriber Orientation = 
    nh.subscribe("orientation", 1, OrientationCallback );

  nh.getParam("/sweep_node/rotation_speed", refreshRate);
  ros::Rate r( refreshRate );

  // Continuously publish new sensor data
  while(nh.ok() && ros::master::check()) {
    
    system("clear");
    std::cout << "Current Time: " << ros::Time::now() << std::endl;

    ros::spinOnce();

    //speed = 0;
    //steering = 0;

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

    msg::vehicle_status vehicleStatusMsg;
    vehicleStatusMsg.drive_mode = curMode;
    VehicleStatePublisher.publish( vehicleStatusMsg );

    VisualizeAckermannDrive( speed, steering );
    
    r.sleep();
  }

  return 0;
}
