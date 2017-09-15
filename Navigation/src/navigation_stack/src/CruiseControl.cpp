#include "CruiseControl.h"
#include "LaneKeepingAssistance.h"

CartesianCoordinate ClosestObstacleAhead(
  const sensor_msgs::LaserScan::ConstPtr & scan, float min_x, float max_x) {

  // Create evenly spaced array to store ranges from -90 degrees to +90 degree
  float ranges[ (int) (M_PI / scan->angle_increment) ] = {0};
  float closestPointY = std::numeric_limits<float>::max();
  CartesianCoordinate point;
  CartesianCoordinate closestCoordinate;

  FilterLaserScanAngles(scan, ranges, M_PI * 1.5, M_PI * 0.5 );
  for( int i = 0; i < (int) (M_PI / scan->angle_increment); i++ ) {
    if( ranges[i] != 0 ) {
      point = PolarToCartesian( ranges[i], M_PI * 1.5 + i * scan->angle_increment );
      if( point.x >= min_x && point.x <= max_x ) {
        if( point.y < closestPointY ) {
          closestPointY = point.y;
          closestCoordinate = point;
        }
      }
    }
  }

  return closestCoordinate;
}

CartesianCoordinate ClosestObstacleBehind(
  const sensor_msgs::LaserScan::ConstPtr & scan, float min_x, float max_x) {

  // Create evenly spaced array to store ranges from +90 degree to +270 degree
  float ranges[ (int) (M_PI / scan->angle_increment) ] = {0};
  float closestPointY = -std::numeric_limits<float>::max();
  CartesianCoordinate point;
  CartesianCoordinate closestCoordinate;

  FilterLaserScanAngles(scan, ranges, M_PI * 0.5, M_PI * 1.5 );
  for( int i = 0; i < (int) (M_PI / scan->angle_increment); i++ ) {
    if( ranges[i] != 0 ) {
      point = PolarToCartesian( ranges[i], M_PI * 0.5 + i * scan->angle_increment );
      if( point.x >= min_x && point.x <= max_x ) {
        if( point.y > closestPointY ) {
	  closestPointY = point.y;
      	  closestCoordinate = point;
        }
      }
    }
  }

  return closestCoordinate;
}

CruiseControl GetCruiseControl( 
  const sensor_msgs::LaserScan::ConstPtr & scan, VehicleGear gear ) {

  CartesianCoordinate closestCoordinate;
  float proposed_speed, proposed_steering_angle, slope;

  // Determine closest obstacle ahead or behind vehicle
  if( gear == VehicleGear::backward )
    closestCoordinate = ClosestObstacleBehind(scan, BACKWARD_MIN_X, BACKWARD_MAX_X);
  else
    closestCoordinate = ClosestObstacleAhead(scan, FORWARD_MIN_X, FORWARD_MAX_X);
  
  std::cout << "x = " << (closestCoordinate.x * 3.28) << " ft \t|\ty = " << (closestCoordinate.y * 3.28) << " ft" << std::endl;

  // Calculate the speed of the vehicle
  float vision_range = std::max(MAX_VISIBLE_RANGE - CAR_BUFFER, (float) 0.1);
  if( gear == VehicleGear::backward ) {
    slope = MAX_SPEED_BACKWARD / vision_range;
    proposed_speed = (closestCoordinate.y + CAR_BUFFER) * slope;
  }
  else {
    slope = MAX_SPEED_FORWARD / vision_range;
    proposed_speed = (closestCoordinate.y - CAR_BUFFER) * slope;
  }

  std::cout << "Speed: " << proposed_speed << " m/s" << std::endl;

  // Fill in cruise control data fields
  CruiseControl cruiseControl;
  cruiseControl.closestCoordinate       = closestCoordinate;
  cruiseControl.proposed_speed          = proposed_speed;
  cruiseControl.proposed_steering_angle = LaneKeepingAssistance( scan );
 
  return cruiseControl;
}

