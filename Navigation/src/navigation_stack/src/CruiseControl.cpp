#include "CruiseControl.h"
#include "LaneKeepingAssistance.h"

CartesianCoordinate ClosestObstacleAhead(
  const std::vector<CartesianCoordinate> & CartesianMap, float min_x, float max_x) {
  
  float closestPointY = MAX_VISIBLE_RANGE;
  CartesianCoordinate point;
  CartesianCoordinate closestCoordinate;

  // Initialize closest coordinate
  closestCoordinate.x = 0;
  closestCoordinate.y = MAX_VISIBLE_RANGE;

  for( int i = 0; i < CartesianMap.size(); i++ ) {
    point = CartesianMap.at(i);
    if( point.x >= min_x && point.x <= max_x && point.y >= 0.1 ) {
      if( point.y < closestPointY ) {
        closestPointY = point.y;
        closestCoordinate = point;
      }
    }
  }

  return closestCoordinate;
}

CartesianCoordinate ClosestObstacleBehind(
  const std::vector<CartesianCoordinate> & CartesianMap, float min_x, float max_x) {

  float closestPointY = -MAX_VISIBLE_RANGE;
  CartesianCoordinate point;
  CartesianCoordinate closestCoordinate;

  // Initialize closest coordinate
  closestCoordinate.x = 0;
  closestCoordinate.y = -MAX_VISIBLE_RANGE;

  for( int i = 0; i < CartesianMap.size(); i++ ) {
    point = CartesianMap.at(i);
    if( point.x >= min_x && point.x <= max_x && point.y <= -0.1 ) {
      if( point.y > closestPointY ) {
        closestPointY = point.y;
        closestCoordinate = point;
      }
    }
  }

  return closestCoordinate;
}

CruiseControl GetCruiseControl( 
  const std::vector<CartesianCoordinate> & CartesianMap,
  const sensor_msgs::LaserScan::ConstPtr & scan, VehicleGear gear ) {

  CartesianCoordinate closestCoordinate;
  float proposed_speed, proposed_steering_angle, slope;

  // Determine closest obstacle ahead or behind vehicle
  if( gear == VehicleGear::backward )
    closestCoordinate = ClosestObstacleBehind(CartesianMap, BACKWARD_MIN_X, BACKWARD_MAX_X);
  else
    closestCoordinate = ClosestObstacleAhead(CartesianMap, FORWARD_MIN_X, FORWARD_MAX_X);
  
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

  std::cout << "Speed " << ((gear == VehicleGear::backward) ? "backward: " : "forward: ") 
    << proposed_speed << " m/s" << std::endl;

  // Fill in cruise control data fields
  CruiseControl cruiseControl;
  cruiseControl.closestCoordinate       = closestCoordinate;
  cruiseControl.proposed_speed          = proposed_speed;
  cruiseControl.proposed_steering_angle = LaneKeepingAssistance( scan, true );

  return cruiseControl;
}

