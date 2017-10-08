#include "CruiseControl.h"
#include "LaneKeepingAssistance.h"

/**
 * Given a cartesian map, determine the coordinate of the closest obstacle ahead
 * of the vehicle and return it. 
 */
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

/**
 * Given a cartesian map, determine the coordinate of the closest obstacle behind
 * the vehicle and return it.
 */
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

/**
 * Given the cartesian map, polar coordinates, and direction the car is interested in,
 * determine the appropriate speed and steering angle the car should take in order to
 * avoid slamming into the obstacle.
 */
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
  
  //std::cout << "x = " << (closestCoordinate.x * 3.28) << " ft \t|\ty = " << (closestCoordinate.y * 3.28) << " ft" << std::endl;

  // Calculate the speed of the vehicle
  float vision_range = std::max(MAX_VISIBLE_RANGE - CAR_BUFFER, (float) 0.1);
  if( gear == VehicleGear::backward ) {
    slope = MAX_OBSTACLE_AVOIDANCE_SPEED_BACKWARD / vision_range;
    proposed_speed = (closestCoordinate.y + CAR_BUFFER) * slope * -1;
  }
  else {
    slope = MAX_OBSTACLE_AVOIDANCE_SPEED_FORWARD / vision_range;
    proposed_speed = (closestCoordinate.y - CAR_BUFFER) * slope;
  }

  std::cout << "Proposed speed " << ((gear == VehicleGear::backward) ? "backward: " : "forward: ") 
    << proposed_speed << " m/s" << std::endl;

  // Fill in cruise control data fields
  CruiseControl cruiseControl;
  cruiseControl.closestCoordinate       = closestCoordinate;
  cruiseControl.proposed_speed          = proposed_speed;

  // If no obstacles detected, sent to the maximum possible cruise speed
  if( proposed_speed == MAX_OBSTACLE_AVOIDANCE_SPEED_FORWARD )
    cruiseControl.proposed_speed = MAX_CRUISE_SPEED;

  // Determine the steering angle based on the lane keeping assistance module
  if( gear == VehicleGear::forward )
    cruiseControl.proposed_steering_angle = LaneKeepingAssistance( scan, true );
  else
    cruiseControl.proposed_steering_angle = LaneKeepingAssistance( scan, false );

  return cruiseControl;
}

