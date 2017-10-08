#include "VehicleState.h"
#include "QuickTurning.h"

/**
 *                    N (PI)
 *                      ^
 *                      |
 *                      |
 *   (3*PI/2)  W <------o------> E  (PI/2)
 *                      |
 *                      |
 *                      v
 *                    S (0)
 */

TurningCommands TurnVehicle( const std::vector<CartesianCoordinate> & CartesianMap,
  float desiredOrientation, float currentOrientation, bool forward ) {

  std::vector<CartesianCoordinate> map;
  float smallestTurningAngle = std::numeric_limits<float>::max();
  Direction direction = Direction::front;
  TurningCommands turningCommands;

  // Flag indicating program found a direction a car can go to
  bool canTurn = true;

  // Figure out if car should head left or right
  if( currentOrientation <= desiredOrientation ) {
    if( desiredOrientation - currentOrientation < smallestTurningAngle ) {
      smallestTurningAngle = desiredOrientation - currentOrientation;
      direction = Direction::left;
    }

    if( currentOrientation + 2 * M_PI - desiredOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation + 2 * M_PI - desiredOrientation;
      direction = Direction::right;
    }
  }
  else {
    if( currentOrientation - desiredOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation - desiredOrientation;
      direction = Direction::right;
    }
    
    if( desiredOrientation + 2 * M_PI - currentOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation - desiredOrientation;
      direction = Direction::left;
    }
  }

  //std::cout << "turning diff: " << smallestTurningAngle << std::endl;

  // Check to see if car has already reached desired orientation
  if( smallestTurningAngle < TURNING_ANGLE_BUFFER ) {
    turningCommands.gear      = VehicleGear::parked;
    turningCommands.direction = Direction::front;
    return turningCommands;
  }

  // Check if car can go left forward
  if( direction == Direction::left ) {
    map = CartesianMapBoxFilter( CartesianMap, FORWARD_LEFT_BOX );

    // Car can go forward left
    if( map.size() == 0 && forward ) {
      turningCommands.gear      = VehicleGear::forward;
      turningCommands.direction = Direction::left;
    }

    // Car cannot go forward left. See if it can go backward right
    else {
      map = CartesianMapBoxFilter( CartesianMap, BACKWARD_RIGHT_BOX );

      // Car can go backward right
      if( map.size() == 0 && !forward ) {
        turningCommands.gear      = VehicleGear::backward;
      	turningCommands.direction = Direction::right;
      }

      // Car cannot turn
      else {
        canTurn = false;
      }
    }
  }

  // Check if car can go right forward
  else {
    map = CartesianMapBoxFilter( CartesianMap, FORWARD_RIGHT_BOX );

    // Car can go forward right
    if( map.size() == 0 && forward ) {
      turningCommands.gear      = VehicleGear::forward;
      turningCommands.direction = Direction::right;
    }

    // Car cannot go forward right. See if it can go backward left
    else {
      map = CartesianMapBoxFilter( CartesianMap, BACKWARD_LEFT_BOX );

      // Car can go backward left
      if( map.size() == 0 && !forward ) {
        turningCommands.gear      = VehicleGear::backward;
      	turningCommands.direction = Direction::left;
      }

      // Car cannot turn
      else {
        canTurn = false;
      }
    }
  }

  //std::cout << "Going Forward: " << forward << std::endl;

  // Return turning commands
  if( canTurn )
    return turningCommands;

  // Indicate error - no turn found
  else {
    std::cout << "!!! CANNOT TURN !!!" << std::endl;
    std::cout << map.size() << std::endl;
    turningCommands.gear      = VehicleGear::forward;
    turningCommands.direction = Direction::front;
    return turningCommands;
  }
}
