#include "util.h"

/**
 * Name: FilterLaserScanAngles
 * TODO: fINISH IT
 * 
 */
void FilterLaserScanAngles(const sensor_msgs::LaserScan::ConstPtr & scan, 
  float * filtered_scan, float min_radian, float max_radian) {

  int   filteredScanSize = (int) (M_PI / scan->angle_increment);
  int   scanSize         = scan->ranges.size();
  int   index            = 0;
  float currentRadian    = 0;

  // Fill in filtered_scan array. Index are mapped evenly from min_radian to max_radian
  for( int i = 0; i < scanSize; i++ ) {

    currentRadian = fmod(scan->angle_min + i * scan->angle_increment, 2*M_PI );

    if( min_radian < max_radian ) {
      if( currentRadian >= min_radian && currentRadian <= max_radian ) {
        index = (int) ((currentRadian - min_radian) / scan->angle_increment);
      }
    }
    else {
      if( currentRadian >= min_radian || currentRadian <= max_radian ) {
        if( currentRadian > min_radian )
	        index = (int) ((currentRadian - min_radian) / scan->angle_increment);
	      else
	        index = (int) ((currentRadian + 2*M_PI - min_radian) / scan->angle_increment);
      }
    }

    if( index >= 0 && index < (int) (M_PI / scan->angle_increment) )
      filtered_scan[ index ] = scan->ranges[i];
  }

}


std::vector<CartesianCoordinate> LaserScanToCartesianMap(
  const sensor_msgs::LaserScan::ConstPtr & scan ) {

  std::vector<CartesianCoordinate> CartesianMap;
  CartesianCoordinate coordinate;

  for( int i = 0; i < scan->ranges.size(); i++ ) {
    coordinate = PolarToCartesian( 
      scan->ranges[i], scan->angle_min + i * scan->angle_increment );
    CartesianMap.push_back( coordinate );
  }

  return CartesianMap;
}

std::vector<CartesianCoordinate> CartesianMapBoxFilter(
  const std::vector<CartesianCoordinate> & CartesianMap,
  BoxCoordinates boxCoordinates ) {

  std::vector<CartesianCoordinate> filteredCartesianMap;
  CartesianCoordinate coordinate;

  std::cout << boxCoordinates.min_x << " - " << boxCoordinates.max_x << std::endl;
  std::cout << boxCoordinates.min_y << " - " << boxCoordinates.max_y << std::endl;

  for( int i = 0; i < CartesianMap.size(); i++ ) {
    coordinate = CartesianMap.at(i);
    if( coordinate.x >= boxCoordinates.min_x && 
        coordinate.x <= boxCoordinates.max_x &&
        coordinate.y >= boxCoordinates.min_y && 
	coordinate.y <= boxCoordinates.max_y )
	  if( coordinate.x != 0 && coordinate.y != 0 )
            filteredCartesianMap.push_back( coordinate );
  }

  return filteredCartesianMap;
}

float getClosestObstacleInRectangleBeam(
  const std::vector<CartesianCoordinate> & CartesianMap, 
  float beamAngle, float beamWidth, float lidarToFrontDistance, float carWidth){

  // Generate the two parallel lines to check the points between
  CartesianCoordinate beamAngleCartesian = PolarToCartesian(1, beamAngle);
  float slope = beamAngleCartesian.y / beamAngleCartesian.x;
  float upperYOffset, upperXOffset, lowerYOffset, lowerXOffset;
  CartesianCoordinate coordinate, closestObstacle;

  // Equation: (y + yOffset) = slope * ( x + xOffset )
  if( slope < 0 ) {
    lowerXOffset = -carWidth / 2;
    upperXOffset =  carWidth / 2;
  }
  else if( slope > 0 ) {
    lowerXOffset =  carWidth / 2;
    upperXOffset = -carWidth / 2;

  }
  else {
    lowerXOffset = -carWidth / 2;
    upperXOffset =  carWidth / 2;
  }
/**
  float closestObstacle = std::numeric_limits<float>::max();
  for( int i = 0; i < CartesianMap.size(); i++ ) {
    if( slope < 0 ) {
      // Check if x coordinate is within the two parallel lines
      if(coordinate.x >= ((coordinate.y + lowerYOffset)/slope - lowerXOffset) &&
         coordinate.x <= ((coordinate.y + upperYOffset)/slope - upperYOffset) &&
	 coordinate.y <= (slope*(coordinate.x + upperXOffset) - upperYOffset) &&
	 coordinate.y >= (slope*(coordinate.x + lowerXOffset) - lowerYOffset) {
	   if( CartesianToPolar.
	   closestObstacle = coordinate;
	 }
    }
    else {
    }

  }


**/
}

int totalPointsInPolygon(
  const CartesianCoordinate & tl, const CartesianCoordinate & tr,
  const CartesianCoordinate & bl, const CartesianCoordinate & br,
  const std::vector<CartesianCoordinate> & v) {

  float s1, s2, s3, s4;
  float b1, b2, b3, b4;

  s1 = (tl.y - tr.y) / (tl.x - tr.x);

  if(tl.x - bl.x == 0)
    s2 = 100000;
  else
    s2 = (tl.y - bl.y) / (tl.x - bl.x);

  if(tr.x - br.x == 0)
    s3 = 100000;
  else
    s3 = (tr.y - br.y) / (tr.x - br.x);

  s4 = (bl.y - br.y) / (bl.x - br.x);
    
  b1 = tl.y - s1 * tl.x;
  
  if(s2 < 1000)
    b2 = tl.y - s2 * tl.x;

  if(s3 < 1000)
    b3 = tr.y - s3 * tr.x;

  b4 = bl.y - s4 * bl.x;

  CartesianCoordinate  temp;
  int inside_count = 0;

  for(int i = 0; i < v.size(); i++) {
    temp = v[i];
    if(temp.y > s1 * temp.x + b1)
      continue;
    if(s2 < 10000) {
      if(temp.x < (temp.y - b2) / s2)
        continue;
    } else {
      if(temp.x < bl.x)
        continue;
    }
    if(s3 < 10000) {
      if(temp.x > (temp.y - b3) / s3)
        continue;
    } else {
      if(temp.x > br.x)
        continue;
    }
    if(temp.y < s4 * temp.x + b4)
      continue;

    inside_count++;
  }

  return inside_count;
}

CartesianCoordinate closestPointInPolygon(
  const CartesianCoordinate & tl, const CartesianCoordinate & tr,
  const CartesianCoordinate & bl, const CartesianCoordinate & br,
  const std::vector<CartesianCoordinate> & v) {

  float s1, s2, s3, s4;
  float b1, b2, b3, b4;

  s1 = (tl.y - tr.y) / (tl.x - tr.x);

  if(tl.x - bl.x == 0)
    s2 = 100000;
  else
    s2 = (tl.y - bl.y) / (tl.x - bl.x);

  if(tr.x - br.x == 0)
    s3 = 100000;
  else
    s3 = (tr.y - br.y) / (tr.x - br.x);

  s4 = (bl.y - br.y) / (bl.x - br.x);
    
  b1 = tl.y - s1 * tl.x;
  
  if(s2 < 1000)
    b2 = tl.y - s2 * tl.x;

  if(s3 < 1000)
    b3 = tr.y - s3 * tr.x;

  b4 = bl.y - s4 * bl.x;

  CartesianCoordinate temp;
  std::vector<CartesianCoordinate> valid;

  for(int i = 0; i < v.size(); i++) {
    temp = v[i];
    if(temp.y > s1 * temp.x + b1)
      continue;
    if(s2 < 10000) {
      if(temp.x < (temp.y - b2) / s2)
        continue;
    } else {
      if(temp.x < bl.x)
        continue;
    }
    if(s3 < 10000) {
      if(temp.x > (temp.y - b3) / s3)
        continue;
    } else {
      if(temp.x > br.x)
        continue;
    }
    if(temp.y < s4 * temp.x + b4)
      continue;

    valid.push_back(temp);
  }

  CartesianCoordinate closest_point{10000, 10000};
  float minDist = 100000000;
  for(int i = 0; i < valid.size(); i++) {
    if(valid[i].x * valid[i].x + valid[i].y * valid[i].y < minDist) {
      minDist = valid[i].x * valid[i].x + valid[i].y * valid[i].y;
      closest_point = valid[i];
    }
  }

  return closest_point;
}

void printProgressBar( std::string title, float percentage ) {
  if( percentage < 0 )
    std::cout << title << ": [" << std::string( std::max(0, 
                 PROGRESSBAR_SIZE -(int) (-PROGRESSBAR_SIZE * percentage)), ' ' )
              << std::string( std::max(0, 
	         (int) (-PROGRESSBAR_SIZE * percentage) + 1), '=' )
              << std::string( PROGRESSBAR_SIZE, ' ') << "] " << percentage * 100 << "%" << std::endl;
  else
    std::cout << title << ": [" << std::string( PROGRESSBAR_SIZE, ' ')
              << std::string( std::max(0,
	         (int) (PROGRESSBAR_SIZE * percentage) + 1), '=' )
              << std::string( std::max(0, 
	         PROGRESSBAR_SIZE - (int) (PROGRESSBAR_SIZE * percentage)), ' ') << "] " 
              << percentage * 100 << "%" << std::endl;
}

float orientationDiff( float currentOrientation, float desiredOrientation ) {

  std::cout << "Current Orientation: " << currentOrientation << std::endl;
  std::cout << "Desired Orientation: " << desiredOrientation << std::endl;

  float smallestTurningAngle = std::numeric_limits<float>::max();
  bool goRight = false;

  // Figure out if car should head left or right
  if( currentOrientation <= desiredOrientation ) {
    if( desiredOrientation - currentOrientation < smallestTurningAngle ) {
      smallestTurningAngle = desiredOrientation - currentOrientation;
      goRight = false;;
    }

    if( currentOrientation + 2 * M_PI - desiredOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation + 2 * M_PI - desiredOrientation;
      goRight = true;;
    }
  }
  else {
    if( currentOrientation - desiredOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation - desiredOrientation;
      goRight = true;;
    }
    
    if( desiredOrientation + 2 * M_PI - currentOrientation < smallestTurningAngle ) {
      smallestTurningAngle = currentOrientation - desiredOrientation;
      goRight = false;;
    }
  }

  if( goRight )
    smallestTurningAngle *= -1;

  std::cout << "Orientation diff from straight wall: " << smallestTurningAngle
  << std::endl;

  return smallestTurningAngle;
}
