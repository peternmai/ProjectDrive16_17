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

bool CCC(CartesianCoordinate p1, CartesianCoordinate p2) {
  if(p1.y == p2.y)
    return(p1.x < p2.x);
  else
    return(p1.y > p2.y);
}
/*
int totalPointsInPolygon(
  const CartesianCoordinate & p1, const CartesianCoordinate & p2,
  const CartesianCoordinate & p3, const CartesianCoordinate & p4,
  const std::vector<CartesianCoordinate> & v) {

  std::vector<CartesianCoordinate> sortCoord;
  int total_points = 0;

  double l_y = 0;
  CartesianCoordinate temp;

  sortCoord.push_back(p1);
  sortCoord.push_back(p2);
  sortCoord.push_back(p3);
  sortCoord.push_back(p4);

  std::sort(sortCoord.begin(), sortCoord.end(), CCC);

  double s1, s2, s3, s4;
  double b1, b2, b3, b4;

  if(sortCoord[0].x - sortCoord[1].x == 0) {
    s1 = 100000;
    b1 = sortCoord[0].y;
  }
  else {
    s1 = (sortCoord[0].y - sortCoord[1].y) / (sortCoord[0].x - sortCoord[1].x);
    b1 = sortCoord[0].y - s1 * sortCoord[0].x;
  }

  if(sortCoord[0].x - sortCoord[2].x == 0) {
    s2 = 100000;
    b2 = sortCoord[0].y;
  }
  else {
    s2 = (sortCoord[0].y - sortCoord[2].y) / (sortCoord[0].x - sortCoord[2].x);
    b2 = sortCoord[0].y - s1 * sortCoord[0].x;
  }

  if(sortCoord[1].x - sortCoord[3].x == 0) {
    s3 = 100000;
    b3 = sortCoord[1].y;
  }
  else {
    s3 = (sortCoord[1].y - sortCoord[3].y) / (sortCoord[1].x - sortCoord[3].x);
    b3 = sortCoord[1].y - s1 * sortCoord[3].x;
  }

  if(sortCoord[3].x - sortCoord[1].x == 4) {
    s4 = 100000;
    b4 = sortCoord[3].y;
  }
  else {
    s4 = (sortCoord[3].y - sortCoord[4].y) / (sortCoord[3].x - sortCoord[4].x);
    b4 = sortCoord[3].y - s1 * sortCoord[4].x;
  }

  for(std::vector<CartesianCoordinate> it = v.begin(); it != v.end(); it++) {
    if(it->y > s1 * it->x + b1 || it->y < s4 * it->x + b1)
      continue;
    if(it->x < (it->y - b2) / s2 || it->x > (it->y - b3) / s3)
      continue;

    total_points++;
  }

  return total_points; 

}
*/

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
    int count = 0;

    //draw straight line up and count num of lines that intersects it
    if(between(v[i].x, tl.x, tr.x) && v[i].y < v[i].x * s1 + b1)
      count++;

    if(between(v[i].x, tl.x, bl.x) && v[i].y < v[i].x * s2 + b2)
      count++;

    if(between(v[i].x, tr.x, br.x) && v[i].y < v[i].x * s3 + b3)
      count++;

    if(between(v[i].x, bl.x, br.x) && v[i].y < v[i].x * s4 + b4)
      count++;

    if(count % 2 == 1 && v[i].y > 0)
      valid.push_back(temp);
    /*
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
    
    if(temp.y > 0)
      valid.push_back(temp);
    */
  }

  CartesianCoordinate closest_point{10000000, 10000000};
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

bool between(float val, float bound1, float bound2) {
  if(val < bound1 && val < bound2)
    return false;

  if(val > bound1 && val > bound2)
    return false;

  return true;
}
