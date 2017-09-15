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

void printProgressBar( std::string title, float percentage ) {
  if( percentage < 0 )
    std::cout << title << ": [" << std::string( PROGRESSBAR_SIZE - (int) (-PROGRESSBAR_SIZE * percentage), ' ' )
              << std::string( (int) (-PROGRESSBAR_SIZE * percentage) + 1, '=' )
              << std::string( PROGRESSBAR_SIZE, ' ') << "] " << percentage * 100 << "%" << std::endl;
  else
    std::cout << title << ": [" << std::string( PROGRESSBAR_SIZE, ' ')
              << std::string( (int) (PROGRESSBAR_SIZE * percentage) + 1, '=' )
              << std::string( PROGRESSBAR_SIZE - (int) (PROGRESSBAR_SIZE * percentage), ' ') << "] " 
              << percentage * 100 << "%" << std::endl;
}

