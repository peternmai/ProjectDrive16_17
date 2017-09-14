#include "SpeedController.h"

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

float GetCruiseSpeed( const sensor_msgs::LaserScan::ConstPtr & scan, float max_speed, float max_distance ) {
  CartesianCoordinate closestCoordinate = ClosestObstacleAhead( scan, -0.3048, 0.3048 );
  std::cout << "x = " << (closestCoordinate.x * 3.28) << " ft \t|\ty = " << (closestCoordinate.y * 3.28) << " ft" << std::endl;

  float car_buffer     = 1;
  float vision_range   = std::max(max_distance - car_buffer, (float) 0.1);
  float slope          = max_speed / vision_range;

  float proposed_speed = (closestCoordinate.y - car_buffer) * slope;
  std::cout << "Speed: " << proposed_speed << " m/s" << std::endl;
 
  return proposed_speed;
}

