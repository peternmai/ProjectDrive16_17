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

