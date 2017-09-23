#include "WallDetection.h"
#include "opencv2/core/version.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static float CalculateSlopeOfLine( cv::Point pt1, cv::Point pt2 ) {
  if( pt1.x < pt2.x )
    return ((pt2.x - pt1.x) != 0) ? (((float)pt2.y - pt1.y)/((pt2.x - pt1.x)) * -1) : 1000;
  else
    return ((pt1.x - pt2.x) != 0) ? (((float)pt1.y - pt2.y)/((pt1.x - pt2.x)) * -1): 1000;
}


float WallDetection( const std::vector<CartesianCoordinate> & CartesianMap,
  float currentImuDirection ) {

  // Initialize 2D array map to store laser scan cartesian map
  int X_AXIS_RANGE = (SCAN_MAX_X - SCAN_MIN_X) * SPACING_IN_METER + 1;
  int Y_AXIS_RANGE = (SCAN_MAX_Y - SCAN_MIN_Y) * SPACING_IN_METER + 1;
  unsigned char mapArray[ Y_AXIS_RANGE ][ X_AXIS_RANGE ];
  for( int x = 0; x < X_AXIS_RANGE; x++ )
    for( int y = 0; y < Y_AXIS_RANGE; y++ )
      mapArray[y][x] = (unsigned char) 0;

  // Convert cartesian map to 2D array
  for( int i = 0; i < CartesianMap.size(); i++ ) {
    CartesianCoordinate coordinate = CartesianMap.at(i);
    if( coordinate.x >= SCAN_MIN_X && coordinate.x <= SCAN_MAX_X &&
        coordinate.y >= SCAN_MIN_Y && coordinate.y <= SCAN_MAX_Y ) {
          mapArray[ Y_AXIS_RANGE - (int) ((coordinate.y - SCAN_MIN_Y) * SPACING_IN_METER) ]
	          [ (int) ((coordinate.x - SCAN_MIN_X) * SPACING_IN_METER) ] = 255;
	}
  }

  // Convert 2D array to OpenCV CV_8UC1 matrix
  std::vector<cv::Vec4i> lines;
  cv::Mat obstacleMatrix = cv::Mat(Y_AXIS_RANGE, X_AXIS_RANGE, CV_8UC1, mapArray);

  // Run hough transform and save the lines onto the detectedWalls matrix
  cv::Mat detectedWalls = obstacleMatrix.clone();
  cv::HoughLinesP( obstacleMatrix, lines, RHO, THETA, THRESHOLD, 
                   MIN_LINE_LENGTH, MAX_LINE_GAP );

  cv::Point pt1, pt2;
  int   totalLinesUsed  = 0;
  float combinedRadian  = 0;
  float averageRadian   = 0;
  
  //cv::HoughLines(obstacleMatrix, lines, 1, CV_PI/180, 100, 0, 0);
  for( size_t i = 0; i < lines.size(); i++ ) {

    // Draw detected lines over original obstacle map
    pt1 = cv::Point(lines[i][0], lines[i][1]);
    pt2 = cv::Point(lines[i][2], lines[i][3]);
    cv::line( detectedWalls, pt1, pt2, CV_RGB(0,100,255), 3, 4);

    // Calculate the slope of the line
    float slope  = CalculateSlopeOfLine( pt1, pt2 );
    float radian = CartesianToPolar( 1, slope ).radian;

    // Convert radian to between +PI/2 and -PI/2
    if( radian > (M_PI/2) )
      radian -= M_PI;
    if( radian < (-M_PI/2) )
      radian += M_PI;

    // Only take into consideration those walls within the MIN and MAX angle
    if( slope <= LEFT_SLOPE || slope >= RIGHT_SLOPE ) {
      totalLinesUsed++;
      combinedRadian += radian;
    }
  }


  std::cout << "Lines: " << lines.size() << std::endl;
  std::cout << "Lines Used: " << totalLinesUsed << std::endl;

  // Get a sense of where all the walls are facing
  if( totalLinesUsed != 0 )
    averageRadian = combinedRadian / totalLinesUsed;

  // Convert Angle between -PI and +PI
  if( averageRadian > (M_PI/2) )
    averageRadian -= M_PI;
  if( averageRadian < (-M_PI/2) )
    averageRadian += M_PI;
  
  std::cout << "Average radian: " << averageRadian << std::endl;

  // Plot the general wall direction onto the graph
  CartesianCoordinate coord = PolarToCartesian( 50, averageRadian );
  cv::line( detectedWalls, cv::Point(101, 199), cv::Point(101 + (int) coord.x,
  199 - (int) coord.y), CV_RGB(0,0,255), 2, 4);
  

  // Show the original obstacle map
  //cv::namedWindow( "Source", CV_WINDOW_NORMAL );
  //cv::imshow( "Source", obstacleMatrix );
  
  // Show the wall map
  cv::namedWindow( "Wall Detection Module", CV_WINDOW_NORMAL );
  cv::imshow( "Wall Detection Module", detectedWalls );

  cv::waitKey(1);

  // Return general direction with respect to IMU
  return currentImuDirection + averageRadian;

}

