/*-----------------------------------------------------------------------------

                                                         Author: Jason Ma
                                                         Date:   Sep 19 2017
                                      drive

  File Name:      beam.cpp
  Description:    Provides function for finding the angle that will allow the
                  vehicle to move furthest forwards.
-----------------------------------------------------------------------------*/

#include <vector>
#include <iostream>
#include <math.h>
#include <chrono>

#include "ObstacleAvoidance.h"
#include "VehicleState.h"

#define WIDTH 0.61 //0.61m for 2ft width
#define PI 3.1415926

using std::vector;
using std::cout;
using std::endl;

//note: angleOffset is the relative difference between the front of the vehicle and forward on track
//left should be positive, right negative
float beamPath(const float & angleOffset, const float & width, 
  const int & numRects, const vector<CartesianCoordinate> & map) {

  auto t_start = std::chrono::high_resolution_clock::now();
  //cout << "Generating: " << numRects << " rectangles" << endl;
  
  vector<vector<float> >::iterator j;
  vector<vector<float> > points;
  vector<CartesianCoordinate>::iterator k;
  vector<CartesianCoordinate> results;
  vector<float> best_angles;
  vector<float> scores;

  int count = 0;
  float maxScore = 0;
  float best_angle = 0;
  
  float x_offset = 0;//cos(angleOffset) * 0.1;
  float y_offset = 0;//sin(angleOffset) * 0.1;
  
  //determine angles of all lines and calculate a central pair of points
  float angleInc = PI / (numRects - 1);
  for(int i = 1; i < numRects - 1; i++) {
    //calc angle
    float angle = angleInc * i + angleOffset;

    //calc central pair of points
    float xCen, yCen;
    xCen = cos(angle) * 100;
    yCen = sin(angle) * 100;

    //calc side points
    float x1, y1, x2, y2, temp_angle1, temp_angle2;
    temp_angle1 = angle - PI / 2;
    temp_angle2 = angle + PI / 2;
    x1 = cos(temp_angle1) * width / 2;
    y1 = sin(temp_angle1) * width / 2;

    x2 = cos(temp_angle2) * width / 2;
    y2 = sin(temp_angle2) * width / 2;
    
    //parallel line endpoint calculations
    CartesianCoordinate p1{xCen + x1 + x_offset, yCen + y1 + y_offset};
    CartesianCoordinate p2{xCen + x2 + x_offset, yCen + y2 + y_offset};
    CartesianCoordinate p3{x1 + x_offset, y1 + y_offset};
    CartesianCoordinate p4{x2 + x_offset, y2 + y_offset};

    CartesianCoordinate result = closestPointInPolygon(p2, p1, p4, p3, map);
    results.push_back(result);
  }
  
  float leftDist = results[numRects * 3 / 4].x * results[numRects * 3 / 4].x + results[numRects * 3 / 4].y * results[numRects * 3 / 4].y;

  float rightDist = results[numRects / 4].x * results[numRects / 4].x + results[numRects / 4].y * results[numRects / 4].y;

  count = 1;
  //Determine best angle using score based on distance and straightness.
  for(k = results.begin(); k != results.end(); k++) {
    float dist = k->x * k->x + k->y * k->y;
    float currAngle = count * angleInc + angleOffset - PI / 2;
    float currScore;
    
    float straightBonus = angleOffset - currAngle;
    if(straightBonus < 0) straightBonus *= -1;
    straightBonus = sqrt(1 / straightBonus);
    
    /*
    float correctionAngle = 3 / (rightDist - leftDist);
    if(correctionAngle < -0.5) correctionAngle = -0.5;
    if(correctionAngle > 0.5) correctionAngle = 0.5;
    correctionAngle = correctionAngle * PI;
    
    float correctionBonus = correctionAngle - currAngle;
    if(correctionBonus < 0) correctionBonus *= -1;
    correctionBonus = sqrt(0.1 / correctionBonus);
    */
    //correctionBonus = 0;

    currScore = dist + straightBonus;

    scores.push_back(currScore);

    if(currScore > maxScore) {
      maxScore = currScore;
      best_angle = currAngle;
    }
    
    count++;

    cout << "[beamPath] Angle: " << currAngle << " | Dist: " << dist << endl;
  }
  
  /*
  float threshold = 0.5;
  count = 1;

  //push all good distances onto array
  for(k = scores.begin(); k < scores.end(); k++) {
    if(*k >= maxScore - threshold) {
      best_angles.push_back(count * angleInc + angleOffset - PI / 2);

      cout << "[beamPath] Good angle: " << count * angleInc + angleOffset - PI / 2 << " | Score: " << *k << endl;
    }
    count++;
  }
  
  
  CartesianCoordinate frontp1{-0.4, 1};
  CartesianCoordinate frontp2{0.4, 1};
  CartesianCoordinate frontp3{-0.4, 0.1};
  CartesianCoordinate frontp4{0.4, 1};
  */

  /*
  if(totalPointsInPolygon(frontp1, frontp2, frontp3, frontp4, map) > 0) {
    
    cout << "[beamPath] Front Obstacle: +" << endl;
    //choose best angle not close to straight
    vector<float>::iterator angle;
    float maxAngle = 0;
    for( angle = best_angles.begin(); angle < best_angles.end(); angle++) {
      if(*angle - angleOffset > maxAngle || angleOffset - *angle > maxAngle) {
        best_angle = *angle;
      }
    }
  }
  else {
    cout << "[beamPath] Front Obstacle: -" << endl;
    //choose best angle closest to straight
    vector<float>::iterator angle;
    float minAngle = 100;
    for(angle = best_angles.begin(); angle < best_angles.end(); angle++) {
      if(*angle - angleOffset < minAngle || angleOffset - *angle < minAngle) {
        best_angle = *angle;
      }
    }
  }
  */
  
  cout << "[beamPath] Best Angle: " << (best_angle) << " | Max Score: " << maxScore << endl;
  
  auto t_end = std::chrono::high_resolution_clock::now();

  //cout << "BeamPath Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << endl;
  return best_angle;
}

float obstacleAvoidance(const vector<CartesianCoordinate> & CartesianMap,
  const float & beamAngleOffset, const float & beamWidth, const float & numBeams) {

  float desiredSteering = 0;

  // Figure out the furthest path out the car should go to
  float furthestAngleOut = beamPath(
    beamAngleOffset, beamWidth, numBeams, CartesianMap );
  
  return furthestAngleOut;
  /*
  // If there are obstacles right in front of car
  int leftPoints   = CartesianMapBoxFilter( CartesianMap, frontLeftBox ).size();
  int middlePoints = CartesianMapBoxFilter( CartesianMap, frontMiddleBox ).size();
  int rightPoints  = CartesianMapBoxFilter( CartesianMap, frontRightBox ).size();

  cout << "Points close up ahead:" << middlePoints << std::endl;
  if( middlePoints > 0 ) {
    if( leftPoints > 0 && rightPoints == 0 )
      desiredSteering = MAX_STEERING_RIGHT;
    else
      desiredSteering = MAX_STEERING_LEFT;
  }
  else
    desiredSteering = furthestAngleOut;


  return desiredSteering;
  */
}
