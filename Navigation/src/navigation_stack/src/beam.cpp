/*-----------------------------------------------------------------------------

                                                         Author: Jason Ma
                                                         Date:   Sep 19 2017
                                      TODO

  File Name:      beam.cpp
  Description:    Provides function for finding the angle that will allow the
                  vehicle to move furthest forwards.
-----------------------------------------------------------------------------*/

#include <vector>
#include <iostream>
#include <math.h>
#include "beam.h"

#define WIDTH 0.61 //0.61m for 2ft width
#define PI 3.1415926

using std::vector;
using std::cout;
using std::endl;

//note: angleOffset is the relative difference between the front of the vehicle and forward on track
//left should be positive, right negative
float beamPath(float angleOffset, float width, int numRects, vector<CartesianCoordinate> map) {
  cout << "Generating: " << numRects << " rectangles" << endl;
  
  //determine angles of all lines and calculate a central pair of points
  float angleInc = PI / (numRects - 1);

  vector<float> angles;
  vector<vector<float> > points;
  
  for(int i = 1; i < numRects - 1; i++) {
    //calc angle
    float angle = angleInc * i + angleOffset;
    angles.push_back(angle);

    //calc central pair of points
    float xCen, yCen;
    xCen = cos(angle) * 1000;
    yCen = sin(angle) * 1000;

    //calc side points
    float temp_angle1 = angle - PI / 2;
    float temp_angle2 = angle + PI / 2;
    
    float x1, y1;
    x1 = cos(temp_angle1) * width / 2;
    y1 = sin(temp_angle1) * width / 2;

    float x2, y2;
    x2 = cos(temp_angle2) * width / 2;
    y2 = sin(temp_angle2) * width / 2;
    
    //parallel line endpoint calculations
    vector<float> point1;
    point1.push_back(xCen + x1);
    point1.push_back(yCen + y1);
    points.push_back(point1);

    vector<float> point2;
    point2.push_back(xCen + x2);
    point2.push_back(yCen + y2);
    points.push_back(point2);

    vector<float> point3;
    point3.push_back(0 + x1);
    point3.push_back(0 + y1);
    points.push_back(point3);

    vector<float> point4;
    point4.push_back(0 + x2);
    point4.push_back(0 + y2);
    points.push_back(point4);
  }

  cout << "Angles: " << endl;
  vector<float>::iterator i;
  for (i = angles.begin(); i != angles.end(); ++i)
    cout << *i << ' ';

  cout << endl;

  int count = 0;
  vector<CartesianCoordinate> results;
  cout << "Points: " << endl;

  //Print points used in rectangle, then find closestPointInPolygon
  vector<vector<float> >::iterator j;
  for (j = points.begin(); j != points.end(); j += 4) {

    cout << (*j)[0] << " " << (*j)[1] << endl;
    cout << (*(j + 1))[0] << " " << (*(j + 1))[1] << endl;
    cout << (*(j + 2))[0] << " " << (*(j + 2))[1] << endl;
    cout << (*(j + 3))[0] << " " << (*(j + 3))[1] << endl;
    
    CartesianCoordinate p1{(*j)[0], (*j)[1]};
    CartesianCoordinate p2{(*(j + 1))[0], (*(j + 1))[1]};
    CartesianCoordinate p3{(*(j + 2))[0], (*(j + 2))[1]};
    CartesianCoordinate p4{(*(j + 3))[0], (*(j + 3))[1]};

    CartesianCoordinate result = closestPointInPolygon(p1, p2, p3, p4, map);
    cout << result.x << " " << result.y << endl;
    results.push_back(result);
  }

  //Determine best angle from closest point found
  float maxDist = 0;
  count = 1;
  float best_angle = 0;

  vector<CartesianCoordinate>::iterator k;
  for(k = results.begin(); k != results.end(); k++) {
    if(k->x * k->x + k->y * k->y > maxDist) {
      maxDist = k->x * k->x + k->y + k->y;
      best_angle = count * angleInc;
    }
  }
  return best_angle - PI / 2;
}
