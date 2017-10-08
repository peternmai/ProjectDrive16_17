/*-----------------------------------------------------------------------------

                                                         Author: Jason Ma
                                                         Date:   Sep 19 2017
                                      TODO

  File Name:      beam.cpp
  Description:    TODO
-----------------------------------------------------------------------------*/

#include <vector>
#include <iostream>
#include <math.h>

#define WIDTH 0.61 //0.61m for 2ft width
#define PI 3.1415926

using std::vector;
using std::cout;
using std::endl;


double beamPath(double width, int numRects) {
  cout << "Generating: " << numRects << " rectangles" << endl;
  
  //determine angles of all lines and calculate a central pair of points
  double angleInc = PI / (numRects - 1);

  vector<double> angles;
  vector<vector<double> > points;
  
  for(int i = 0; i < numRects; i++) {
    //calc angle
    double angle = angleInc * i;
    angles.push_back(angle);

    //calc central pair of points
    double xCen, yCen;
    xCen = cos(angle);
    yCen = sin(angle);

    //calc side points
    double temp_angle1 = angle - PI / 2;
    double temp_angle2 = angle + PI / 2;
    
    double x1, y1;
    x1 = cos(temp_angle1) * width / 2;
    y1 = sin(temp_angle1) * width / 2;

    double x2, y2;
    x2 = cos(temp_angle2) * width / 2;
    y2 = sin(temp_angle2) * width / 2;
    
    //parallel line endpoint calculations
    vector<double> point1;
    point1.push_back(xCen + x1);
    point1.push_back(yCen + y1);
    points.push_back(point1);

    vector<double> point2;
    point2.push_back(xCen + x2);
    point2.push_back(yCen + y2);
    points.push_back(point2);

    vector<double> point3;
    point3.push_back(0 + x1);
    point3.push_back(0 + y1);
    points.push_back(point3);

    vector<double> point4;
    point4.push_back(0 + x2);
    point4.push_back(0 + y2);
    points.push_back(point4);
  }

  cout << "Angles: " << endl;
  for (auto i = angles.begin(); i != angles.end(); ++i)
    cout << *i << ' ';

  cout << endl;

  int count = 0;
  cout << "Points: " << endl;
  for (auto i = points.begin(); i != points.end(); ++i) {
    for (auto j = i->begin(); j != i->end(); j++) {
      cout << *j << ' ';
    }

    count++;
    cout << endl;

    if(count % 4 == 0)
      cout << endl;
  }
  
  
}

int main() {
  beamPath(WIDTH, 9);

  return 0;
}
