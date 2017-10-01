#include "LaneKeepingAssistance.h"

/*
  This file is mostly used as a unit tester for the 
  LaneKeepingAssistance.h file. It creates an area publisher
  from just the laser scan message and then it broadcasts the results.
  It is just a basic implementation of initializing a ROS node.
*/

int main(int argc, char ** argv) {

  ros::init(argc, argv, "lra_pub");
  
  ros::NodeHandle n;
  ros::Subscriber scansub = n.subscribe("filtered_scan", MAX_READINGS,
    laserCallback);
  ros::Rate r(PUBLISH_RATE);

  while(n.ok()) {

    ros::spinOnce();

    double l_area = findLArea();
    double r_area = findRArea();
    double total = l_area + r_area;
    double diff = (l_area - r_area) / total;

    displayResults(l_area, r_area, total, diff);
    
    r.sleep();
  }
}
