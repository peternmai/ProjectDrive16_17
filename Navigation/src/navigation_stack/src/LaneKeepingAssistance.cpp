#include "LaneKeepingAssistance.h"

int main(int argc, char ** argv) {

  ros::init(argc, argv, "lra_pub");
  
  ros::NodeHandle n;
  ros::Subscriber scansub = n.subscribe("scan", MAX_READINGS,
    laserCallback);
  ros::Rate r(PUBLISH_RATE);

  while(n.ok()) {

    ros::spinOnce();

    double left_area = findLArea();
    double right_area = findRArea();
    
    r.sleep();
  }
}
