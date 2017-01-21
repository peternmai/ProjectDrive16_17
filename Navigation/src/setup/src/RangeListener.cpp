#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include<string>

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  //std::string s = "";
  std::cout << "I heard something: ";
  for( int i = 0; i < msg->ranges.size(); i++ ) {
     //s = strcat(s, std::to_string(msg->ranges[i]));
     std::cout << msg->ranges[i] << " ";
  }
     std::cout << std::endl;
  //ROS_INFO("I heard something: [%s]", s);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_scan", 1000, chatterCallback);
  ros::spin();

  return 0;

}
