#include "nav_headers.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

static unsigned int callback_instance = 0;

void tfpubCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static tf::TransformBroadcaster pub;
  tf::Transform transform;

  float dist = msg->ranges[callback_instance];
  float angle = msg->angle_min + msg->angle_increment * callback_instance;
  float x = dist * sin(angle);
  float y = dist * cos(angle);

  transform.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, angle);
  transform.setRotation(q);
  //needs numbers for transform to base_link from laser
  pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
  "base_link"));
  callback_instance++;
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_scan", 100,
  tfpubCallback);

  ros::spin();
  return 0;
};
