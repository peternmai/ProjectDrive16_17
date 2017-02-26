#include "nav_headers.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

static unsigned int callback_instance = 0;

void tfpubCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static tf::TransformBroadcaster pub;

  tf::Transform baseframe_laserframe_transform;
  tf::Transform map_baseframe_transform;
  tf::Quaternion btl_q;
  tf::Quaternion mtb_q;

  float dist = msg->ranges[callback_instance];
  float angle = msg->angle_min;
  float x = dist * cos(angle);
  float y = dist * sin(angle);
  ros::Time s_t = msg->header.stamp;

  baseframe_laserframe_transform.setOrigin(tf::Vector3(0, 0, 0));
  btl_q.setRPY(0, 0, angle);
  baseframe_laserframe_transform.setRotation(btl_q);
  
  map_baseframe_transform.setOrigin(tf::Vector3(x, y, 0.0));
  mtb_q.setRPY(0, 0, 0);
  map_baseframe_transform.setRotation(mtb_q);

  pub.sendTransform(tf::StampedTransform(
  map_baseframe_transform, s_t,"map","base_frame"));
  pub.sendTransform(tf::StampedTransform(
  baseframe_laserframe_transform, s_t, "base_frame", "laser_frame"));
  callback_instance++;
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_scan", 600, tfpubCallback);

  ros::spin();
  return 0;
};
