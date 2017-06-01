#include "nav_headers.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <msg/optical_encoder.h>

void tfpubCallback(const msg::optical_encoder::ConstPtr& msg) {
  static tf::TransformBroadcaster pub;
  static unsigned int callback_instance = 0;

  tf::Transform map_baselink_transform;
  tf::Transform baselink_laserframe_transform;
  tf::Quaternion btl_q;
  tf::Quaternion mtb_q;

  float angle = msg->angle;
  ros::Time s_t = msg->time;

  baselink_laserframe_transform.setOrigin(tf::Vector3(0, 0, 0));
  btl_q.setRPY(0, 0, angle);
  baselink_laserframe_transform.setRotation(btl_q);
  
  map_baselink_transform.setOrigin(tf::Vector3(0, 0, 0));
  mtb_q.setRPY(0, 0, 0);
  map_baselink_transform.setRotation(mtb_q);

  pub.sendTransform(tf::StampedTransform(
  map_baselink_transform, s_t,"/map","/base_link"));
  pub.sendTransform(tf::StampedTransform(
  baselink_laserframe_transform, s_t, "/base_link", "/laser_frame"));
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("optical_encoder", 600, tfpubCallback);

  ros::spin();
  return 0;
};
