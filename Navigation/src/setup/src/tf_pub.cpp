#include "nav_headers.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

void tfpubCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static tf::TransformBroadcaster pub;

  tf::Transform map_odom_t;
  tf::Transform base_laser_t;
  tf::Quaternion mto_q;
  tf::Quaternion btl_q;

  ros::Time s_t = msg->header.stamp;

  map_odom_t.setOrigin(tf::Vector3(0, 0, 0));
  base_laser_t.setOrigin(tf::Vector3(0, 0, 0));

  mto_q.setRPY(0, 0, 0);
  btl_q.setRPY(0, 0, 0);

  map_odom_t.setRotation(mto_q);
  base_laser_t.setRotation(btl_q);

  pub.sendTransform(tf::StampedTransform(map_odom_t, s_t,"/map","/odom"));
  pub.sendTransform(tf::StampedTransform(base_laser_t, s_t,
    "/base_link", "/laser_frame"));
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("scan", 600, tfpubCallback);

  ros::spin();
  return 0;
};
