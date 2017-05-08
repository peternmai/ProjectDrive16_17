#include "nav_headers.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

void tfpubCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static tf::TransformBroadcaster pub;
  static unsigned int callback_instance = 0;

  tf::Transform map_baselink_transform;
  tf::Transform baselink_laserframe_transform;
  tf::Quaternion btl_q;
  tf::Quaternion mtb_q;

  //while(callback_instance <= (msg->angle_max - msg->angle_min)/msg->angle_increment) {
    //float dist = msg->ranges[callback_instance];
    //float angle = msg->angle_min + callback_instance * msg->angle_increment;
    //ros::Time s_t = msg->header.stamp + ros::Duration(msg->time_increment * callback_instance);

    float angle = msg->angle_max;
    ros::Time s_t = msg->header.stamp;

    baselink_laserframe_transform.setOrigin(tf::Vector3(0, 0, 0));
    btl_q.setRPY(0, 0, 0);
    baselink_laserframe_transform.setRotation(btl_q);
  
    map_baselink_transform.setOrigin(tf::Vector3(0, 0, 0));
    mtb_q.setRPY(0, 0, 0);
    map_baselink_transform.setRotation(mtb_q);

    pub.sendTransform(tf::StampedTransform(
    map_baselink_transform, s_t,"/map","/odom"));
    pub.sendTransform(tf::StampedTransform(
    baselink_laserframe_transform, s_t, "/base_link", "/laser_frame"));
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("laser_scan", 600, tfpubCallback);

  ros::spin();
  return 0;
};
