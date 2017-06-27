#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

void odomCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static tf::TransformBroadcaster odom;
  ros::Time s_t = msg->header.stamp;

  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double w = msg->orientation.w;

  /*either these equations or roll and pitch are + for first term
  **Peter's way with base footprint and 2D odom */
  double ysqr = y * y;
  double t0 = 2.0 * (w * x + y * z);
  double t1 = 1.0 - 2.0 * (x * x + ysqr);
  double roll = atan2(t0, t1);

  double t2 = 2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2);
  
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (ysqr + z * z);
  double yaw = atan2(t3,t4);

  //need a new way to turn linx -> velx -> position
  tf::Transform odom_baselink;
  tf::Quaternion otb_q;
  tf::quaternionMsgToTF(msg->orientation, otb_q);
  //otb_q.setRPY(roll, pitch, yaw);
  odom_baselink.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  odom_baselink.setRotation(otb_q);

  odom.sendTransform(tf::StampedTransform(odom_baselink, s_t, "/odom",
    "base_link"));

  /*2D odom method
  tf::Transform odom_baseft;
  tf::Transform baseft_base;
  tf::Quaternion otbf_q;
  tf::Quaternion bftb_q;

  odom_baseft.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  baseft_base.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

  otbf_q.setRPY(roll, pitch, yaw);
  bftb_q.setRPY(roll, pitch, yaw);

  odom_baseft.setRotation(otbf_q);
  baseft_base.setRotation(bftb_q);

  odom.sendTransform(tf::StampedTransform(odom_baseft, s_t, "/odom",
    "base_footprint"));
  odom.sendTransform(tf::StampedTransform(baseft_base, s_t, "/base_footprint",
    "/base_link"));
  */
}

int main(int argc, char** argv){
   ros::init(argc, argv, "odom_pub");
   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("imu", 600, odomCallback);

   ros::spin();
   return 0;
 }

