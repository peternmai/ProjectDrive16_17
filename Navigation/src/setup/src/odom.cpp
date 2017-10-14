#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

void odomCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static tf::TransformBroadcaster odom;
  ros::Time s_t = msg->header.stamp;


  //need a new way to turn linx -> velx -> position
  tf::Transform odom_baselink;
  tf::Quaternion otb_q;
  tf::quaternionMsgToTF(msg->orientation, otb_q);
  double roll, pitch, yaw;
  tf::Matrix3x3 m(otb_q);
  m.getRPY(roll, pitch, yaw);
  std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << "\n";
  odom_baselink.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  odom_baselink.setRotation(otb_q);

  odom.sendTransform(tf::StampedTransform(odom_baselink, s_t, "/odom",
    "base_link"));

}

int main(int argc, char** argv){
   ros::init(argc, argv, "odom_pub");
   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("imu", 600, odomCallback);

   ros::spin();
   return 0;
 }

