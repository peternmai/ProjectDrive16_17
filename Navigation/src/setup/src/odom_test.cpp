#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

static tf::Transform odom_baselink;
static tf::Quaternion otb_q;
static ros::Time s_t;

static void odomCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static tf::TransformBroadcaster odom;
  s_t = msg->header.stamp;

  tf::quaternionMsgToTF(msg->orientation, otb_q);
  odom_baselink.setRotation(otb_q);
}

static void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  odom_baselink.setOrigin(tf::Vector3(msg->post.position.x, msg->pose.position.y,
  msg->pose.position.z))
}

int main(int argc, char** argv){
   ros::init(argc, argv, "odom_pub");
   ros::NodeHandle n;

   tf::TransformBroadcaster odom;

   ros::Subscriber isub = n.subscribe("imu", 600, odomCallback);
   ros::Subscriber psub = n.subscribe("slam_out_pose", 600, poseCallback);

   while(n.ok()) {
     ros::spinOnce();
     odom.sendTransform(tf::StampedTransform(odom_baselink, s_t, "/odom",
     "/base_link"));
     n.sleep();
 }

