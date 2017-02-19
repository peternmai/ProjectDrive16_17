#include "nav_headers.h"
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle n;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = n.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
  n.advertise<geometry_msgs::Twist>("turtle/command_velocity", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while(n.ok()) {
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base_link","world", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0* atan2(transform.getOrigin().y(),
    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(),2) +
    pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);
    rate.sleep();
   }
   return 0;
}
