#include "nav_headers.h"
#include <tf/transform_listener.h>

float terarangerone_ranges[NUM_READINGS] = {0};
int callback_instance = 0;

void tf_transformCallback(const sensor_msgs::Range::ConstPtr& msg) {
  terarangerone_ranges[callback_instance] = msg->ranges;
  callback_instance = callback_instance % NUM_READINGS;
}

void transformPoint(const tf::TransformListener& listener) {
  ros::NodeHandle n;
  ros::Subscriber anglesub = n.subscribe("angle_publisher", NUM_READINGS,
  tf_transformCallback);

  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  laser_point.header.stamp = ros::Time();

  //need to change based on which direction lidar goes
  laser_point.point.x = terarangerone_ranges[callback_instance];
  laser_point.point.y = terarangerone_ranges[callback_instance];
  laser_point.point.z = 0;

  try{
  //creating a message listener object to listen for specifically geometry
  //messages
  geometry_msgs::PointStamped base_point;
  listener.transformPoint("base_link", laser_point, base_point);
  }
  catch(tf::TransformException& ex) {
    //ROS_ERROR("Received an exception trying to transform a point from
    base_laser to base_link");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  ros::Timer timer = n.createTimer(ros::Duration(OP_FREQ),
  boost::bind(&transformPoint, boost::ref(listener)));
  ros::spin();
}
