#include "nav_headers.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle n;

  ros::Rate r(OP_FREQ);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()) {
  broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        //btVector is the laser offset from the robot base
	//will need real values
	ros::Time::now(), "base_link", "base_laser"));
    r.sleep();
  }
}
