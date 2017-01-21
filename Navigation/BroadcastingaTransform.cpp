/**
 * Name: BroadcastingaTransform.cpp
 * Description: This program is for TF Transformation. A TF Transform allows translation and rotation.
 *
 * Pre-requisite:  Need to have data that is being collected by the laser.
 *
 * Compilation:    g++ BroadcastingaTransform.cpp
 * Run:            a.out
 *
 * Effect/Outcome: Rotation from quaternion and translation from vector
 *
 * Return Value:   None
 */


//Including the ros.h file
#include <ros/ros.h>
/* Need to include tf package to help publishing transforms. This is necessary 
in order to use methods like sendTransform. */
/*“This class provides an easy way to publish coordinate frame transform information. 
It will handle all the messaging and stuffing of messages. And the function prototypes 
lay out all the necessary data needed for each message.” */
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	//Initialize the roscpp Node.
	//argc and arg are to parse remapping arguments.
	//The string is the node name
	ros::init(argc, argv, "robot_tf_publisher");

	//Starting the roscpp Node. This will call ros::start() until the last node is 
	//destroyed, then it will call ros::shutdown().
	ros::NodeHandle n;
	
	//Creates a rate and the parameter is the frequency (type is a double and is measured in Hz).
	ros::Rate r(100);

	//Created a TransformBroadcaster object that sends the base_link -> base_laser transform
	//over the wire.
	tf::TransformBroadcaster broadcaster;
	
	//Checks the state of shutdown
	while(n.ok()){
		//Sending a transform
		broadcaster.sendTransform(
			//Parameters: input, timestamp, frame_id, and child_frame_id
			tf::StampedTransform(
				//The input tf::Quaternion is the rotation transform parameters from scalar (x,y,z,w)
				//The parameters for btVector3 represents the x, y, z coordinates. 
				//As shown below, there is a x offset of 10 cm and z offset of 20cm from the base.
				//ros::Time::now() will publish a timestamp. “base_link” is the parent node name and
				//“base_laser” is the child node name.
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
				ros::Time::now(),"base_link", "base_laser"));
		//ROS will sleep
		r.sleep();
	}
}
