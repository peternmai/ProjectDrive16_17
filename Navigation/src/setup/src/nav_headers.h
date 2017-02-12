//<ros/ros.h> is a header file to utilise all of the ros functions
#include <ros/ros.h>

//these message headers are to manipulate the messages. needed for laser_pub...
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

//headers needed for tf_pub and tf_sub
#include <geometry_msgs/PointStamped.h>
//#include <ros/angles.h>
#include <std_msgs/String.h>

//general c++ headers for input and output
#include <iostream>
#include <string>

//our operational frequency and the amount of readings we want to keep
#define OP_FREQ 600
#define NUM_READINGS 100


