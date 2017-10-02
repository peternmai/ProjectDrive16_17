#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

static const float MIN_LASER_SCAN_RADIUS = 0.22;

static sensor_msgs::LaserScan filtered_scan;


static void laserScanCallback( const sensor_msgs::LaserScan::ConstPtr & raw ) {
  filtered_scan = *raw;
  for( int i = 0; i < raw->ranges.size(); i++ ) {
    if( raw->ranges[i] < MIN_LASER_SCAN_RADIUS || 
      raw->ranges[i] == std::numeric_limits<float>::infinity()  )
        filtered_scan.ranges[i] = 0;
  }
}

int main( int argc, char ** argv ) {

  // Anounce ROS this program is running
  ros::init(argc, argv, "LaserFilterNode");

  // Create publisher and subscriber
  ros::NodeHandle n;
  ros::Publisher filtered_scan_pub = n.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);
  ros::Subscriber raw_scan_sub = n.subscribe("scan", 1, laserScanCallback );

  int rate = 0;
  n.getParam("/sweep_node/rotation_speed", rate);  
  ros::Rate r( rate );

  while(n.ok()) {
    ros::spinOnce();
    filtered_scan_pub.publish(filtered_scan);
    r.sleep();
  }
}
