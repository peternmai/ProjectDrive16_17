#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	std::cout << "I heard something: ";
	for (int i = 0; i < msg->ranges.size(); i++) {
		std::cout << msg->ranges[i] << " ";
	}
	std::count << std::endl;
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "range_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("laser_scan", 1000, chatterCallback);
	ros::spin();

	std::cout << "Publishing Laswer Scan Information..." << std::endl;
	ros::init(argc, argv, "laswer_scan_publisher");
	ros::Publisher scan_pub =
	n.advertise<sensor_msgs::LaserScan>("laser_scan", 50);

	unsigned int num_readings = 100;
	double laswer_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];
	int count = 0;

	ros::Rate r (1.0);

	while (n.ok()) {
		for (unsigned int i = 0; i < num_readings; i++) {
			ranges[i] = count;
			intensities[i] = 100 + count;
		}
		ros::Time scan_time = ros::Time::now();
		
		std::cout << scan_time << std::endl;
		
		sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_frame";

		scan.angle_min = -1.57;
		scan.angle_max = 1.57;
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		scan.angle_increment = 3.14 / num_readings;
		scan.time_increment = (1 / laser_frequency) / (num_readings);
		scan.ranges.resize(num_readings);
		scan.intensities.resize(num_readings);
		for(unsigned int i = 0; i < num_readings; i++) {
			scan.ranges[i] = ranges[i];
			scan.intensities[i] = intensities[i];
		}
		
		scan_pub.publish(scan);
		++count;
		r.sleep();
	}
	return 0;
}
