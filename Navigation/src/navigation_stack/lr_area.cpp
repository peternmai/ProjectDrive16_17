#include <sensor_msgs/LaserScan.h>

#define MAX_LENGTH 5
#define PUBLISH_RATE 3
#define LASER_FREQUENCY 1000
#define MAX_READINGS long(LASER_FREQUENCY / PUBLISH_RATE)

static float left_area;
static float right_area;
static ros::Time r_t;

static float ranges[MAX_READINGS] = {0};
static float angles[MAX_READINGS] = {0};


static void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  
  for(int i = 0; i < msg->ranges.length; i++) {
    ranges[i] = msg->ranges[i] > 5 ? 5 : msg->ranges[i];
    angles[i] = msg->angle_min + msg->angle_increment * i;
  }
  r_t = msg->header.stamp;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "lra_pub");

  ros::NodeHandle n;
  ros::Subscriber laserscan_sub = n.subscribe("scan", MAX_READINGS,
  laserCallback);

  ros::Rate r(PUBLISH_RATE);

  while(n.ok()) {
    

