#include "imu_to_orientation.h"

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  yaw = tf::getYaw(msg->orientation);
  r_time = msg->header.stamp;
  
  if(yaw < 0)
    yaw += 2 * M_PI;

  if(prev_yaw > 5.5 && yaw < 1.5 && prev_yaw != 0)
    cont++;
  if(prev_yaw < 1.5 && yaw > 5.5 && prev_yaw != 0)
    cont--;

  std::cout << "YAW: " << yaw << " PREV_YAW: " << prev_yaw << "\n";
  std::cout << "CONT: " << cont << "\n";
  
  prev_yaw = yaw;
  yaw += cont * 2 * M_PI;

  old[it_idx] = yaw;
  it_idx++;
  it_idx = it_idx % 30;

  avg_orien = 0;
  float tot_orien = 0;
  double dur = 0;

  for(int i = 0; i < sizeof(old) / sizeof(old[i]); i++) {
    if(old[i] != 0) {
      tot_orien += old[i];
      dur = dur + 1;
    }
  }
  
  avg_orien = tot_orien / dur;
  
  avg_orien = fmod(avg_orien, 2 * M_PI);
  yaw = fmod(yaw, 2 * M_PI);
  
  
  if(avg_orien < 0)
    avg_orien += 2 * M_PI;

  std::cout << "The average orientation in the past " << dur / double(3);
  std::cout << " seconds is " << avg_orien << "\n";

}

/* The way to use the imu to orientation */
int main(int argc, char ** argv) {
  ros::init(argc, argv, "orien_pub");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);
  ros::Publisher imu_pub = n.advertise<msg::imu_orientation>("orientation", 1);

  ros::Rate r(5);

  while(n.ok()) {

    std::vector<float> old_oriens;
    for(int i = 0; i < sizeof(old) / sizeof(old[0]); i++)
      old_oriens.push_back(fmod(old[i], 2 * M_PI));

    ros::spinOnce();
    
    msg::imu_orientation imu_msg;
    imu_msg.header.stamp = r_time;
    imu_msg.header.frame_id = "orientation";
    imu_msg.orientation = yaw;
    imu_msg.avg_orientation = avg_orien;
    imu_msg.old_orientations = old_oriens;

    imu_pub.publish(imu_msg);

    r.sleep();
  }
} /*
*/
