#include "imu_to_orientation.h"
/*
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
  old_avg = 0;
  float tot_orien = 0;
  float tot_old = 0;
  double dur = 0;
  double old_dur = 0;

  for(int i = 0; i < sizeof(old) / sizeof(old[i]); i++) {
    if(old[i] != 0) {
      tot_orien += old[i];
      dur = dur + 1;
    }
  }
  
  avg_orien = tot_orien / dur;

  for(int i = it_idx + 1 % 30; i < it_idx + 1 + 15 % 30; i++) {
    if(old[i] != 0) {
      tot_old += old[i];
      old_dur += 1;
    }
  }

  old_avg = tot_old / old_dur;
  old_avg = fmod(old_avg, 2 * M_PI);
  avg_orien = fmod(avg_orien, 2 * M_PI);
  yaw = fmod(yaw, 2 * M_PI);
  
  
  if(avg_orien < 0)
    avg_orien += 2 * M_PI;
  
  std::cout << "The avg orientation is: " << avg_orien << "\n";
  
}
*/

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  yaw = tf::getYaw(msg->orientation);
  r_time = msg->header.stamp;

  if(yaw < 0)
    yaw += 2 * M_PI;

  if(prev_yaw > 5.5 && yaw < 1.5 && prev_yaw != 0)
    cont++;
  if(prev_yaw < 1.5 && yaw > 5.5 && prev_yaw != 0)
    cont--;

  prev_yaw = yaw;
  yaw += cont * 2 * M_PI;

  o.push_back(yaw);

  if(o.size() > 30)
    o.pop_front();

  avg_orien = 0;
  old_avg = 0;

  float tot_orien = 0;
  float dur = 0;

  std::list<float>::iterator it;
  for(it = o.begin(); it != o.end(); it++) {
    tot_orien += *it;
    dur = dur + 1;
  }

  avg_orien = tot_orien / dur;
  avg_orien = fmod(avg_orien, 2 * M_PI);
  yaw = fmod(yaw, 2 * M_PI);

  tot_orien = dur = 0;
  it = o.begin();

  for(int i = 0; i < 15; i++) {
    tot_orien += *it;
    dur += 1;
    it++;
    if(it == o.end())
      break;
  }

  old_avg = tot_orien / dur;
  old_avg = fmod(old_avg, 2 * M_PI);

  if(avg_orien < 0)
    avg_orien += 2 * M_PI;

  if(old_avg < 0)
    old_avg += 2 * M_PI;

  if(yaw < 0)
    yaw += 2 * M_PI;
  
  std::cout << "Yaw: " << yaw << " Prev_Yaw: " << prev_yaw << "\n";
  std::cout << "The average orientation is: " << avg_orien <<"\n";
  std::cout << "The old average orientation is: " << old_avg << "\n";
}

void modeCallback(const msg::vehicle_status::ConstPtr & msg) {
  int mode = msg->drive_mode;
  std::list<float>::iterator it;

  if(mode == 1) {
    for(int i = 0; i < o.size(); i++) {
      o.push_back(yaw);
      o.pop_front();
      
  }
  mode = 0;
}

//  if(mode == 1) {
//    for(int i = 0; i < sizeof(old) / sizeof(old[i]); i++)
//      old[i] = yaw;
//  }
}

/* The way to use the imu to orientation */
int main(int argc, char ** argv) {
  ros::init(argc, argv, "orien_pub");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);
  ros::Subscriber mode_sub = n.subscribe("vehicle_status", 1, modeCallback);
  ros::Publisher imu_pub = n.advertise<msg::imu_orientation>("orientation", 1);

  ros::Rate r(3);

  while(n.ok()) {

    ros::spinOnce();

    std::vector<float> old_oriens;
    std::list<float>::iterator it;
    for(it = o.begin(); it != o.end(); it++) {
      if(*it < 0)
        old_oriens.push_back(*it + 2 * M_PI);
      else
        old_oriens.push_back(*it);
    }
    yaw = o.back();
    yaw = fmod(yaw, 2 * M_PI);
    if(yaw < 0)
      yaw += 2 * M_PI;
    msg::imu_orientation imu_msg;
    imu_msg.header.stamp = r_time;
    imu_msg.header.frame_id = "orientation";
    imu_msg.orientation = yaw;
    imu_msg.avg_orientation = avg_orien;
    imu_msg.old_orientations = old_oriens;
    imu_msg.old_avg = old_avg;

    imu_pub.publish(imu_msg);

    r.sleep();
  }
}
