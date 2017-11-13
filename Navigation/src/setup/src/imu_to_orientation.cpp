#include "imu_to_orientation.h"

/*This function uses the IMU publisher to get the orientation
  of the car. Note: the orientation that the IMU publishes is
  the orientation relative to the orientation of the IMU when
  it is first initialized (roll, pitch, yaw are all 0 at the 
  position it is initialized). It also records the last 10
  seconds of yaw readings and 3 seconds of pitch readings.
*/
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {


  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3 m(q);
  double y1;
  yaw = tf::getYaw(msg->orientation);

  m.getRPY(roll, pitch, y1);
  r_time = msg->header.stamp;

  /*The IMU publishes from -pi to pi. We wanted 0 to 2pi, so
    this makes the readings from 0 to 2pi */
  if(yaw < 0)
    yaw += 2 * M_PI;

  /*We only really need pitch and yaw so we record those values
    for averages. originally, I called o for orientation because
    we only needed yaw, but then we needed pitch. So, the o list
    is for YAW and the p list is for pitch. Sorry. */
  o.push_back(yaw);
  p.push_back(pitch);

  /*Since the imu is publishing at a rate of 3 per second, if we
    record 30 readings, it is the last 10 seconds */
  if(o.size() > 30)
    o.pop_front();

  if(p.size() > 9)
    p.pop_front();

  /*Since we were doing averages of yaw and pitch, there
    is the issue of adding a 2pi reading and a 0 reading
    and getting an average of just pi. My solution was to make
    the yaw continuous. So, yaw can be values from -infinity to
    infinity. With other calculations, this creates a stable 
    average in terms of navigation. We do this by taking the old
    yaw and comparing it to the new yaw, if the new yaw and old
    yaw cross the 2pi to 0 or 0 to 2pi it counts it.*/
  if(prev_yaw > 5.5 && yaw < 1.5 && prev_yaw != 0)
    cont++;
  if(prev_yaw < 1.5 && yaw > 5.5 && prev_yaw != 0)
    cont--;

  prev_yaw = o.back();
  yaw += cont * 2 * M_PI;

  scaled_o.push_back(yaw);

  if(scaled_o.size() > 30)
    scaled_o.pop_front();

  //getting the totals and size for the averages
  avg_orien = 0;
  old_avg = 0;

  float tot_orien = 0;
  float dur = 0;

  std::list<float>::iterator it;
  for(it = scaled_o.begin(); it != scaled_o.end(); it++) {
    tot_orien += *it;
    dur = dur + 1;
  }

  avg_orien = tot_orien / dur;
  avg_orien = fmod(avg_orien, 2 * M_PI);

  //getting the oldest 5 seconds yaw average
  tot_orien = dur = 0;
  it = scaled_o.begin();

  for(int i = 0; i < 15; i++) {
    tot_orien += *it;
    dur += 1;
    it++;
    if(it == scaled_o.end())
      break;
  }

  old_avg = tot_orien / dur;
  old_avg = fmod(old_avg, 2 * M_PI);

  //getting the average pitch
  double tot_pit = 0;

  std::list<double>::iterator pit;
  for(pit = p.begin(); pit != p.end(); pit++) {
    tot_pit += *pit;
  }

  avg_pitch = tot_pit / p.size();

  //the averages can be negative, but we want only positive
  if(avg_orien < 0)
    avg_orien += 2 * M_PI;

  if(old_avg < 0)
    old_avg += 2 * M_PI;

  yaw = o.back();

  std::cout << "Pitch: " << pitch << "\n";
  std::cout << "The average pitch is: " << avg_pitch <<"\n";
}

/*this mode is for u-turn protocol. We need to u-turn if our 
  average and current are too different. Then, we need to change
  our average when we are u-turning to our correct position.
*/
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

}

/* The main ROS way to publish our orientation message 
   This is a custom message. Information about this message
   can be found by using the command rosmsg info imu_orientation.
   The message can be found in
   ~/ProjectDrive16-17/Sensor/src/msg/msg/imu_orientation.msg*/
int main(int argc, char ** argv) {
  ros::init(argc, argv, "orien_pub");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);
  ros::Subscriber mode_sub = n.subscribe("vehicle_status", 1, modeCallback);
  ros::Publisher imu_pub = n.advertise<msg::imu_orientation>("orientation", 1);
  
  //publishing at 3 per second
  ros::Rate r(3);

  while(n.ok()) {

    ros::spinOnce();

    std::vector<float> old_oriens;
    std::list<float>::iterator it;

    for(it = o.begin(); it != o.end(); it++)
        old_oriens.push_back(*it);

    msg::imu_orientation imu_msg;
    imu_msg.header.stamp = r_time;
    imu_msg.header.frame_id = "orientation";
    imu_msg.orientation = yaw;
    imu_msg.roll = roll;
    imu_msg.pitch = pitch;
    imu_msg.avg_orientation = avg_orien;
    imu_msg.avg_pitch = avg_pitch;
    imu_msg.old_orientations = old_oriens;
    imu_msg.old_avg = old_avg;

    imu_pub.publish(imu_msg);

    r.sleep();
  }
}
