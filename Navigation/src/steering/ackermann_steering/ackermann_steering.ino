/**
 * Arduino ROS Node for Ackermann Steering
 * Used for controlling car like RC vehicle. For using with with ROS Navigation
 * Stack, please convert geometry_msgs::Twist to ackermann_msgs::AckermannDrive
 * using...
 *
 *    rosrun teb_local_planner_tutorials cmd_vel_to_ackermann_drive.py
 * 
 * MIT License
 * University of California, San Diego
 * IEEE, Project Drive Team 16/17 (C) 2017
 */

#include <Servo.h>
#include <math.h>

#include <ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

const int STEERING_SERVO_PIN = 9;
const int ESC_SERVO_PIN = 10;

//80 - 100
const float MIN_THROTTLE = 80;            // min throttle (0-180)
const float MAX_THROTTLE = 100;           // max throttle (0-180)

const float MIN_STEERING_ANGLE = 20;      // min steering angle (0-180)
const float MAX_STEERING_ANGLE = 160;     // max steering angle (0-180)

const float THROTTLE_WEIGHT_OFFSET = 0.7; // Offset car's weight in throttle
const int   DELAY = 50;

Servo steeringServo;
Servo electronicSpeedController;

// Forward Declaration
void ackermannCallback( const ackermann_msgs::AckermannDriveStamped & ackermann );

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/ackermann_cmd", &ackermannCallback);
ros::NodeHandle nodeHandle;

int lastDirection = 1; //1 = forward, 0 = backward

/**
 * Function:    ackermannCallback()
 * Input:       driveMsg - Ackermann ROS message (throttle, steering angle)
 * Description: This method is invoked whenever a new message appears in ROS regarding
 *              a new throttle/steering specification for the vehicle. This function will
 *              change the steering angle and throttle accordingly to match that of the
 *              subscribed ackermann message.
 */
void ackermannCallback( const ackermann_msgs::AckermannDriveStamped & ackermann )
{
  // Convert steering angle from radian to degree that fits in range of (0-180)
  float steering_angle = ackermann.drive.steering_angle * (180 / M_PI) + 90;
  
  // Get throttle in range of 0-180 and offset the weight
  float throttle = ackermann.drive.speed * 10 + 90;

  if(throttle > 90) {
    throttle += THROTTLE_WEIGHT_OFFSET * 10;
    lastDirection = 1;
  }
  else if(throttle < 90) {
    if(lastDirection == 1) {
      electronicSpeedController.write(90);
      electronicSpeedController.write(70);
      electronicSpeedController.write(90);
    }
    throttle -= THROTTLE_WEIGHT_OFFSET * 10;
    lastDirection = 0;
  }
  
  /*
  float throttle = ackermann.drive.speed * 10 + 90;
  if (throttle > 90)
    throttle += THROTTLE_WEIGHT_OFFSET * 10;
  if (throttle < 90)
    throttle -= THROTTLE_WEIGHT_OFFSET * 10;
  */
  
  // Check for allowed min steering angle
  if( steering_angle < MIN_STEERING_ANGLE )
    steering_angle = MIN_STEERING_ANGLE;
    
  // Check for allowed max steering angle
  if( steering_angle > MAX_STEERING_ANGLE )
    steering_angle = MAX_STEERING_ANGLE;
    
  // Check for allowed min throttle
  if( throttle < MIN_THROTTLE )
    throttle = MIN_THROTTLE;
  
  // Check for allowed max throttle
  if( throttle > MAX_THROTTLE )
    throttle = MAX_THROTTLE;
    
  steeringServo.write( steering_angle );
  electronicSpeedController.write( throttle );
}


/**
 * Function:    setup()
 * Input:       None
 * Description: Run once at initialization. Subscribe to ROS's ackermann_msg
 *              and connect to steering and ESC servo.
 */
void setup() {
  
  // Connect steering and esc servo
  steeringServo.attach( STEERING_SERVO_PIN );
  electronicSpeedController.attach( ESC_SERVO_PIN );
  
  // Initialize node and subscribe to ackermann msg
  nodeHandle.initNode();
  nodeHandle.subscribe( ackermannSubscriber );
  
  // Initialize steering and throttle to neutral
  steeringServo.write(90);
  electronicSpeedController.write(90);
  
  delay(1000);
  
  
}

void loop() { 
  nodeHandle.spinOnce();
  delay(DELAY);
}
