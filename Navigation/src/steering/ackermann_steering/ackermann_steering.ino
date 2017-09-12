/**
 * Arduino ROS Node for Ackermann Steering
 * Used for controlling car like RC vehicle. Please convert geometry_msgs::Twist
 * to ackermann_msgs::AckermannDrive using...
 *
 *    rosrun teb_local_planner_tutorials cmd_vel_to_ackermann_drive.py
 *
 * Note: This was tested on the XERUN SCT PRO ESC.
 * 
 * MIT License
 * University of California, San Diego
 * IEEE, Project Drive Team 16/17 (C) 2017
 */

#include <Servo.h>
#include <math.h>

#include <std_msgs/Float64.h>

#include <ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

const int STEERING_SERVO_PIN = 9;
const int ESC_SERVO_PIN = 10;
const int PIN_LED = 13;

const int SDA_PIN = 18;
const int SCL_PIN = 19;

const float NEUTRAL_THROTTLE = 90;
const float NEUTRAL_STEERING_ANGLE = 90;

const float MIN_THROTTLE = 60;             // min throttle (0-180)
const float MAX_THROTTLE = 100;            // max throttle (0-180)

const float MIN_STEERING_ANGLE = 20;       // min steering angle (0-180)
const float MAX_STEERING_ANGLE = 160;      // max steering angle (0-180)

const float THROTTLE_FORWARD_OFFSET  = 7;  // Offset car's weight in throttle (m/s) * 10
const float THROTTLE_BACKWARD_OFFSET = 28; // Offset car's weight in throttle (m/s) * 10

const int   DELAY = 5;

Servo steeringServo;
Servo electronicSpeedController;

// Forward Declaration
void ackermannCallback( const ackermann_msgs::AckermannDriveStamped & ackermann );

// Throttle/Steering Command Subscriber
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/ackermann_cmd", &ackermannCallback);
ros::NodeHandle nodeHandle;

// Vehicle State
const int FORWARD    =  1;
const int STOP       =  0;
const int BACKWARD   = -1;
int lastVehicleState = STOP;

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
  if (throttle > NEUTRAL_THROTTLE)
    throttle += THROTTLE_FORWARD_OFFSET;
  if (throttle < NEUTRAL_THROTTLE)
    throttle -= THROTTLE_BACKWARD_OFFSET;
  
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
  
  // Switches ESC to backward mode if necessary
  if( throttle <= NEUTRAL_THROTTLE - THROTTLE_BACKWARD_OFFSET ) {
    if( lastVehicleState != BACKWARD ) {
      electronicSpeedController.write( NEUTRAL_THROTTLE );
      delay(50);
      electronicSpeedController.write( MIN_THROTTLE );
      delay(50);
      electronicSpeedController.write( MIN_THROTTLE );
      delay(50);
      electronicSpeedController.write( NEUTRAL_THROTTLE );
      delay(50);
    }
  }
  
  // Update last vehicle's state
  if( throttle >= NEUTRAL_THROTTLE + THROTTLE_FORWARD_OFFSET )
    lastVehicleState = FORWARD;
  else if ( throttle <= NEUTRAL_THROTTLE - THROTTLE_BACKWARD_OFFSET )
    lastVehicleState = BACKWARD;
  else
    lastVehicleState = STOP;
  
  // Send out steering and throttle commands
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
  steeringServo.write( NEUTRAL_STEERING_ANGLE );
  electronicSpeedController.write( NEUTRAL_THROTTLE );
  
  delay(1000);
  
}

void loop() {
  
  // Stop car if ROS connection lost
  if( !nodeHandle.connected() ) {
    steeringServo.write( NEUTRAL_STEERING_ANGLE );
    electronicSpeedController.write( NEUTRAL_THROTTLE );
  }
  
  nodeHandle.spinOnce();
  delay(DELAY);
}
