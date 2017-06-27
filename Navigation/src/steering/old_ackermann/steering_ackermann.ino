
/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

ros::NodeHandle nodeHandle;
// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)

const float minController = -3.14; //min angle change controller command (rads)
const float maxController = 3.14;  //max angle change controller command (rads)
const float minSteering = -0.785;  //min angle for steering (rads)
const float maxSteering = 0.785;   //max angle for steering (rads)
const float minThrottle = 70;      //min throttle (0-180)
const float maxThrottle = 110;     //max throttle (0-180)
const float minSpeed = 0;          // min speed (m/s)
const float maxSpeed = 9;          // max speed (m/s)

const float STEERING_LIMIT_FACTOR = 0.05; //limit the speed of turning with this
const int DELAY = 50;             //delay between spinning

float currAngle = 0;
double lastAngle = 90;

Servo steeringServo;
Servo electronicSpeedController;  // The ESC on the TRAXXAS works like a Servo

// std_msgs::Int32 str_msg;
// ros::Publisher steering_angle("steering_angle", &str_msg); 

ackermann_msgs::AckermannDrive ackermann_msg;
ros::Publisher steering_ackermann("steering_ackermann", &ackermann_msg);

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void smoothTurn(double startAngle, double endAngle, int steps) {
  
  double stepSize = (endAngle - startAngle) / steps;
  double currAngle = startAngle;
  for(int i = 0; i < steps; i++) {
    currAngle += stepSize;
    steeringServo.write(currAngle);
    delay(5);
  }
}

void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{  
  float angleChange = (float) (fmap(twistMsg.angular.z, 0.0, 1.0, minController, maxController) / (1.0 / STEERING_LIMIT_FACTOR));
  // The following could be useful for debugging
  currAngle += angleChange;
  // Check to make sure steeringAngle is within car range
  if (currAngle < minSteering) { 
    currAngle = minSteering;
  }
  if (currAngle > maxSteering) {
    currAngle = maxSteering;
  }
  
  double steerAngle = fmap(currAngle, minSteering, maxSteering, 0, 180);
  
  if(steerAngle > lastAngle + 0.005 || steerAngle < lastAngle - 0.005)
    steeringServo.write(steerAngle);
    //smoothTurn(lastAngle, steerAngle, 10);
  lastAngle = steerAngle;

  // ESC forward is between 0.5 and 1.0
  int escCommand;
  /*
  if (twistMsg.linear.x >= 0.5) {
    escCommand = (int)fmap(twistMsg.linear.x, 0.5, 1.0, 90.0, maxThrottle);
  } else {
    escCommand = (int)fmap(twistMsg.linear.x, 0.0, 1.0, 0.0, 180.0);
  }
  */
  
  //Write speed command to ESC
  escCommand = (int) fmap(twistMsg.linear.x, -9.0, 9.0, minThrottle, maxThrottle);
  electronicSpeedController.write(escCommand);

  //Calculate desired speed for ackermann_msg
  float vehicleSpeed = (float) fmap(twistMsg.linear.x, -9, 9, minSpeed, maxSpeed);
  
  // Check to make sure speed for ackermann_msg is within bounds
  if (vehicleSpeed < minSpeed)
    vehicleSpeed = minSpeed;
  else if (vehicleSpeed > maxSpeed)
    vehicleSpeed = maxSpeed;
  
  //TODO only for debugging
  vehicleSpeed = escCommand;
  
  ackermann_msg.steering_angle = currAngle;
  ackermann_msg.steering_angle_velocity = 0;
  ackermann_msg.speed = vehicleSpeed;
  ackermann_msg.acceleration = 0;
  ackermann_msg.jerk = 0;
  steering_ackermann.publish(&ackermann_msg);
  
  //Don't know if this serves any practical function, but not going to touch this.
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

  //DEBUG
  //Serial.println("end of driveCallback");
  
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/steering_cmd", &driveCallback);

void testRun() {
  steeringServo.write(90);
  delay(10);
}

void setup(){
  steeringServo.attach(9);
  //steeringServo.write(90);
  electronicSpeedController.attach(10); // ESC is on pin 10
  
  /*
  for(int i = 0; i < 1000; i++)
    testRun();
  
  electronicSpeedController.write(90);
  delay(1000);
  electronicSpeedController.write(100);
  for(int i = 0; i < 1000; i++)
    testRun();
  electronicSpeedController.write(90);
  */
  
  //Set up node
  nodeHandle.initNode();
  nodeHandle.advertise(steering_ackermann);

  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber);
  
  pinMode(13, OUTPUT);

  // Attach the servos to actual pins
  //steeringServo.attach(9); // Steering servo is attached to pin 9
  //electronicSpeedController.attach(10); // ESC is on pin 10
  
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.write(90);
  electronicSpeedController.write(90);
  
  //Don't enable both serial and nodeHandle at same time
  //Serial.begin(115200);
  //Serial.println("Done initializing");
  delay(1000);
}

void loop(){
  nodeHandle.spinOnce();
  delay(DELAY);
}
