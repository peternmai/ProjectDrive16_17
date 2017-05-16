/******************************************************************************

                                                         Author: Jason Ma
                                                         Date:   May 21 2017
                                   lidarAngleCalcv2

 File Name:       lidarAngleCalcv2.ino
 Description:     Uses light sensor to determine what angle lidar is reading at.
******************************************************************************/

#include <ros.h>
#include <msg/optical_encoder.h>

/*[Config Vars]--------------------------------------------------------------*/

//connection type (not sure if needed)
#define USB_CON

//use serial mon or node handler
#define SERIAL_MON 0

//threshold after which light sensor readings trigger state change
#define SAME_THRES 3

//pins
#define PIN_POTEN 0
#define PIN_MOTOR 3
#define PIN_LIGHT 5
#define PIN_LED   13

/*[Global Vars]--------------------------------------------------------------*/

ros::NodeHandle nh;
msg::optical_encoder message;
ros::Publisher p("optical_encoder", &message);
ros::Time now = nh.now();

unsigned long times[15];          //first times of 15 notches
unsigned long lengths[15];        //lengths of 15 notches (in units of time)
unsigned long lastArd = millis(); 
unsigned long nowArd = millis();  

double lastAngle;                 //last angle returned to ROS
int currNotch;                    //current notch
int rotations;                    //rotations since last broadcast
bool lastPoints[SAME_THRES];      //last few light readings for smoothing
bool currState;                   //light sensor stabilized state

/*---------------------------------------------------------------------------*
Method Name: setup
File:        lidarAngleCalcv2.ino
 
Description: Initialize program.
*----------------------------------------------------------------------------*/
void setup() {
  
  //set up either serial mon or node handler
  if(SERIAL_MON) {
    Serial.begin(9600);
    Serial.println("Start setup");
  }
  else {
    nh.initNode();
    nh.advertise(p);
  }
  
  //set up pins
  pinMode(PIN_POTEN, INPUT);
  pinMode(PIN_LIGHT, INPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  analogWrite(PIN_MOTOR, 100);
  digitalWrite(PIN_LED, LOW);
  
  currState = 0;
  lastAngle = 0;
  currNotch = 0;
  rotations = 0;
}

/*----------------------------------------------------------------------------*
 Method Name: loop
 File:        lidarAngleCalcv2.ino
  
 Description: Constantly runs, updates light status and motor speed
 *----------------------------------------------------------------------------*/
void loop() {
  
  //set motor speed
  //if(nh.connected())
    analogWrite(PIN_MOTOR, analogRead(PIN_POTEN) / 4);
  //else
  //  analogWrite(PIN_MOTOR, 0);
  
  //update last points with latest light value
  pollLight();
  
  //note: stateDetector updates currState
  //check whether state change has occured
  if(currState != lightStateDetector()) {
    //notch blocking light sensor
    if(currState == 0) {
      //digitalWrite(PIN_LED, HIGH);
      
      //store time in times
      times[currNotch] = millis();
    }
    //ligiht sensor unobstructed, notch was read in
    else {
      int minInd;
      //digitalWrite(PIN_LED, LOW);
      
      //check difference in time from last state change
      lengths[currNotch] = millis() - times[currNotch];
      
      //new notch has been read in
      currNotch = (currNotch + 1) % 15;
      
      //15 notches read in
      if(currNotch == 0) {
        rotations += 1;

        //attempt to find unique smallest notch
        minInd = checkCalibration();
        
        //notch found, not at expected loc
        if(minInd > 0) {
          //shift values in array so that notch is at "0" location
          for(int i = minInd; i < 15; i++) {
            times[i - minInd] = times[i];
            lengths[i - minInd] = lengths[i];
          }
      
          //adjust currNotch so it is "pointing" after the last read notch time
          currNotch = 14 - minInd + 1;
        }
      }
      
      //handle notch logic
      if(SERIAL_MON) {        
        if(minInd != -1) {
            Serial.println("+");
          }
          else {
            Serial.println("B");
          }
        //Serial.print("Notch read");
      }
      else {
        now = nh.now();   
        nowArd = millis();
        message.time = now;
        message.angle = currNotch * 0.418879;
        message.avg_angular_velocity = 0.418879 / ((double) (nowArd - lastArd) / 1000);
        lastArd = nowArd;
        
        p.publish(&message);
        nh.spinOnce();
      }
    }
  }
}

 
/*----------------------------------------------------------------------------*
 Method Name: checkCalibration
 File:        lidarAngleCalcv2.ino
  
 Description: Check how many unique small notches can be determined from the
              last 15 notches
              
 Parameter Descriptions:
 name               description
 ------------------ -----------------------------------------------
 <return>           index of smallest notch (-1 if not unique)
 *----------------------------------------------------------------------------*/
int checkCalibration() {
  int i, avgLen, count, minInd;
  
  if(SERIAL_MON)
    Serial.println("Calibration begin");
  
  //calculate average length of notch as a threshold for the small notch
  for(i = 0; i < 15; i++)
    avgLen += lengths[i];
  avgLen /= 15.0;

  count = 0;
  minInd = -1;

  //calculate average length of notch as a threshold for the small notch
  for(i = 0; i < 15; i++)
    avgLen += lengths[i];
  avgLen /= 15.0;

  //if can uniquely determine one notch that is short, set that notch to be the "0" notch
  for(i = 0; i < 15; i++) {
    //finds number of lengths lower than average, if things are working properly should only be 1.
    if(lengths[i] < avgLen - 4) {
      count++;
      minInd = i;
    }
  }
  
  //not uniquely determined
  if(count != 1) {
    digitalWrite(PIN_LED, HIGH);
    return -1;
  }

  digitalWrite(PIN_LED, LOW);
  
  //uniquely determined min index
  return minInd;
}

/*----------------------------------------------------------------------------*
 Method Name: lightStateDetector
 File:        lidarAngleCalcv2.ino
  
 Description: Uses last light readings to detect changes in light state
              robustly.
              
 Parameter Descriptions:
 name               description
 ------------------ -----------------------------------------------
 <return>           current state
 *----------------------------------------------------------------------------*/
bool lightStateDetector() {
  int i, count;

  count = 0;
  
  //count the number of 1 readings
  for(i = 0; i < SAME_THRES; i++)
    if(lastPoints[i])
      count++;

  //all readings must be 1 or 0 to trigger a state change
  if(count == 0)
    currState = 0;
  else if(count == SAME_THRES)
    currState = 1;
  
  return currState;
}

/*----------------------------------------------------------------------------*
 Method Name: pollLight
 File:        lidarAngleCalcv2.ino
  
 Description: Reads status of light sensor and stores it in lastPoints
 *----------------------------------------------------------------------------*/
void pollLight() {
  //if(debug)
  //  Serial.println("Start pollLight");
  int i;

  //shift lastPoints data
  for(i = 0; i < SAME_THRES - 1; i++)
    lastPoints[i] = lastPoints[i + 1];

  //read in new data
  lastPoints[SAME_THRES - 1] = digitalRead(PIN_LIGHT);
}

