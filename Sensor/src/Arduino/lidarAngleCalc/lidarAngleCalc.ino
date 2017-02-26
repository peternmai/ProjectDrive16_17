 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Feb 02 2017
                                   lidarAngleCalc

 File Name:       lidarAngleCalc.ino
 Description:     Uses light sensor to determine what angle lidar is reading at.
 *****************************************************************************/

//TODO implement threshold before state changes can occur
//#define STATE_CHANGE_THRESHOLD ____
#define USB_CON
#define SAME_THRES 3

#include <ros.h>
#include <msg/optical_encoder.h>

ros::NodeHandle nh;
msg::optical_encoder message;
ros::Publisher p("optical_encoder", &message);
ros::Time now = nh.now();
unsigned long times[15];   //first times of 15 notches
unsigned long lengths[15]; //lengths of 15 notches (in units of time)
unsigned long lastArd = millis();
unsigned long nowArd = millis();
double lastAngle;   //last angle returned to ROS
bool lastPoints[SAME_THRES]; //last 5 light readings for smoothing
int currNotch;      //current notch
int lightPin = 5;   //pin id of light sensor
int motorPin = 3;   //pin id of lidar motor
int debugInt;
int tries;
bool currState;     //state of light
bool debugBool;
bool debug = true;

 /********************************************************************
 | Routine Name: setup
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads in 15 notches, initializes some values.
 ********************************************************************/
void setup() {
  nh.initNode();
  nh.advertise(p);
  
  int i;
  
  if(debug) {
    Serial.begin(9600);
    Serial.println("Start setup");
  }
  
  pinMode(0, INPUT);
  pinMode(lightPin, INPUT);
  pinMode(motorPin, OUTPUT);
  
  analogWrite(motorPin, 200);
    
  currState = 0;
  lastAngle = 0;
  currNotch = 0;

  debugInt = 0;
  debugBool = false;
  
  /*
  for(i = 0; i < 10; i++) {
    message.time = nh.now();
    message.angle = 0;
    message.avg_angular_velocity = 0;
    
    p.publish(&message);
    nh.spinOnce();
    delay(1000);
  }
  */
  
  recalibrateNotch();
}

 /********************************************************************
 | Routine Name: loop
 | File:         lidarAngleCalc.ino
 | 
 | Description: Sees if recallibration is needed, then 
 ********************************************************************/
void loop() {
  //if(debug)
  //  Serial.println("Start loop");
  double avgLen;
  int i, minLen, count;
  
  analogWrite(motorPin, analogRead(0) / 4);
  
  recalibrateNotch();

  //calculate average length of notch as a threshold for the small notch
  for(i = 0; i < 15; i++)
    avgLen += lengths[i];
  avgLen /= 15.0;

  count = 0;
  minLen = -1;
  tries = 0;
  //try to uniquely determine small notch, and recalibrate until it can be determined.
  while(count != 1) {
    
    analogWrite(motorPin, analogRead(0) / 4);
    
    count = 0;
    minLen = -1;

    //calculate average length of notch as a threshold for the small notch
    for(i = 0; i < 15; i++)
      avgLen += lengths[i];
    avgLen /= 15.0;
  
    //if can uniquely determine one notch that is short, set that notch to be the "0" notch
    for(i = 0; i < 15; i++) {
      //finds number of lengths lower than average, if things are working properly should only be 1.
      if(lengths[i] < avgLen - 4) {
        count++;
        minLen = i;
      }
    }
    
    if(debug) {
      Serial.print(count);
      
      //Serial.print(avgLen);
      Serial.print('\n');
  
      /*
      Serial.print("Lens:\t");
      for(i = 0; i < 15; i++) {
        Serial.print(lengths[i]);
        Serial.print(" ");
      }
      Serial.print('\n');
      */
    }

    if(count != 1) {
      if(debug)
        Serial.println("B");
      currNotch = 0;
      recalibrateNotch();
    }
    
    tries++;
  }
  //Serial.println("+");

  //small notch uniquely determined
  if(minLen > 0) {
    //shift values in array so that notch is at "0" location
    for(i = minLen; i < 15; i++) {
      times[i - minLen] = times[i];
      lengths[i - minLen] = lengths[i];
    }

    //adjust currNotch so it is "pointing" after the last read notch time
    currNotch = 14 - minLen + 1;
  }

  //continue reading notches
  readNotch();

  /*
  Serial.print("Times:\t");
  //print times and lengths
  for(i = 0; i < 15; i++) {
    Serial.print(times[i]);
    Serial.print(" ");
  }
  Serial.print('\n');

  Serial.print("Lens:\t");
  for(i = 0; i < 15; i++) {
    Serial.print(lengths[i]);
    Serial.print(" ");
  }
  Serial.print('\n');
  */
  
  /*
  now = nh.now();
  nowArd = millis();
    
  message.time = now;
  message.angle = getAngle(nowArd, true);
  message.avg_angular_velocity = message.angle / ((double) (nowArd - lastArd) / 1000);
  
  lastArd = nowArd;
  */
  
  //p.publish(&message);
  //nh.spinOnce();
}

 /********************************************************************
 | Routine Name: recalibrateNotch
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads 15 notches
 ********************************************************************/
void recalibrateNotch() {
  //if(debug)
  //  Serial.println("Start recalibrate");
  int i;
  
  //initialize all times and lengths to 0
  for(i = 0; i < 15; i++) {
    times[i] = 0;
    lengths[i] = 0;
  }
  
  //fill up times and lengths array
  for(i = 0; i < 15; i++) {
    
    /*
    if(i == 8) {
      now = nh.now();
      nowArd = millis();
    }
    */
    readNotch();
  }
}

 /********************************************************************
 | Routine Name: stateDetector
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads one notch, records its time and length
 ********************************************************************/
void readNotch() {
  //if(debug)
  //  Serial.println("Start readNotch");
  while(stateDetector() == 1) {
      pollLight();
    }     
    
    //store first 1 in times[i] 
    times[currNotch] = millis();
    
    //check to make sure first 1 is reasonably spaced from previous times

    //read until 5 0s are read in a row
    while(stateDetector() == 0) {
      pollLight();
    }

    lengths[currNotch] = millis() - times[currNotch];

    currNotch = (currNotch + 1) % 15;
    
  now = nh.now();   
  nowArd = millis();
  message.time = now;
  message.angle = currNotch * 0.418879;
  message.avg_angular_velocity = 0.418879 / ((double) (nowArd - lastArd) / 1000);
  lastArd = nowArd;
  
  p.publish(&message);
  nh.spinOnce();
}

 /********************************************************************
 | Routine Name: stateDetector
 | File:         lidarAngleCalc.ino
 | 
 | Description: Uses last light readings to detect changes in state 
 |              robustly.
 |              lastPoints should be up to date before calling function.
 | 
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | return             current state
 ********************************************************************/
bool stateDetector() {
  //if(debug)
  //  Serial.println("Start stateDetector");
  int i, count;

  count = 0;
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

 /********************************************************************
 | Routine Name: pollLight
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads point status of light sensor and stores it in lastPoints
 ********************************************************************/
void pollLight() {
  //if(debug)
  //  Serial.println("Start pollLight");
  int i;

  //shift lastPoints data
  for(i = 0; i < SAME_THRES - 1; i++)
    lastPoints[i] = lastPoints[i + 1];

  //read in new data
  lastPoints[SAME_THRES - 1] = digitalRead(lightPin);

  /*
  lastPoints[SAME_THRES - 1] = debugBool;
  debugInt++;

  if(debugInt == 10) {
    debugInt = 0;
    if(debugBool)
      debugBool = false;
    else
      debugBool = true;
  }
  */

  /*
  for(i = 0; i < SAME_THRES; i++) {
    Serial.print(lastPoints[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
  */
}

 /********************************************************************
 | Routine Name: getAngle
 | File:         lidarAngleCalc.ino
 | 
 | Description: Returns angle using time from TeraRanger
 | 
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | t                  time from TeraRanger
 | rads               whether to return in radians
 | return             angle that corresponds to time
 ********************************************************************/
double getAngle(unsigned long t, boolean rads) {
  //if(debug)
  //  Serial.println("Start getAngle");
  unsigned long higher, lower, intervalTime, tOnInterval;
  int i = 0;
  
  int lowInd;
  /*
  while(i < 15 && t >= times[i])
    i++;

  //TODO handle cases where i = 0, i = 14, i = 15
  //this is really up to how the times array gets updated and when old values get flushed
  
  if(i == 0) {
    higher = 0;
    lower = 14;
    lowInd = 14;
  }
  else if(i == 15) {
    higher = times[14] * 2 - times[13];
    lower = times[14];
    lowInd = 14;
  }
  else {
    higher = times[i + 1];
    lower = times[i];
    lowInd = i;
  }
  */
  
  int intervalIncrease = times[14] - times[13];
  int intervals = 1;
  while(times[14] + intervalIncrease * intervals < t) {
    intervals++;
  }
  
  higher = times[14] + intervalIncrease * intervals;
  lower = times[14];
  lowInd = 14;
  
  //idea is to create interval from higher and lower times, then find where t is on that interval
  intervalTime = higher - lower;
  tOnInterval = higher - t;
  double result = (lowInd + i) * 24 + ((double) tOnInterval / intervalTime) * 24;
  
  if(rads)
    result *= 3.14159265 / 180;
  return result;
}

 /********************************************************************
 | Routine Name: calcAngleInterval
 | File:         lidarAngleCalc.ino
 | 
 | Description: Calculates angle change since last call to this function
 | 
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | rads               whether to return in radians
 | return             angle change since last call
 ********************************************************************/
double calcAngleInterval(boolean rads) {
  //if(debug)
  //  Serial.println("Start calcAngleInterval");
  double angleChange;
  angleChange = getAngle(millis(), rads) + 360 * tries - lastAngle;
  lastAngle = getAngle(millis(), rads);
  return angleChange;
}

/*
double calcAvgChange(double angle, unsigned long time) {
  double avgChange = angle / ((time - lastTime) / 1000);
  lastTime = time;
  return avgChange;
}
*/

