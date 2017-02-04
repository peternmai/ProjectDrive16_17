 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Feb 02 2017
                                   lidarAngleCalc

 File Name:       lidarAngleCalc.ino
 Description:     Uses light sensor to determine what angle lidar is reading at.
 *****************************************************************************/
//TODO implement threshold before state changes can occur
//#define STATE_CHANGE_THRESHOLD ____

double times[15];   //first times of 15 notches
double lengths[15]; //lengths of 15 notches (in units of time)
double lastAngle;   //last angle returned to ROS
bool lastPoints[5]; //last 5 light readings for smoothing
int currNotch;      //current notch
bool currState;     //state of light

 /********************************************************************
 | Routine Name: setup
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads in 15 notches, initializes some values.
 ********************************************************************/
void setup() {
  int i;

  currState = 0;
  lastAngle = 0;
  currNotch = 0;
  
  recalibrateNotch();
}

 /********************************************************************
 | Routine Name: loop
 | File:         lidarAngleCalc.ino
 | 
 | Description: Sees if recallibration is needed, then 
 ********************************************************************/
void loop() {
  double avgLen;
  int i, minLen, count;

  //calculate average length of notch as a threshold for the small notch
  for(i = 0; i < 15; i++)
    avgLen += lengths[i];
  avgLen /= 15;

  count = 0;
  minLen = -1;

  //try to uniquely determine small notch, and recalibrate until it can be determined.
  while(count != 1) {

    count = 0;
    minLen = -1;
    //if can uniquely determine one notch that is short, set that notch to be the "0" notch
    for(i = 0; i < 15; i++) {
      //finds number of lengths lower than average, if things are working properly should only be 1.
      if(lengths[i] < avgLen) {
        count++;
        minLen = i;
      }
    }

    if(count != 1) {
      currNotch = 0;
      recalibrateNotch();
    }
  }

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
}

 /********************************************************************
 | Routine Name: recalibrateNotch
 | File:         lidarAngleCalc.ino
 | 
 | Description: Reads 15 notches
 ********************************************************************/
void recalibrateNotch() {
  int i;
  
  //initialize all times and lengths to -1
  for(i = 0; i < 15; i++) {
    times[i] = -1;
    lengths[i] = -1;
  }
  
  //fill up times and lengths array
  for(i = 0; i < 15; i++) {
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
  while(stateDetector() == 0) {
      pollLight();
    }     
    
    //store first 1 in times[i] 
    times[currNotch] = getTRTime();
    
    //check to make sure first 1 is reasonably spaced from previous times

    //read until 5 0s are read in a row
    while(stateDetector() == 1) {
      pollLight();
    }

    lengths[currNotch] = getTRTime() - times[currNotch];

    currNotch = (currNotch + 1) % 15;
}

 /********************************************************************
 | Routine Name: stateDetector
 | File:         lidarAngleCalc.ino
 | 
 | Description: Uses last 5 light readings to detect changes in state 
 |              robustly.
 |              lastPoints should be up to date before calling function.
 | 
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | return             current state
 ********************************************************************/
bool stateDetector() {
  int i, count;

  count = 0;
  for(i = 0; i < 5; i++)
    if(lastPoints[i])
      count++;

  //all 5 readings must be 1 or 0 to trigger a state change
  if(count == 0)
    currState = 0;
  else if(count == 5)
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
  //TODO read light sensor

  //TODO store read values into lastPoints

  //requires shifting lastPoints values and adding new one to end
}

 /********************************************************************
 | Routine Name: getTRTime
 | File:         lidarAngleCalc.ino
 | 
 | Description: Returns current time from TeraRanger
 | 
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | return             current time from TeraRanger
 ********************************************************************/
double getTRTime() {
  //TODO return teraRanger time.
  return 0;
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
 | return             angle that corresponds to time
 ********************************************************************/
double getAngle(double t) {
  double higher, lower, intervalTime, tOnInterval;
  int i = 0;
  
  while(t < times[i])
    i++;

  //TODO handle cases where i = 0, i = 14, i = 15
  //this is really up to how the times array gets updated and when old values get flushed
    
  //idea is to create interval from higher and lower times, then find where t is on that interval
  intervalTime = higher - lower;
  tOnInterval = higher - t;
  return lower * 24 + tOnInterval / intervalTime * 24;
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
 | return             angle change since last call
 ********************************************************************/
double calcAngleInterval() {
  double angleChange;
  angleChange = getAngle(getTRTime()) - lastAngle;
  lastAngle = getAngle(getTRTime());
  return angleChange;
}

