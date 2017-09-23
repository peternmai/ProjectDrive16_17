#!/usr/bin/env python

import sys
import rospy
import serial

if __name__ == '__main__':

  if len(sys.argv) != 2:
    sys.stderr.write("\033[91mERROR: Usage: " + sys.argv[0] + " <port_name>\n\n\033[0m")
    sys.exit(1)

  try:

    # Initiate ROS node
    rospy.init_node('SweepLifeLine', anonymous=True)
    rate = rospy.Rate(1) 

    # Activate Scanse Sweep 360 LIDAR
    sweep = serial.Serial(sys.argv[1],
                      baudrate = 115200,
                      parity=serial.PARITY_NONE,
                      bytesize = serial.EIGHTBITS,
                      stopbits = serial.STOPBITS_ONE,
                      xonxoff = False,
                      rtscts = False,
                      dsrdtr = False)
    
    # Keep Scanse Sweep alive so long as ROS is alive
    print("Successfully activated Scanse Sweep on port: " + sys.argv[1])
    while not rospy.is_shutdown():
      rate.sleep();

    # Close port on successful shutdown
    print("Gracefully closing serial communication with port " + sys.argv[1] + "...")
    sweep.close()
    
  except rospy.ROSInterruptException:
    pass
