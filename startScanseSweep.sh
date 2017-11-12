#!/bin/bash

if [ $# -ne 1 ]; then
  echo "usage:  $0: [rotation_speed]"
  exit -1
fi

echo $1

if [ -f usbList.txt ]; then
  rm usbList.txt
fi

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)" &> /dev/null
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL" >> "usbList.txt" 
	
    )
done

steer=$(grep "1a86" usbList.txt | cut -c1-12)
sweep=$(grep "FTDI_FT230X_Basic_UART_DM00LJRY" usbList.txt | cut -c1-12)

if [ ${steer} == "" ] || [ ${sweep} == "" ]; then
	echo "ERROR! ONE OR MORE COMPONENTS ARE NOT CONNECTED!"
	exit 1
fi

echo steer  = ${steer}
echo sweep = ${sweep}

rm usbList.txt

roscore &
ROSCORE_PID=$1
sleep 1
rosrun setup ActivateScanseSweep.py ${sweep} &
sleep 5
roslaunch sweep_ros sweep2scan.launch portname:=${sweep} rotation_speed:=$1 &
sleep 5
roslaunch setup test2.launch steering:=${steer} &
sleep 5

while true; do
  sleep 1
done

echo Killing all processes launch under PID $$
trap kill -TERM -- $$ EXIT
#roslaunch navigation_stack hector_slam.launch
