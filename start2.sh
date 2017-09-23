#!/bin/bash

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

optenc=$(grep -m 1 "1a86" usbList.txt | cut -c1-12)
steer=$(grep -m 2 "1a86" usbList.txt | sed '0,/1a86/d' | cut -c1-12)
tera=$(grep "FTDI_FT232R_USB_UART_AL0339KD" usbList.txt | cut -c1-12)
sweep=$(grep "FTDI_FT230X_Basic_UART_DM00LJRY" usbList.txt | cut -c1-12)

if [ ${optenc} == "" ] || [ ${steer} == "" ] || [ ${tera} == "" ]; then
	echo "ERROR! ONE OR MORE COMPONENTS ARE NOT CONNECTED!"
	exit 1
fi

echo optenc = ${optenc}
echo steer  = ${steer}
echo tera = ${tera}
echo sweep = ${sweep}

rm usbList.txt

roscore &
ROSCORE_PID=$1
sleep 1
rosrun setup ActivateScanseSweep.py ${sweep} &
sleep 5
roslaunch setup test2.launch optic:=${optenc} steering:=${steer} tera:=${tera} sweep:=${sweep} &
sleep 5

while true; do
  sleep 1
done

echo Killing all processes launch under PID $$
trap kill -TERM -- $$ EXIT
#roslaunch navigation_stack hector_slam.launch