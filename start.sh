#!/bin/bash

rm usbList.txt

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL" >> "usbList.txt" 
	
    )
done

optenc=$(grep -m 1 "1a86" usbList.txt | cut -c1-12)
steer=$(grep -m 2 "1a86" usbList.txt | sed '0,/1a86/d' | cut -c1-12)
tera=$(grep "FTDI" usbList.txt | cut -c1-12)

if [ ${optenc} == "" ] || [ ${steer} == "" ] || [ ${tera} == "" ]; then
	echo "ERROR! ONE OR MORE COMPONENTS ARE NOT CONNECTED!"
	exit 1
fi

echo optenc = ${optenc}
echo steer  = ${steer}
echo tera = ${tera}

rm usbList.txt

roslaunch setup test.launch optic:=${optenc} steering:=${steer} tera:=${tera} &
sleep 5
roslaunch navigation_stack hector_slam.launch
