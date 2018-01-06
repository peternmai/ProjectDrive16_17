---
layout: default
---

# Hardware

![Vehicle Image 1](/assets/img/vehicle_01.jpg)

This page is dedicated to covering the hardware aspects of our vehicle. This
range from vehicle chassis, batteries, all the way to different sensors we
used.

## Topics

* [Back Story and Design Ideology](#back-story-and-design-ideology)
* [Basic Vehicle Setup](#basic-vehicle-setup)
* [Sensors](#sensors)
* [Custom Power Regulator and Circuit Breaker](#custom-power-regulator-and-circuit-breaker)
* [Future Design Changes](#future-design-changes)
* [Hardware Tips](#hardware-tips)

* * *

## Back Story and Design Ideology

When tasked to build a vehicle that would compete in the SparkFun AVC 2017
competition a year ahead of the competition date, the team must finalize on an
official objective / goal the team would strive to achieve. We knew that
SparkFun hosted various annual competitions near the end of summer, one of
which centered around autonomous self-driving race cars. We also had an
impression of what to expect looking through past competitions rules and
race course. But since SparkFun changes their competition year to year, and
tend to not release the official rules/course until a few months before the
actual competition, it was hard to gauge how the course would actually
turn out.

However, we did noticed a growing trend on how SparkFun was changing their
competition year to year. They were adding more obstacles that would not
be stationary, using dirt terrains over concrete for less tractions, and
penalizing teams whom relied heavily on GPS. Over the years, SparkFun had
changed their competition to gradually mimic real life self driving
scenarios, where there were more uncertainties, while at the same time, trying
to stop teams from resorting to [dead
reckoning](https://en.wikipedia.org/wiki/Dead_reckoning), a process of
pre-recording vehicle's movement (wheel rotations and turning angles) and
then replaying it during the actual competition.

From this, we concluded that the best way to prepare ourselves for the
competition was to build a general purpose, fully autonomous, obstacle avoiding
vehicle. This vehicle should be able to navigate and explore any given flat
and confined environment under various lighting conditions, while at the
same time, be able to detect and avoid any obstacles along the way of
varying material. In this sense, the team was not building a race car, but
rather, a general purpose obstacle avoiding vehicle. Since we knew the
SparkFun's race course (along with most other race courses) would be in a
confined environment with only a few paths to choose from, this design would
be ideal for the competition as all the vehicle had to do was continue
exploring until it eventually reach the finish line.

Woo-Hoo! With a plan in mind and an end vision to strive for, let's begin
the next phase of determining the necessary hardware requirements that would
be needed to bring this idea into reality.

* * *

## Basic Vehicle Setup

Every basic 1/10 scale race car, whether it be autonomous or remote control,
are built upon four fundamental building blocks. These building blocks are
(1) vehicle chassis, (2) electronic speed controller, (3) control platform,
and (4) batteries. The setup we had, as well as our reasons for choosing them
are listed below.

#### Vehicle Chassis: Traxxass
We used a [Traxxass](https://traxxas.com/) chassis passed on to us from the
previous year Project Drive team. As the team is given a limited budget from
our sponsors for the project, this was great as it saved us roughly $700
from buying another chassis kit and building it from scratch. Overall, we
had a good experience with the chassis, however, the shock absorber we had
performed poorer than we had hoped. As a result, we had custom 3D printed molds
to help increase the shock absorber's responsiveness.


#### Electronic Speed Controller (ESC): Hobbywing Xerun SCT Pro
The ESC is also a very vital part of the vehicle as it outputs throttle to the
actual wheel and gets the car moving. We used the Hobbywing Xerun SCT Pro,
which was also passed on from the previous year, as our ESC. The great thing
about this ESC was that it could be powered by any 2-4S LiPo battery. However,
we did wish it had much higher torque. Also, a thing to note with this ESC was
that when outputting PWM commands, the speed changed in a rather exponential
fashion, rather than linear. Though we did not use any odometer in our setup,
we would highly encourage future teams to add an odometer to their vehicle,
allowing for more precise control of the vehicle's speed through a
[closed-loop system](https://en.wikipedia.org/wiki/Control_theory). An odometer
would also significantly increase the reliability of any spatial mapping
and localization (SLAM) algorithm. For those interested in controlling their
ESC in the ROS environment, we have developed a ROS Arduino module that
can subscribe to [Ackermann ROS messages](http://wiki.ros.org/ackermann_msgs)
and output the appropriate PWM commands to the ESC
[here](https://github.com/peternmai/ProjectDrive16_17/blob/master/Navigation/src/steering/ackermann_steering/ackermann_steering.ino).

#### Control Platform: Nvidia Jetson TX1 and Arduino Nano
The control platform is very much what gives the vehicle life. It allows the
vehicle's software to interface with much of the hardware connected to the
vehicle. We chose the Nvidia Jetson TX1 as our main computing unit as it
provided us with the power of a small computer, while at the same time,
running on the linux environment. This would allow us to communicate to
our vehicle wirelessly through SSH, as well as visualize OpenGL applications
(e.g. RViz), through services such as VNC server. Furthermore, it supported
the full ROS platform in which we would be using to build our application
upon. We also used the Arduino Nano as it was also compatible with ROS
while allowing a quick and easy way to send out servo commands and
interpret raw sensor data from our custom 360 LIDAR setup.

#### Batteries: Two EcoPower LiPo Batteries
No vehicle would be complete without some way of powering it! As we would be
developing and testing directly on the vehicle, we needed some way to power
the system for an extended period of time, while allowing our vehicle to
move around freely. We therefore resorted to using two batteries in our setup.
We used one EcoPower "Electron" 2S Li-Poly 30C hard case battery pack to power
the ESC. We then used another EcoPower "Electron" 4S Li-Poly 35C hard case
battery pack to power the on board control platform and sensors. With our set
up, the 2S LiPo battery would last roughly two hours of continuous driving
while the 4s LiPo battery could power the Nvidia Jetson and all the sensors
for roughly four hours.

* * *

## Sensors

Sensor is the key element to any autonomous system. Sensors allow the system
to interpret and understand the environment around it, and therefore,
enable it to respond back in an appropriate manner.

Since we were building a general purpose obstacle avoiding race car that would
be navigating in a flat environment, all we really needed to accomplish this
was a 2D map of the car's environment. Being on a budget, what other way would
be more cost efficient while providing all these valuable information than
a 2D, 360 degree, depth sensor! The sensors we used are listed below.

#### 360 LIDAR: Custom Build [NOT USED IN FINAL PRODUCT]
Initially, the team wanted to build most of the hardware from scratch to get
more practical skills and develop a deeper understanding in how sensors worked.
We ended up building our custom 360 LIDAR sensors from scratched for most of the
year, as described in the timeline, and it did fulfill our desires. We learned a
tons on the inner workings of sensors. We had a much clearer pictures on the
challenges of signal processing, how to reduce noise, and how there are many
factors that affected the performance of the sensors.

To give a brief overview, we used the [TeraRanger
One](http://www.teraranger.com/products/), a pseudo 1D LIDAR sensor, to retrieve
range readings. We then attached the sensor on a 3D printed spinning disk
that had tick marks underneath. We used a photosensor on those tick marks to
get an understanding of which angle the sensor was facing. We then tweaked
the TeraRanger's ROS source code to fit our needs, and used this setup to
generate a 2D map of its surrounding, which we then outputted as a [ROS
LaserScan Message](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html).
The Arduino code to determine angle from the photosensor can be found
[here](https://github.com/peternmai/ProjectDrive16_17/blob/master/Sensor/src/Arduino/lidarAngleCalc/lidarAngleCalc.ino),
while the code that subscribed to the angles and range readings to then output
a 2D LaserScan map message can be found
[here](https://github.com/peternmai/ProjectDrive16_17/blob/master/Navigation/src/setup/src/LaserScanPublisher.cpp).

![Custom 360 LIDAR with SLAM](/assets/img/custom-360-lidar-with-slam.png)

This worked! Or.. ehh.. it worked only indoor. We made one fatal mistake when
building our custom 360 LIDAR. We knew we needed a LIDAR because it has been
proven to work well in various lighting condition and could detect various
material type. We also needed one with a very fast refresh rate as it was
critical in generating a clear and dense 2D map when spinning the sensor at
a high rate. The TeraRanger One's specification on its website matched both
of these conditions, and even had two versions, one which was recommended
better for outdoor usage. We decided to go for it, however, we later discovered
that its usage of infrared as its LIDAR sensor had very poor performance
outdoor. This discovery came when we finally had a reliable working indoor
custom 360 LIDAR setup at the end of spring quarter. With summer approaching
and most of the team going separate ways for internships, it was not feasible
for us to develop a brand new custom 360 LIDAR setup from scratch in time
for the competition. Therefore, we ended up using a new off the self 360
LIDAR sensor as described below.

We were sad to see that our year's worth of tuning and perfecting our custom
360 LIDAR setup did not make to the final version of the car. However, we
were very grateful to have the opportunity to undergo such task. The sensor does
work very well indoor, outputting very precise and reliable 2D maps at a rate of
five times per second (limited by motor choice). But as with any engineering
feat, we do not always stumble upon great innovation on our first attempt.
Failure is one of the many keys to success, and with every mistake we make,
comes great lessons that will spark new and improve innovations.

#### 360 LIDAR: Scanse Sweep
The [Scanse Sweep](http://scanse.io/) was a brand new 2D 360 LIDAR that came
out in 2017, claiming to have a detection range of 40 meters, sample rate of 1000
samples/sec, and a rotational frequency of up to 10 Hz for only $349.00. The
specification was great, matching what we were looking for as described in our
custom 360 LIDAR setup, and the price were very competitive compared to what other
companies were offering for the same price. From our experience, this sensor
worked phenomenally both indoor and outdoor with very accurate map reading,
whether we were spinning the sensor at 1 Hz or 10 Hz. Expect range to not
actually be 40 meters outdoor, closer to 20 meters, but that was much more
than what we needed for the competition. We did noticed that since our vehicle
vibrated quite a bit, prolong usage of the sensor would result in slightly
fuzzier map. However, stopping and restarting the sensor would recalibrate
the sensor and the map would be back to normal. We would highly recommend
this sensor for future teams looking for a good 2D 360 LIDAR sensor that
can operate well in various environment.

*Note: The Scanse Sweep requires that we manually open a serial port connection
with ROS before we can actually receive any data from it when running on the
Nvidia Jetson. If you are having problems reading in data using the default
[ROS package](https://github.com/scanse/sweep-ros) developed by the
manufacturer, we have developed a ROS node which you can use to establish
and maintain a serial connection with the Scanse Sweep which can be found
[here](https://github.com/peternmai/ProjectDrive16_17/blob/master/Navigation/src/setup/src/ActivateScanseSweep.py).
Just run this node before attempting to read in any data.*

#### IMU: Adafruit BNO055
Inertial measurement units (IMU) are often found on various autonomous system
as it allowed the system to determine the precise orientation it is in. We
initially didn't plan on using an IMU as a 360 LIDAR would be enough to allow
us to navigate and avoid obstacles. However, we added an IMU for redundancy
in case our algorithm failed to detect that our vehicle has turned 180 degree
and is now going the wrong way. Though this was the primary intention of using
the IMU, it proved to be much more helpful at the actual competition. As we
found out at the competition that the course was not all flat as it had stated in
the course description. It included ramps, which would for one be higher than
our 360 LIDAR 2D plane causing our vehicle to think it's a wall, and two,
cause our open loop throttle control to not detect an upward ramp and
increase throttle power. Overall, we believe that using an IMU is an essential
element to any autonomous system as it allows for redundancy and improved
interpretation of the system's position/orientation in respect to its
surrounding environment.

* * *

## Custom Power Regulator and Circuit Breaker

![Power Regulator](/assets/img/power-regulator-all.jpg)

When designing our custom power regulator, we wanted it to satisfy two conditions.
Since we were powering our setup using LiPo batteries, we (1) needed some way to
ensure that the batteries were always operating at the ideal capacity and
would never discharge too low as it would wreck havoc on the battery. We
also (2) needed a constant 12V output to power both the Nvidia Jetson
and all the additional sensors.

Therefore, we designed and built our very own power regulator and circuit
breaker from the ground up to meet our needs. This custom PCB would receive
power from the 4S LiPo battery and constantly monitor the remaining juice in
it. It would output a steady 12V to be used by the control platform and
sensors, and have a power kill switch which would activate once the battery
reaches too low a discharge. There were two LEDs on the board, indicating green
if the battery is in a good state, and red when the battery is nearing its
cut off state, or has exceeded its cut off state.

* * *

## Future Design Changes
For future design changes, please refer to our
[reflection page](reflection#future-design-changes).

* * *

## Hardware Tips
Work in progress
