rosrun teb_local_planner_tutorials cmd_vel_to_ackermann_drive.py &
backgroundPID=$!
sleep 2
rosrun rqt_robot_steering rqt_robot_steering

echo Killing cmd_vel_to_ackermann_drive.py PID=$backgroundPID
kill -9 $backgroundPID
