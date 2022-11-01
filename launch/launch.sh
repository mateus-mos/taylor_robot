#!/bin/sh

killall -9 -q gzclient & killall -9 -q gzserver # Kill gazebo's process

xterm -title "localization_server" -e "ros2 launch taylor_robot localization_server.launch.py" & 
xterm -title "launch_file" -e "ros2 launch taylor_robot test.launch.py"   
