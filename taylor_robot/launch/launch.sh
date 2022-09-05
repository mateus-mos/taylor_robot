#!/bin/bash
ros2 launch taylor_robot localization_server.launch.py & 
ros2 launch taylor_robot test.launch.py &