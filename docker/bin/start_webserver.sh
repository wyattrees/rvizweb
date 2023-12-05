#!/usr/bin/env bash
source /rvizweb_ws/install/setup.bash

roslaunch ur_gazebo ur5.launch &
sleep 2
roslaunch rvizweb rvizweb.launch
