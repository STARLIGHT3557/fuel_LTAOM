#!/usr/bin/bash

gnome-terminal --window -e 'bash -c "roslaunch prometheus_gazebo sitl_outdoor_1uav_P600_mid360.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_gazebo 3dlidar_to_octomap.launch; exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch;exit; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch mission_planner click_demo.launch;exit; exec bash"' \