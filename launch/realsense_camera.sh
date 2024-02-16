#!/bin/zsh

launcher "rs_camera_435" "roslaunch tycho_demo_ros realsense-camera.launch serial_no:=832112070487 camera:=435 --wait"
launcher "rs_camera_415_1" "roslaunch tycho_demo_ros realsense-camera.launch serial_no:=746112060198 camera:=415_1 --wait"
launcher "rs_camera_415_2" "roslaunch tycho_demo_ros realsense-camera.launch serial_no:=747612060067 camera:=415_2 --wait"