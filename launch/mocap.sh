#!/bin/zsh

echo "\033[94mPlace Optitrack Points to be at initialization position!\n... then, press enter to continue\033[0m"
read -n 1 k <&1
launcher "mocap" "roslaunch \"mocap_optitrack\" \"mocap.launch\" mocap_config_file:=$(rospack find tycho_demo_ros)/launch/mocap.yaml"