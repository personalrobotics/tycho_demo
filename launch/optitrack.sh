#!/bin/zsh


# Publish a transformation between the Optitrack frame and the world frame
# If you run the calibration and obtained an R that transforms optitrack frame to the world frame (robot base)
# The R is probably spelled as qx qy qz qw x y z
# Here, we expect the transformation to be specified as " ... x y z qx qy qz qw world optitrack ".
# Note that the height (z) is usually not optimized, but kept constant
launcher "tycho_transform" "rosrun tf static_transform_publisher -0.93919181, 0.16715677, -0.00317523 -0.00162249, 0.00215902, -0.00000002, 0.99999635 world optitrack 10"
# (Last Updated 2022 May 30)

echo "\033[94mPlace Optitrack Points to be at initialization position!\n... then, press enter to continue\033[0m"
read -n 1 k <&1
launcher "mocap" "roslaunch \"mocap_optitrack\" \"mocap.launch\" mocap_config_file:=$(rospack find tycho_demo_ros)/launch/optitrack.yaml"