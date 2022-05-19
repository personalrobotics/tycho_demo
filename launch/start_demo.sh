#!/bin/zsh

cd /home/prl/tycho_ws/
source $(catkin locate)/devel/setup.zsh

# 0. tmux launcher
source $(rospack find tycho_demo_ros)/launch/tmux_launcher.sh

# 1. roscore
launcher "core" "roscore"
sleep 2s

# 2. tycho description
pr_ros_launcher "tycho_description" "tycho_description" "robot_chopsticks.launch"

# 3. transform
source $(rospack find tycho_demo_ros)/launch/optitrack_transform.sh

# 4. camera
source $(rospack find tycho_demo_ros)/launch/ros_camera.sh

# 5. mocap
echo "\033[94mPlace Optitrack Points to be at initialization position!\n... then, press enter to continue\033[0m"
read -n 1 k <&1
launcher "mocap" "roslaunch \"mocap_optitrack\" \"mocap.launch\" mocap_config_file:=$(rospack find tycho_demo_ros)/launch/mocap.yaml"

echo "Preparation done; You can view the robot in RViz. "
echo "Ready to launch demo script."
