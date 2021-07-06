#!/bin/zsh

function launcher {
    tmux has-session -t $1 2>/dev/null
    if [ "$?" -eq 0 ] ; then
        echo "WARNING! $1 is already running! (tmux session)"
    else
        echo "Creating a new tmux session: $1"
	tmux new-session -d -s $1
	tmux send -t $1 "source $(catkin locate)/devel/setup.zsh && $2$(printf \\r)"
    fi
}

function pr_ros_launcher {
    launcher $1 "roslaunch $2 $3 --wait"
}

cd /home/prl/tycho_ws/
source $(catkin locate)/devel/setup.zsh


# 1. roscore
launcher "core" "roscore"
sleep 2s

# 2. tycho description
pr_ros_launcher "tycho_description" "tycho_description" "robot_chopsticks.launch"

# 3. transform
# Updated 2021 June 24
# If you run thecalibration and obtained an R that transforms optitrack frame to base, it is probably qx qy qz qw x y z
# We expect the transformation to be specified as " ... x y z qx qy qz qw world optitrack " here.
launcher "tycho_transform" "rosrun tf static_transform_publisher -1.07318192 0.17227197 -0.00543 0.00668706 0.00741980 -0.00000018 0.99995011 world optitrack 10"

# 4. camera
#pr_ros_launcher "realsense" "tycho_demo" "realsense-camera.launch"
launcher "rs_camera_435" "roslaunch tycho_demo realsense-camera.launch serial_no:=832112070487 camera:=435 --wait"
launcher "rs_camera_415_1" "roslaunch tycho_demo realsense-camera.launch serial_no:=746112060198 camera:=415_1 --wait"
launcher "rs_camera_415_2" "roslaunch tycho_demo realsense-camera.launch serial_no:=747612060067 camera:=415_2 --wait"

# 5. mocap
echo "\033[94mPlace Optitrack Points to be at initialization position!\n... then, press enter to continue\033[0m"
read -n 1 k <& 1
launcher "mocap" "roslaunch \"mocap_optitrack\" \"mocap.launch\" mocap_config_file:=$(rospack find tycho_teleop)/launch/mocap.yaml"

echo "Preparation done; You can view the robot in RViz. "
echo "Ready to launch demo script."