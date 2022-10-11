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

launch_mocap() {
	echo "\033[94mPlace Optitrack Points to be at initialization position!\n... then, press enter to continue\033[0m"
	read -n 1 k <&1
	launcher "mocap" "roslaunch \"mocap_optitrack\" \"mocap.launch\" mocap_config_file:=$(rospack find tycho_demo_ros)/launch/mocap.yaml"
}

launch_cam() {
	launcher "azcam" "roslaunch azure_kinect_ros_driver driver_azcam_front.launch fps:=30 color_resolution:=720P --wait"
	launcher "azcam_undistort" "roslaunch tycho_demo_ros undistort_azcam.launch --wait"
	if [ "$1" = "true" ] ; then
		tmux new -d -s ball_pub "python $(rospack find tycho_demo_ros)/../tycho_perception/src/camera_ball_publisher.py"
	fi
}

# 5. ball tracker
launched_tracker="false"
while getopts "aco" flag; do
	case "${flag}" in
		c)
			launched_tracker="true"
			launch_cam "true"
			;;
		o)
			launched_tracker="true"
			launch_mocap
			;;
		a)
			launched_tracker="true"
			launch_cam "false"
			launch_mocap
	esac
done

if [[ "$launched_tracker" != "true" ]]
then
	echo "Defaulting to optitrack"
	launch_mocap
fi

echo "Preparation done; You can view the robot in RViz. "
echo "Ready to launch demo script."
