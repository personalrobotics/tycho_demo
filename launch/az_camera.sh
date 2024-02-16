function az_launcher {
    # Notice that az workspace is different -> Need to source that workspace
    tmux has-session -t $1 2>/dev/null
    if [ "$?" -eq 0 ] ; then
        echo "WARNING! $1 is already running! (tmux session)"
    else
        echo "Creating a new tmux session: $1"
        tmux new-session -d -s $1
        tmux send -t $1 "source /home/prl/azcam_ws/devel/setup.zsh && source $(catkin locate)/devel/setup.zsh --extend && $2$(printf \\r)"
    fi
}

az_launcher "azcam_front" "roslaunch azure_kinect_ros_driver driver_azcam_front.launch fps:=30 color_resolution:=720P --wait"
az_launcher "azcam_side" "roslaunch azure_kinect_ros_driver driver_azcam_side.launch fps:=30 color_resolution:=720P --wait"
az_launcher "azcam_back" "roslaunch azure_kinect_ros_driver driver_azcam_back.launch fps:=30 color_resolution:=720P --wait"
launcher "azcam_undistort_front" "roslaunch tycho_demo_ros azcam_undistort_front.launch --wait"
launcher "azcam_undistort_side" "roslaunch tycho_demo_ros azcam_undistort_side.launch --wait"
launcher "azcam_undistort_back" "roslaunch tycho_demo_ros azcam_undistort_back.launch --wait"
