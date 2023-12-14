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

az_launcher "azcam" "roslaunch azure_kinect_ros_driver driver_azcam_front.launch fps:=30 color_resolution:=720P --wait"
launcher "azcam_undistort" "roslaunch tycho_demo_ros azcam_undistort.launch --wait"
