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

# For debugging, launch camera without sync (wired_sync_mode:=0 and subordinate_delay_off_master_usec:=0)
#az_launcher "azcam_side" "roslaunch tycho_demo_ros driver_azcam.launch sensor_sn:=000252514612 camera_name:=azcam_side wired_sync_mode:=0 subordinate_delay_off_master_usec:=0 fps:=30 color_resolution:=720P --wait"


az_launcher "azcam_side" "roslaunch tycho_demo_ros driver_azcam.launch sensor_sn:=000252514612 camera_name:=azcam_side wired_sync_mode:=2 subordinate_delay_off_master_usec:=160 fps:=30 color_resolution:=720P --wait"
launcher "azcam_undistort_side" "ROS_NAMESPACE=azcam_side roslaunch tycho_demo_ros az_camera.launch camera:=azcam_side --wait"

sleep 1

az_launcher "azcam_back" "roslaunch tycho_demo_ros driver_azcam.launch sensor_sn:=001059614112 camera_name:=azcam_back wired_sync_mode:=1 fps:=30 color_resolution:=720P --wait"
launcher "azcam_undistort_back" "ROS_NAMESPACE=azcam_back roslaunch tycho_demo_ros az_camera.launch camera:=azcam_back --wait"



az_launcher "azcam_front" "roslaunch tycho_demo_ros driver_azcam.launch sensor_sn:=000704321512 camera_name:=azcam_front wired_sync_mode:=2 subordinate_delay_off_master_usec:=320 fps:=30 color_resolution:=720P --wait"
launcher "azcam_undistort_front" "ROS_NAMESPACE=azcam_front roslaunch tycho_demo_ros az_camera.launch camera:=azcam_front --wait"