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