kill_tmux_if_running() {
	tmux has-session -t $1 2>/dev/null
	if [ "$?" -eq 0 ] ; then
		tmux kill-session -t $1
	fi
}

tmux kill-session -t core
tmux kill-session -t tycho_description
kill_tmux_if_running kill-session -t tycho_transform
kill_tmux_if_running rs_camera_435
kill_tmux_if_running rs_camera_415_1
kill_tmux_if_running rs_camera_415_2
kill_tmux_if_running azcam
kill_tmux_if_running mocap
kill_tmux_if_running ball_pub
kill_tmux_if_running azcam_undistort
