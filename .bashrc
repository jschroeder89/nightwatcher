################################################
# Your environment can be extended by including
# packages using the RCINFO system.
# Use rcinfo-package-selector for that in favour
# of editing this file!
#
alias cm='catkin_make'
alias nw_launch='roslaunch amiro_gazebo amiro_watchmen_route_project.launch'
alias nw='cd ~/github/nightwatcher/catkin_ws/ && source devel/setup.bash' 
alias ros='cd ~/catkin_ws && source devel/setup.bash'
alias code='cd && ./Downloads/vscode/usr/share/code/code'
################################################
# DO NOT REMOVE STUFF BELOW THIS LINE, OTHERWISE
# YOU WILL BREAK YOUR ENVIRONMENT!
if [ -r /vol/local/etc/startup/profile ]; then
	. /vol/local/etc/startup/profile
fi

