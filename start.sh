source devel/setup.bash
gnome-terminal -- bash -c "roslaunch rviz.launch;read"
sleep 2s
gnome-terminal -- bash -c "rosrun altosradar altosRadarParse"
