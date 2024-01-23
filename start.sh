source devel/setup.bash
gnome-terminal -- bash -c "roslaunch rviz.launch;read"
sleep 3s
gnome-terminal -- bash -c "rosrun altosradar altosRadarParse"
